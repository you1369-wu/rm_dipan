// 四台 M3508 麦克纳姆底盘控制：失能、遥控运动和反馈接收。
#include "control_task.hpp"

#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "imu_task.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "pid_train.hpp"
#include "remote_task.h"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

extern sp::CAN can1;
extern sp::RM_Motor motor_lf;
extern sp::RM_Motor motor_lr;
extern sp::RM_Motor motor_rf;
extern sp::RM_Motor motor_rr;

ChassisWheel wheels[4] = {{&motor_lf, {}}, {&motor_lr, {}}, {&motor_rf, {}}, {&motor_rr, {}}};

// 保守调试初值，不是已验收的参数。
constexpr float kMaxVx = 0.30f;                        // 前后最大目标速度为 0.30 m/s
constexpr float kMaxVy = 0.30f;                        // 左右最大目标速度为 0.30 m/s
constexpr float kStickDeadband = 0.05f;                // 摇杆死区，0.05 对应 5% 的最大速度
constexpr float kDialDeadband = 0.10f;                 // 拨轮死区，0.10 对应 10% 的最大角速度
constexpr float kMaxWheelSpeed = 12.0f;                // 轮轴 rad/s
constexpr float kMaxTorque = 0.50f;                    // 轮轴 N·m
constexpr float kAcceleration = 0.5f;                  // 平移目标加速度 m/s²
constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(1);  // 控制周期 1 ms
static_assert(configTICK_RATE_HZ == 1000, "This controller requires 1 kHz RTOS tick");

// 创建麦轮解算对象
// 半径、前后半间距、左右半间距；方向参数必须按实车校准。
sp::Mecanum chassis(0.077f, 0.165f, 0.185f, false, false, true, true);

//                           dt      kp     ki    kd    mo     mio    alpha  ang?   dynamic?
sp::PID motor_lf_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_lr_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_rf_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_rr_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);

// 指针数组引用上述四个实际控制器，不另外创建 PID。
sp::PID * const wheel_pid[4] = {
  &motor_lf_pid_speed, &motor_lr_pid_speed, &motor_rf_pid_speed, &motor_rr_pid_speed};

// 目标速度和 PID 输出缓存
float chassis_vx_set = 0.0f;
float chassis_vy_set = 0.0f;
float chassis_wz_set = 0.0f;
float wheel_pid_p[4] = {};
float wheel_pid_i[4] = {};
float wheel_pid_d[4] = {};
volatile uint32_t disable_can_enqueue_errors = 0U;  //错误计数
volatile uint32_t disable_can_mailbox_busy = 0U;
volatile bool chassis_remote_online = false;
volatile bool chassis_motors_online = false;
volatile bool chassis_right_switch_mid = false;

// 全局状态
bool motion_active = false;
// 失能状态：清掉运动状态并持续发送零命令
void run_disable_mode()
{
  // 从运动转入失能时，取消排队中的运动帧
  if (motion_active) {
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;

    if (HAL_CAN_AbortTxRequest(&hcan1, mailboxes) != HAL_OK) {
      ++disable_can_enqueue_errors;
    }

    motion_active = false;
  }

  // 清车体目标速度
  chassis_vx_set = 0.0f;
  chassis_vy_set = 0.0f;
  chassis_wz_set = 0.0f;

  // 清四轮控制状态、PID 和电机内部命令
  for (unsigned i = 0; i < 4U; ++i) {
    wheels[i].data = {};
    wheel_pid[i]->clear();
    wheels[i].motor->cmd(0.0f);

    wheel_pid_p[i] = 0.0f;
    wheel_pid_i[i] = 0.0f;
    wheel_pid_d[i] = 0.0f;
  }

  // C620：ID1—4 对应 0x200，ID5—8 对应 0x1FF
  constexpr uint16_t groups[] = {0x200, 0x1FF};
  static unsigned next_group = 0U;

  for (unsigned attempt = 0; attempt < 2U; ++attempt) {
    // 整帧清零，无需逐台 write()
    for (auto & byte : can1.tx_data) {
      byte = 0U;
    }

    // 邮箱满时，下周期继续尝试当前组
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
      ++disable_can_mailbox_busy;
      break;
    }

    // 原中间件 send() 返回 void，不接收返回值
    can1.send(groups[next_group]);

    next_group = (next_group + 1U) % 2U;
  }
}

// 控制任务
extern "C" void control_task(void const * argument)
{
  (void)argument;
  can1.config();
  can1.start();
  run_disable_mode();

  bool armed = false;                          // 上电或故障后，必须先经过有效下档。
  TickType_t last_wake = xTaskGetTickCount();  // 记录固定周期调度的基准时间
  uint32_t last_cycle_ms = osKernelSysTick();  // 记录上次周期的系统时钟，单位 ms

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    const uint32_t elapsed_ms = now_ms - last_cycle_ms;
    last_cycle_ms = now_ms;
    RemoteTaskData remote_data = {};
    float feedback[4] = {};  // 四轮实际速度
    bool motors_ready = true;
    bool feedback_finite = true;

    // 与 UART/CAN 接收中断建立一致快照。
    taskENTER_CRITICAL();
    const bool remote_ready =
      remote_task_get_data(&remote_data);  // 读取遥控器快照，并取得是否可用的结果
    for (unsigned i = 0; i < 4U; ++i) {
      feedback[i] = wheels[i].motor->speed;  // 保存当前轮子的实际轮轴角速度，单位 rad/s
      if (!wheels[i].motor->is_alive(now_ms)) motors_ready = false;
      if (!std::isfinite(feedback[i])) feedback_finite = false;
    }
    taskEXIT_CRITICAL();

    // 检查遥控器输入是否有效，摇杆和拨轮必须是有限数且在 [-1, 1] 范围内
    const bool input_valid =
      std::isfinite(remote_data.ch_lv) && std::isfinite(remote_data.ch_lh) &&
      std::isfinite(remote_data.ch_lu) && std::fabs(remote_data.ch_lv) <= 1.0f &&
      std::fabs(remote_data.ch_lh) <= 1.0f && std::fabs(remote_data.ch_lu) <= 1.0f;
    chassis_remote_online = remote_ready && remote_data.is_alive;
    chassis_motors_online = motors_ready && feedback_finite;
    chassis_right_switch_mid = chassis_remote_online && remote_data.sw_r == REMOTE_SWITCH_MID;

    if (!chassis_remote_online || !input_valid || !chassis_motors_online || elapsed_ms > 5U) {
      armed = false;
      run_disable_mode();
    }
    else if (remote_data.sw_r == REMOTE_SWITCH_DOWN) {
      armed = true;  // 不代表立即运动，只表示下一次切到中档可以进入运动控制
      run_disable_mode();
    }
    else if (remote_data.sw_r != REMOTE_SWITCH_MID || !armed) {
      armed = false;
      run_disable_mode();
    }
    else {
      // 摇杆死区后重新缩放。x 向前，y 向左。
      const float lv = remote_data.ch_lv;
      const float lh = remote_data.ch_lh;
      const float vx_target =
        std::fabs(lv) <= kStickDeadband
          ? 0.0f
          : sp::sgn(lv) * (std::fabs(lv) - kStickDeadband) / (1.0f - kStickDeadband) * kMaxVx;
      const float vy_target =
        std::fabs(lh) <= kStickDeadband
          ? 0.0f
          : -sp::sgn(lh) * (std::fabs(lh) - kStickDeadband) / (1.0f - kStickDeadband) * kMaxVy;
      const float dt = elapsed_ms > 0U ? elapsed_ms * 0.001f : 0.001f;
      const float max_step = kAcceleration * dt;
      chassis_vx_set += sp::limit_max(vx_target - chassis_vx_set, max_step);
      chassis_vy_set += sp::limit_max(vy_target - chassis_vy_set, max_step);

      // 拨轮死区外定速旋转：正=逆时针，负=顺时针。
      // 拨轮正负与实车方向须调试确认。
      chassis_wz_set = std::fabs(remote_data.ch_lu) <= kDialDeadband
                         ? 0.0f
                         : static_cast<float>(sp::sgn(remote_data.ch_lu));

      // 麦轮解算和同比限速
      chassis.calc(
        chassis_vx_set, chassis_vy_set, chassis_wz_set);  // 车体目标速度转换成四轮目标角速度
      float target[4] = {
        chassis.speed_lf, chassis.speed_lr, chassis.speed_rf,
        chassis.speed_rr};  // 保存解算结果，单位 rad/s
      float peak = 0.0f;    // 查找四轮目标速度的最大绝对值
      for (float speed : target) {
        if (std::fabs(speed) > peak) peak = std::fabs(speed);
      }
      const float scale =
        peak > kMaxWheelSpeed ? kMaxWheelSpeed / peak : 1.0f;  // 超上限时计算缩放比例，否则比例为 1
      for (unsigned i = 0; i < 4U; ++i) {
        wheels[i].data.target_speed_set = target[i] * scale;
        wheel_pid[i]->calc(wheels[i].data.target_speed_set, feedback[i]);
        wheels[i].data.given_torque = sp::limit_max(wheel_pid[i]->out, kMaxTorque);
        wheel_pid_p[i] = wheel_pid[i]->data.pout;
        wheel_pid_i[i] = wheel_pid[i]->data.iout;
        wheel_pid_d[i] = wheel_pid[i]->data.dout;
      }

      // 发送前重新读取安全状态；禁止接收中断在检查和入队间更新。
      bool send_ok = true;
      taskENTER_CRITICAL();
      RemoteTaskData latest = {};
      const uint32_t send_ms = osKernelSysTick();
      send_ok =
        remote_task_get_data(&latest) && latest.is_alive && latest.sw_r == REMOTE_SWITCH_MID;
      for (auto & wheel : wheels) {
        if (!wheel.motor->is_alive(send_ms)) send_ok = false;
      }
      if (send_ok) {
        motion_active = true;
        for (unsigned i = 0; i < 4U; ++i) {
          wheels[i].motor->cmd(wheels[i].data.given_torque);
        }
        // 按实际 tx_id 分组，每组只发一次。
        // 按 tx_id 分组，同一组只发送一次
        for (unsigned i = 0; i < 4U && send_ok; ++i) {
          const uint16_t group = wheels[i].motor->tx_id;

          bool already_sent = false;

          for (unsigned j = 0; j < i; ++j) {
            if (wheels[j].motor->tx_id == group) {
              already_sent = true;
            }
          }

          if (already_sent) {
            continue;
          }

          // 清空发送缓冲区
          for (auto & byte : can1.tx_data) {
            byte = 0U;
          }

          // 写入当前组的电机命令
          for (auto & wheel : wheels) {
            if (wheel.motor->tx_id == group) {
              wheel.motor->write(can1.tx_data);
            }
          }

          // 邮箱满，退出并由后续代码进入失能
          if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
            ++disable_can_mailbox_busy;
            send_ok = false;
            break;
          }

          // 原 send() 返回 void，不能赋值给 status
          can1.send(group);
        }
      }
      taskEXIT_CRITICAL();
      if (!send_ok) {
        armed = false;
        run_disable_mode();
      }
    }
    vTaskDelayUntil(&last_wake, kPeriodTicks);
  }
}
// CAN 接收回调：更新四轮反馈
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};

    // 直接读取一帧，取得帧头和数据
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }

    // 电机反馈必须是标准帧、数据帧、8 字节
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8U) {
      continue;
    }

    const uint32_t stamp_ms = osKernelSysTick();

    // 按反馈 ID 更新对应电机
    for (auto & wheel : wheels) {
      if (header.StdId == wheel.motor->rx_id) {
        wheel.motor->read(
          data,
          stamp_ms);  // 调用中间件解析反馈，更新：轮轴角度、轮轴角速度、电流换算的估算转矩、温度、最近反馈时间
      }
    }
  }
}
