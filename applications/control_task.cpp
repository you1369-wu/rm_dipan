// 四台 M3508 麦克纳姆底盘控制：失能、遥控运动和反馈接收。
#include "control_task.hpp"

#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
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
float chassis_vx_set = 0.0f;
float chassis_vy_set = 0.0f;
float chassis_wz_set = 0.0f;
float wheel_pid_p[4] = {};
float wheel_pid_i[4] = {};
float wheel_pid_d[4] = {};
volatile bool chassis_remote_online = false;
volatile bool chassis_motors_online = false;
volatile bool chassis_right_switch_mid = false;

namespace
{
constexpr float kMaxVx = 0.30f;                        // 前后最大目标速度为 0.30 m/s
constexpr float kMaxVy = 0.30f;                        // 左右最大目标速度为 0.30 m/s
constexpr float kStickDeadband = 0.05f;                // 摇杆死区
constexpr float kDialDeadband = 0.10f;                 // 拨轮死区
constexpr float kMaxWheelSpeed = 12.0f;                // 每个轮子目标角速度上限 rad/s
constexpr float kAcceleration = 0.5f;                  // 平移目标加速度 m/s^2
constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(1);  // 控制周期 1 ms
static_assert(configTICK_RATE_HZ == 1000, "This controller requires 1 kHz RTOS tick");

// 轮半径、前后半间距、左右半间距；右侧轮的转向按当前安装方向取反。
sp::Mecanum chassis(0.077f, 0.165f, 0.185f, false, false, true, true);

//                           dt      kp     ki    kd    mo     mio    alpha  ang?   dynamic?
//需要对mo，mio进行限幅
sp::PID motor_lf_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_lr_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_rf_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);
sp::PID motor_rr_pid_speed(0.001f, 0.08f, 0.0f, 0.0f, 0.50f, 0.10f, 1.0f, false, true);

sp::PID * const wheel_pid[4] = {
  &motor_lf_pid_speed, &motor_lr_pid_speed, &motor_rf_pid_speed, &motor_rr_pid_speed};

// 目标轮速按左前、左后、右前、右后排列，单位 rad/s。
// 读取并检查四轮反馈后，运行各自的单速度环 PID，再统一发送控制帧。
bool run_wheel_speed_control(const float target[4])
{
  float feedback[4] = {};
  bool feedback_valid = true;
  const uint32_t now_ms = osKernelSysTick();
  taskENTER_CRITICAL();
  for (unsigned i = 0; i < 4U; ++i) {
    feedback[i] = wheels[i].motor->speed;
    if (!wheels[i].motor->is_alive(now_ms) || !std::isfinite(feedback[i])) {
      feedback_valid = false;
    }
  }
  taskEXIT_CRITICAL();

  chassis_motors_online = feedback_valid;
  if (!feedback_valid) return false;

  for (unsigned i = 0; i < 4U; ++i) {
    wheels[i].data.target_speed_set = target[i];
    wheel_pid[i]->calc(wheels[i].data.target_speed_set, feedback[i]);
    wheels[i].data.given_torque = wheel_pid[i]->out;
    wheel_pid_p[i] = wheel_pid[i]->data.pout;
    wheel_pid_i[i] = wheel_pid[i]->data.iout;
    wheel_pid_d[i] = wheel_pid[i]->data.dout;
  }

  // 发送前复查，防止计算期间遥控器失联或电机反馈超时。
  taskENTER_CRITICAL();
  RemoteTaskData latest = {};
  const uint32_t send_ms = osKernelSysTick();
  bool command_allowed =
    remote_task_get_data(&latest) && latest.is_alive && latest.sw_r == REMOTE_SWITCH_MID;
  for (auto & wheel : wheels) {
    if (!wheel.motor->is_alive(send_ms) || !std::isfinite(wheel.motor->speed)) {
      command_allowed = false;
      chassis_motors_online = false;
    }
  }

  if (command_allowed) {
    for (auto & byte : can1.tx_data) {
      byte = 0U;
    }
    for (auto & wheel : wheels) {
      wheel.motor->cmd(wheel.data.given_torque);
      wheel.motor->write(can1.tx_data);
    }
    can1.send(motor_lf.tx_id);
  }
  taskEXIT_CRITICAL();

  return command_allowed;
}

void run_disable_mode()
{
  chassis_motors_online = false;
  chassis_vx_set = 0.0f;
  chassis_vy_set = 0.0f;
  chassis_wz_set = 0.0f;

  for (unsigned i = 0; i < 4U; ++i) {
    wheels[i].data = {};
    wheel_pid[i]->clear();
    wheel_pid_p[i] = 0.0f;
    wheel_pid_i[i] = 0.0f;
    wheel_pid_d[i] = 0.0f;
    wheels[i].motor->cmd(0.0f);
  }

  // 四台电机的命令槽均写入零力矩，再发送共同的 0x200 控制帧。？？？实际按照电调修改，ID5-8对应0x1FF
  for (auto & byte : can1.tx_data) {
    byte = 0U;
  }
  for (auto & wheel : wheels) {
    wheel.motor->write(can1.tx_data);
  }
  can1.send(motor_lf.tx_id);
}
}  // namespace

extern "C" void control_task(void const * argument)
{
  (void)argument;
  can1.config();
  can1.start();
  run_disable_mode();

  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_cycle_ms = osKernelSysTick();

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    const uint32_t elapsed_ms = now_ms - last_cycle_ms;
    last_cycle_ms = now_ms;
    RemoteTaskData remote_data = {};
    taskENTER_CRITICAL();
    const bool remote_ready = remote_task_get_data(&remote_data);
    taskEXIT_CRITICAL();

    //检查遥控器输入合法性
    const bool input_valid =
      std::isfinite(remote_data.ch_lv) && std::isfinite(remote_data.ch_lh) &&
      std::isfinite(remote_data.ch_lu) && std::fabs(remote_data.ch_lv) <= 1.0f &&
      std::fabs(remote_data.ch_lh) <= 1.0f && std::fabs(remote_data.ch_lu) <= 1.0f;
    const bool remote_online = remote_ready && remote_data.is_alive;
    const bool right_switch_mid = remote_data.sw_r == REMOTE_SWITCH_MID;  //检查遥控器是否在中档
    chassis_remote_online = remote_online;
    chassis_right_switch_mid = remote_online && right_switch_mid;

    // 右拨杆中档直接允许运动，下档/上档、失联或反馈异常均立即失能
    const bool motion_allowed =
      remote_online && right_switch_mid && input_valid && elapsed_ms <= 5U;
    if (!motion_allowed) {
      run_disable_mode();
    }
    else {
      //进入运动模式
      //开环测试
      // for (auto & byte : can1.tx_data) byte = 0U;
      // for (unsigned i = 0; i < 4U; ++i) {
      //   wheels[i].motor->cmd(i == 3U ? 0.5f : 0.0f);
      //   wheels[i].motor->write(can1.tx_data);
      // }
      // can1.send(motor_lf.tx_id);
      //遥控器左摇杆映射到速度（摇杆死区后重新缩放。x 向前，y 向左。）
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

      // 拨轮映射到旋转速度（拨轮死区外定速旋转：正=逆时针，负=顺时针。）
      chassis_wz_set = std::fabs(remote_data.ch_lu) <= kDialDeadband
                         ? 0.0f
                         : static_cast<float>(sp::sgn(remote_data.ch_lu));

      chassis.calc(chassis_vx_set, chassis_vy_set, chassis_wz_set);  //麦轮运动学解算
      float target[4] = {chassis.speed_lf, chassis.speed_lr, chassis.speed_rf, chassis.speed_rr};
      float peak = 0.0f;
      for (float speed : target) {
        if (std::fabs(speed) > peak) peak = std::fabs(speed);
      }
      const float scale = peak > kMaxWheelSpeed ? kMaxWheelSpeed / peak : 1.0f;
      for (unsigned i = 0; i < 4U; ++i) {
        target[i] *= scale;
      }

      if (!run_wheel_speed_control(target)) {
        run_disable_mode();
      }
    }

    vTaskDelayUntil(&last_wake, kPeriodTicks);
  }
}

//CAN 接收回调函数:更新四轮反馈
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan != &hcan1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef header = {};
    uint8_t data[8] = {};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }

    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8U) {
      continue;
    }

    const uint32_t stamp_ms = osKernelSysTick();
    for (auto & wheel : wheels) {
      if (header.StdId == wheel.motor->rx_id) {
        wheel.motor->read(data, stamp_ms);
      }
    }
  }
}
