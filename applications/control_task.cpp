// 四台 M3508 麦克纳姆底盘控制：失能、遥控运动和反馈接收。
#include "control_task.hpp"

#include <cmath>

#include "can.h"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee.hpp"
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
float chassis_vx_set = 0.0f;  // 车体前后速度，m/s
float chassis_vy_set = 0.0f;  // 车体横向速度，m/s
float chassis_wz_set = 0.0f;  // 车体旋转角速度，rad/s
float wheel_pid_p[4] = {};
float wheel_pid_i[4] = {};
float wheel_pid_d[4] = {};
volatile bool chassis_remote_online = false;
volatile bool chassis_motors_online = false;
volatile bool chassis_right_switch_mid = false;
volatile float chassis_predicted_power_w = 0.0f;
volatile float chassis_real_power_w = 0.0f;
volatile float chassis_power_limit_w = 0.0f;
volatile float chassis_power_scale = 1.0f;
ControlSource control_source = ControlSource::MANUAL;
mission::Status mission_status{};

namespace
{
// 只生成自动速度目标，四轮 PID、功率保护和 CAN 仍由本任务负责。
mission::Controller mission_controller;

constexpr float kMaxVx = 4.0f;               // 前后最大目标速度为 3.5 m/s
constexpr float kMaxVy = 4.0f;               // 左右最大目标速度为 3.5 m/s
constexpr float kStickDeadband = 0.05f;      // 摇杆死区
constexpr float kDialDeadband = 0.10f;       // 拨轮死区
constexpr float kSpinWz = 1.0f;              // 小陀螺目标角速度大小，rad/s；旋转方向由拨轮决定
constexpr float kMaxWheelSpeedRadS = 24.0f;  // 每个轮子目标角速度上限，rad/s
constexpr float kAcceleration = 0.5f;        // 平移目标加速度 m/s^2
// 功率模型经验初值，需根据功率计实测数据重新标定。
constexpr float kPowerK1 = 2.6f;
constexpr float kPowerK2 = 0.006f;
constexpr float kPowerK3 = 4.0f;
constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(1);  // 控制周期 1 ms
static_assert(configTICK_RATE_HZ == 1000, "This controller requires 1 kHz RTOS tick");

// 轮半径、前后半间距、左右半间距；右侧轮的转向按当前安装方向取反。
constexpr float kWheelRadiusM = 0.077f;
sp::Mecanum chassis(kWheelRadiusM, 0.165f, 0.185f, false, false, true, true);

//                           dt      kp     ki    kd    mo     mio    alpha  ang?   dynamic?
//需要对mo，mio进行限幅
sp::PID motor_lf_pid_speed(0.001f, 0.6f, 0.3f, 0.0f, 3.0f, 0.5f, 1.0f, false, true);
sp::PID motor_lr_pid_speed(0.001f, 0.6f, 0.3f, 0.0f, 3.0f, 0.5f, 1.0f, false, true);  //can id 为1
sp::PID motor_rf_pid_speed(0.001f, 0.6f, 0.3f, 0.0f, 3.0f, 0.5f, 1.0f, false, true);
sp::PID motor_rr_pid_speed(0.001f, 0.6f, 0.3f, 0.0f, 3.0f, 0.5f, 1.0f, false, true);

sp::PID * const wheel_pid[4] = {
  &motor_lf_pid_speed, &motor_lr_pid_speed, &motor_rf_pid_speed, &motor_rr_pid_speed};

// 输入：遥控快照；输出：摇杆/拨轮和开关是否合法，两种控制来源共用。
bool valid_remote_input(const RemoteTaskData & remote)
{
  const bool switches_valid =
    (remote.sw_r == REMOTE_SWITCH_DOWN || remote.sw_r == REMOTE_SWITCH_MID ||
     remote.sw_r == REMOTE_SWITCH_UP) &&
    (remote.sw_l == REMOTE_SWITCH_DOWN || remote.sw_l == REMOTE_SWITCH_MID ||
     remote.sw_l == REMOTE_SWITCH_UP);
  return switches_valid && std::isfinite(remote.ch_lv) && std::isfinite(remote.ch_lh) &&
         std::isfinite(remote.ch_lu) && std::fabs(remote.ch_lv) <= 1.0f &&
         std::fabs(remote.ch_lh) <= 1.0f && std::fabs(remote.ch_lu) <= 1.0f;
}

// 无输入；清除车体目标和四轮积分，不发送 CAN，供切换、停车和失能共用。
void clear_motion_commands()
{
  chassis_vx_set = chassis_vy_set = chassis_wz_set = 0.0f;
  for (unsigned i = 0; i < 4U; ++i) {
    wheels[i].data = {};
    wheel_pid[i]->clear();
    wheel_pid_p[i] = wheel_pid_i[i] = wheel_pid_d[i] = 0.0f;
    wheels[i].motor->cmd(0.0f);
  }
}

// 用四轮期望转矩和实际轮轴转速预测底盘输入功率
float predict_chassis_power(
  const float torque[4], const float omega[4], float k1, float k2, float k3)
{
  float power = k3;
  for (unsigned i = 0; i < 4U; ++i) {
    power += torque[i] * omega[i] + k1 * torque[i] * torque[i] + k2 * omega[i] * omega[i];
  }
  return power;
}

// 目标轮速按左前、左后、右前、右后排列，单位 rad/s。
// 读取并检查四轮反馈后，运行各自的单速度环 PID，再统一发送控制帧。
bool run_wheel_speed_control(const float target_rad_s[4])
{
  float feedback_rad_s[4] = {};
  bool feedback_valid = true;
  const uint32_t now_ms = osKernelSysTick();
  taskENTER_CRITICAL();
  for (unsigned i = 0; i < 4U; ++i) {
    feedback_rad_s[i] = wheels[i].motor->speed;
    if (!wheels[i].motor->is_alive(now_ms) || !std::isfinite(feedback_rad_s[i])) {
      feedback_valid = false;
    }
  }
  taskEXIT_CRITICAL();

  chassis_motors_online = feedback_valid;
  if (!feedback_valid) return false;

  float torque[4] = {};
  for (unsigned i = 0; i < 4U; ++i) {
    wheels[i].data.target_speed_set = target_rad_s[i];
    wheel_pid[i]->calc(wheels[i].data.target_speed_set, feedback_rad_s[i]);
    wheels[i].data.given_torque = wheel_pid[i]->out;
    torque[i] = wheels[i].data.given_torque;
    wheel_pid_p[i] = wheel_pid[i]->data.pout;
    wheel_pid_i[i] = wheel_pid[i]->data.iout;
    wheel_pid_d[i] = wheel_pid[i]->data.dout;
  }

  // PID 计算后、CAN 发送前预测功率。并按裁判系统上限缩放四轮转矩，限制功率。
  chassis_predicted_power_w =
    predict_chassis_power(torque, feedback_rad_s, kPowerK1, kPowerK2, kPowerK3);

  // 模型结果异常时交由失能模式发送零指令。
  if (!std::isfinite(chassis_predicted_power_w)) return false;

  const float power_limit_w = 80.0f;
  //static_cast<float>(pm02.robot_status.chassis_power_limit) - 5.0f;  //读取功率上限
  chassis_power_limit_w = power_limit_w;

  float kPowerK = 1.0f;

  // 可用功率额度不大于零时，发送零电流指令。
  if (power_limit_w <= 0.0f) {
    kPowerK = 0.0f;
  }
  else if (chassis_predicted_power_w > power_limit_w) {
    // P(K) - P_limit = a*K^2 + b*K + c，K 是等比例缩小四轮期望电流系数。
    float a = 0.0f;
    float b = 0.0f;
    float c = kPowerK3 - power_limit_w;
    for (unsigned i = 0; i < 4U; ++i) {
      a += kPowerK1 * torque[i] * torque[i];
      b += torque[i] * feedback_rad_s[i];
      c += kPowerK2 * feedback_rad_s[i] * feedback_rad_s[i];
    }

    // 二次项为零或无实数解时，按零电流处理，避免除零和无效输出。
    const float discriminant = b * b - 4.0f * a * c;
    if (a <= 0.0f || discriminant < 0.0f) {
      kPowerK = 0.0f;
    }
    else {
      kPowerK = (-b + std::sqrt(discriminant)) / (2.0f * a);
    }

    // 缩放系数只允许减小原电流，不能反向或放大。
    if (kPowerK < 0.0f) kPowerK = 0.0f;
    if (kPowerK > 1.0f) kPowerK = 1.0f;
  }

  chassis_power_scale = kPowerK;
  for (auto & wheel : wheels) {
    wheel.data.given_torque *= kPowerK;  //I_new = K * I_original
  }

  // 发送前复查，防止计算期间遥控器失联或电机反馈超时。
  taskENTER_CRITICAL();
  RemoteTaskData latest = {};
  const uint32_t send_ms = osKernelSysTick();
  const bool latest_online = remote_task_get_data(&latest) && latest.is_alive;
  const bool latest_automatic = latest.sw_l == REMOTE_SWITCH_DOWN;
  const bool right_enabled =
    latest.sw_r == REMOTE_SWITCH_MID ||
    (control_source == ControlSource::AUTOMATIC && latest.sw_r == REMOTE_SWITCH_UP);
  bool command_allowed = latest_online && valid_remote_input(latest) && right_enabled &&
                         latest_automatic == (control_source == ControlSource::AUTOMATIC);
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
  chassis_predicted_power_w = 0.0f;
  chassis_power_limit_w = 0.0f;
  chassis_power_scale = 1.0f;
  clear_motion_commands();

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

// 输入：系统时间和周期间隔；输出：本周期四轮控制帧与调试快照，无阻塞。
static void run_control_cycle(uint32_t now_ms, uint32_t elapsed_ms)
{
  RemoteTaskData remote_data = {};
  taskENTER_CRITICAL();
  const bool remote_ready = remote_task_get_data(&remote_data);
  taskEXIT_CRITICAL();

  //检查遥控器输入合法性
  const bool input_valid = valid_remote_input(remote_data);
  const bool remote_online = remote_ready && remote_data.is_alive;
  const bool right_switch_mid = remote_data.sw_r == REMOTE_SWITCH_MID;  //检查遥控器是否在中档
  chassis_remote_online = remote_online;
  chassis_right_switch_mid = remote_online && right_switch_mid;

  bool motors_ready = true;
  taskENTER_CRITICAL();
  for (auto & wheel : wheels) {
    if (!wheel.motor->is_alive(now_ms) || !std::isfinite(wheel.motor->speed)) motors_ready = false;
  }
  taskEXIT_CRITICAL();
  const ControlSource previous_source = control_source;
  const auto previous_state = mission_controller.status().state;
  const bool automatic_selected = remote_online && remote_data.sw_l == REMOTE_SWITCH_DOWN;
  control_source = automatic_selected ? ControlSource::AUTOMATIC : ControlSource::MANUAL;
  mission::Input input{};
  input.selected = automatic_selected;
  input.remote_online = remote_online;
  input.disable_requested = remote_data.sw_r == REMOTE_SWITCH_DOWN;
  input.input_valid = input_valid;
  input.motors_ready = motors_ready;
  input.now_ms = now_ms;
  input.gap_ms = elapsed_ms > 0U ? elapsed_ms : 1U;
  const auto command = mission_controller.update(input);
  if (previous_source != control_source || previous_state != mission_controller.status().state)
    clear_motion_commands();

  // 右下始终失能；手动仍仅右中允许运动，自动允许右中/右上。
  const bool right_enabled =
    right_switch_mid || (automatic_selected && remote_data.sw_r == REMOTE_SWITCH_UP);
  const bool motion_allowed = remote_online && right_enabled && input_valid && elapsed_ms <= 5U &&
                              mission_controller.status().fault == mission::Fault::None &&
                              (!automatic_selected || command.enabled);
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
    if (automatic_selected) {
      // 轮轴 rad/s 先按同一轮半径换为 m/s，保留麦轮安装方向、轮速限幅和功率保护。
      chassis_vx_set = command.forward_wheel_speed_rad_s * kWheelRadiusM;
      chassis_vy_set = chassis_wz_set = 0.0f;
    }
    else {
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
                         : static_cast<float>(sp::sgn(remote_data.ch_lu)) * kSpinWz;
    }

    // 麦轮解算将车体 m/s、rad/s 转为四轮目标角速度 rad/s。
    chassis.calc(chassis_vx_set, chassis_vy_set, chassis_wz_set);
    float target_rad_s[4] = {
      chassis.speed_lf, chassis.speed_lr, chassis.speed_rf, chassis.speed_rr};
    float peak_rad_s = 0.0f;
    for (float speed_rad_s : target_rad_s) {
      if (std::fabs(speed_rad_s) > peak_rad_s) peak_rad_s = std::fabs(speed_rad_s);
    }
    const float scale = peak_rad_s > kMaxWheelSpeedRadS ? kMaxWheelSpeedRadS / peak_rad_s : 1.0f;
    for (unsigned i = 0; i < 4U; ++i) {
      target_rad_s[i] *= scale;
    }

    if (!run_wheel_speed_control(target_rad_s)) {
      if (automatic_selected) {
        RemoteTaskData latest = {};
        taskENTER_CRITICAL();
        const bool latest_online = remote_task_get_data(&latest) && latest.is_alive;
        taskEXIT_CRITICAL();
        if (!latest_online)
          mission_controller.abort(mission::Fault::RemoteLost);
        else if (!chassis_motors_online)
          mission_controller.abort(mission::Fault::FeedbackLost);
        else if (!valid_remote_input(latest))
          mission_controller.abort(mission::Fault::InvalidInput);
        else if (latest.sw_l == REMOTE_SWITCH_DOWN && latest.sw_r != REMOTE_SWITCH_DOWN)
          mission_controller.abort(mission::Fault::ControlFault);
      }
      run_disable_mode();
    }
  }
  mission_status = mission_controller.status();
}

extern "C" void control_task(void const * argument)
{
  (void)argument;
  can1.config();
  can1.start();
  run_disable_mode();
  referee_start_receive();

  TickType_t last_wake = xTaskGetTickCount();
  uint32_t last_cycle_ms = osKernelSysTick();

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    const uint32_t elapsed_ms = now_ms - last_cycle_ms;
    last_cycle_ms = now_ms;
    run_control_cycle(now_ms, elapsed_ms);

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

    //分流功率计反馈帧
    if (header.StdId == 0x301U) {
      // 功率计反馈为大端、单位 0.01 W。
      const uint16_t power_x100 = (static_cast<uint16_t>(data[4]) << 8) | data[5];
      chassis_real_power_w = power_x100 / 100.0f;
      continue;
    }

    //根据 ID 分流 CAN 数据到对应电机
    for (auto & wheel : wheels) {
      if (header.StdId == wheel.motor->rx_id) {
        wheel.motor->read(data, stamp_ms);
      }
    }
  }
}
