#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include "mission/mission_config.hpp"

// Isolate referee transport, not the actual motor, PID, kinematics or power limiter.
#define REFEREE_HPP
void referee_start_receive();
#include "../applications/control_task.cpp"

namespace
{
uint32_t test_ms = 1000U;
RemoteTaskData remote_input{};
bool remote_available = true;
bool override_send_snapshot = false;
RemoteTaskData send_snapshot{};
unsigned remote_reads = 0U, sends = 0U;
uint8_t last_frame[8]{};
float measured_speed[4]{};

void check(bool result, const char * message)
{
  if (!result) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
bool near(float a, float b) { return std::fabs(a - b) < 1e-5f; }
void feedback()
{
  uint8_t frame[8] = {0x0F, 0xFF, 0, 0, 0, 0, 25, 0};
  for (unsigned i = 0; i < 4; ++i) {
    wheels[i].motor->read(frame, test_ms);
    wheels[i].motor->speed = measured_speed[i];
  }
}
void tick(uint32_t now, uint32_t gap = 1U, bool fresh = true)
{
  test_ms = now;
  remote_reads = 0U;
  if (fresh) feedback();
  const auto before = sends;
  run_control_cycle(now, gap);
  check(sends == before + 1U, "every control period sends exactly one wheel command frame");
}
void prepare()
{
  test_ms = 1000U;
  remote_input = {};
  remote_input.is_alive = true;
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  remote_input.sw_l = REMOTE_SWITCH_MID;
  remote_available = true;
  override_send_snapshot = false;
  control_source = ControlSource::MANUAL;
  mission_controller = mission::Controller{};
  mission_status = {};
  std::fill(measured_speed, measured_speed + 4, 0.0f);
  feedback();
  run_disable_mode();
  sends = 0U;
}
void zero_commands(const char * message)
{
  for (const auto byte : last_frame) check(byte == 0U, message);
  for (const auto & wheel : wheels)
    check(wheel.data.target_speed_set == 0.0f && wheel.data.given_torque == 0.0f, message);
}
uint32_t start_auto()
{
  tick(test_ms);
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_MID;
  const uint32_t start = test_ms + 1U;
  tick(start);
  check(mission_status.state == mission::State::Running, "armed automatic mode starts");
  return start;
}
}

CAN_HandleTypeDef hcan1;
extern "C" uint32_t osKernelSysTick() { return test_ms; }
extern "C" int osDelay(uint32_t) { return 0; }
extern "C" TickType_t xTaskGetTickCount() { return test_ms; }
extern "C" void vTaskDelayUntil(TickType_t *, TickType_t) {}
extern "C" uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t) { return 0U; }
extern "C" int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *)
{ return HAL_ERROR; }
extern "C" bool remote_task_get_data(RemoteTaskData * data)
{
  *data = override_send_snapshot && remote_reads++ > 0U ? send_snapshot : remote_input;
  return remote_available;
}
void referee_start_receive() {}

namespace sp
{
CAN::CAN(CAN_HandleTypeDef * hcan) : hcan_(hcan) { std::fill(tx_data, tx_data + 8, 0U); }
void CAN::config() {}
void CAN::start() {}
void CAN::send(uint32_t tx_id)
{
  check(tx_id == motor_lf.tx_id, "only the existing shared wheel command group is sent");
  std::copy(tx_data, tx_data + 8, last_frame);
  ++sends;
}
}

int main()
{
  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  remote_input.ch_lv = remote_input.ch_lh = 0.5f;
  remote_input.ch_lu = 1.0f;
  tick(test_ms);
  check(control_source == ControlSource::MANUAL && near(chassis_vx_set, 0.0005f) &&
          near(chassis_vy_set, -0.0005f) && near(chassis_wz_set, kSpinWz),
        "manual joystick, acceleration ramp and small-spin speed remain unchanged");
  for (const auto & wheel : wheels)
    check(std::fabs(wheel.data.target_speed_set) <= kMaxWheelSpeedRadS + 0.00001f,
          "manual wheel targets retain the current wheel limit");
  remote_input.sw_r = REMOTE_SWITCH_UP;
  tick(++test_ms);
  zero_commands("manual right UP remains disabled");

  prepare();
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  remote_input.sw_r = REMOTE_SWITCH_UP;
  tick(test_ms);
  check(mission_status.fault == mission::Fault::NotArmed, "startup auto requires right DOWN");
  zero_commands("unarmed automatic startup is zero torque");

  prepare();
  const uint32_t start = start_auto();
  check(control_source == ControlSource::AUTOMATIC &&
          near(chassis_vx_set, mission_config::kForwardWheelSpeedRadS * kWheelRadiusM) &&
          chassis_vy_set == 0.0f && chassis_wz_set == 0.0f, "auto requests forward-only body velocity");
  const float forward_wheel_speed =
    std::min(mission_config::kForwardWheelSpeedRadS, kMaxWheelSpeedRadS);
  check(near(wheels[0].data.target_speed_set, forward_wheel_speed) &&
          near(wheels[1].data.target_speed_set, forward_wheel_speed) &&
          near(wheels[2].data.target_speed_set, -forward_wheel_speed) &&
          near(wheels[3].data.target_speed_set, -forward_wheel_speed),
        "real mecanum mapping retains current protection and installation directions");
  check(near(forward_wheel_speed, 12.0f) && near(chassis_vx_set, 0.924f),
        "automatic 12rad/s is not confused with 12m/s or the global wheel speed cap");
  check(near(chassis_power_limit_w, 80.0f), "automatic mode retains the 80W power limit");
  remote_input.sw_r = REMOTE_SWITCH_UP;
  remote_input.ch_lv = 0.8f;
  remote_input.ch_lh = -0.6f;
  remote_input.ch_lu = 1.0f;
  tick(start + mission_config::kDriveDurationMs - 1U);
  check(mission_status.state == mission::State::Running &&
          near(chassis_vx_set, mission_config::kForwardWheelSpeedRadS * kWheelRadiusM) &&
          chassis_vy_set == 0.0f && chassis_wz_set == 0.0f, "auto ignores valid manual motion inputs");
  measured_speed[0] = measured_speed[1] = 1.0f;
  measured_speed[2] = measured_speed[3] = -1.0f;
  tick(start + mission_config::kDriveDurationMs);
  check(mission_status.state == mission::State::Finished && chassis_vx_set == 0.0f,
        "exact 4000ms boundary sets zero body speed");
  for (unsigned i = 0; i < 4; ++i)
    check(wheels[i].data.target_speed_set == 0.0f &&
            wheels[i].data.given_torque * measured_speed[i] < 0.0f,
          "finished mission clears old integral and commands braking, not forward torque");
  tick(start + 9000U);
  check(mission_status.state == mission::State::Finished && chassis_vx_set == 0.0f,
        "held auto switch keeps the stopped state without repeating");
  remote_input.sw_r = REMOTE_SWITCH_DOWN;
  tick(start + 9001U);
  zero_commands("right DOWN always overrides automatic braking or driving");
  check(mission_status.elapsed_ms == 0U, "DOWN clears the automatic timer");

  prepare();
  start_auto();
  remote_input.sw_l = REMOTE_SWITCH_MID;
  remote_input.ch_lv = 0.5f;
  tick(++test_ms);
  check(control_source == ControlSource::MANUAL && near(chassis_vx_set, 0.0005f),
        "returning to manual does not retain the automatic forward request");
  remote_input.sw_l = REMOTE_SWITCH_DOWN;
  tick(++test_ms);
  check(mission_status.state == mission::State::Running && mission_status.elapsed_ms == 0U,
        "normal auto reentry starts a new run");

  prepare();
  start_auto();
  tick(test_ms + 1U, 6U);
  check(mission_status.fault == mission::Fault::Timing, "automatic scheduling stalls latch a fault");
  zero_commands("scheduling fault sends a zero frame");
  remote_input.sw_l = REMOTE_SWITCH_MID;
  remote_input.ch_lv = 1.0f;
  tick(++test_ms);
  zero_commands("manual selection cannot bypass an automatic fault");

  prepare();
  start_auto();
  remote_input.is_alive = false;
  tick(++test_ms);
  check(mission_status.fault == mission::Fault::RemoteLost, "automatic remote loss is latched");
  zero_commands("remote loss disables all four motors");

  prepare();
  start_auto();
  tick(test_ms + 100U, 1U, false);
  check(mission_status.fault == mission::Fault::FeedbackLost, "automatic motor timeout is latched");
  zero_commands("motor timeout disables all four motors");

  prepare();
  start_auto();
  remote_input.ch_lv = NAN;
  tick(++test_ms);
  check(mission_status.fault == mission::Fault::InvalidInput, "nonfinite input aborts auto");
  zero_commands("nonfinite input is never sent as a motor command");

  prepare();
  start_auto();
  send_snapshot = remote_input;
  send_snapshot.sw_r = REMOTE_SWITCH_DOWN;
  override_send_snapshot = true;
  tick(++test_ms);
  zero_commands("send-time right DOWN overrides an earlier automatic motion snapshot");

  prepare();
  remote_input.sw_r = REMOTE_SWITCH_MID;
  feedback();
  for (auto & wheel : wheels) wheel.motor->speed = 5.0f;
  const float target[4] = {10, 10, 10, 10};
  check(run_wheel_speed_control(target) && chassis_predicted_power_w > 80.0f &&
          chassis_power_scale < 1.0f && chassis_power_scale >= 0.0f,
        "existing power limiter still scales excessive torque before CAN output");
  std::cout << "PASS: manual regression, 4s automatic stop, braking, wheel/power limits, "
               "switch safety and fault handling\n";
}
