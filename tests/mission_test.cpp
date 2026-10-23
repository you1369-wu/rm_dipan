#include <cmath>
#include <cstdlib>
#include <iostream>
#include "mission/mission.hpp"
#include "mission/mission_config.hpp"

namespace
{
void check(bool result, const char * message)
{
  if (!result) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
void arm(mission::Controller & controller, mission::Input & input)
{
  input.disable_requested = true;
  check(!controller.update(input).enabled && controller.status().armed,
        "DOWN disables output and arms the next run");
  input.disable_requested = false;
}
}

int main()
{
  using namespace mission;
  check(mission_config::kForwardWheelSpeedRadS == 12.0f &&
          mission_config::kDriveDurationMs == 4000U,
        "automatic defaults request wheel-shaft 12rad/s for 4 seconds");
  Controller controller;
  Input input{};
  input.selected = input.remote_online = input.input_valid = input.motors_ready = true;
  check(!controller.update(input).enabled && controller.status().fault == Fault::NotArmed,
        "startup cannot drive without online DOWN acknowledgment");
  arm(controller, input);
  input.motors_ready = false;
  check(!controller.update(input).enabled && controller.status().state == State::WaitReady,
        "missing feedback waits without starting the timer");
  input.now_ms = 5000U;
  input.motors_ready = true;
  check(controller.update(input).forward_wheel_speed_rad_s == mission_config::kForwardWheelSpeedRadS &&
          controller.status().started_ms == 5000U, "ready feedback starts forward motion");
  input.now_ms += mission_config::kDriveDurationMs - 1U;
  check(controller.update(input).forward_wheel_speed_rad_s > 0.0f &&
          controller.status().state == State::Running,
        "3999 ms remains in forward phase");
  ++input.now_ms;
  auto command = controller.update(input);
  check(command.enabled && command.forward_wheel_speed_rad_s == 0.0f &&
          controller.status().state == State::Finished &&
          controller.status().elapsed_ms == mission_config::kDriveDurationMs,
        "4000 ms commands an active zero-speed stop");
  input.now_ms += 10000U;
  check(controller.update(input).forward_wheel_speed_rad_s == 0.0f &&
          controller.status().state == State::Finished,
        "held automatic switch never repeats the run");
  input.selected = false;
  check(!controller.update(input).enabled && controller.status().state == State::Disabled,
        "manual selection releases automatic control");
  input.selected = true;
  check(controller.update(input).forward_wheel_speed_rad_s > 0.0f &&
          controller.status().elapsed_ms == 0U,
        "normal reentry starts a fresh timed run");
  input.remote_online = false;
  check(!controller.update(input).enabled && controller.status().fault == Fault::RemoteLost,
        "remote loss aborts a running mission");
  input.remote_online = true;
  input.selected = false;
  check(!controller.update(input).enabled && controller.status().fault == Fault::RemoteLost,
        "manual selection cannot bypass an automatic fault");
  input.selected = true;
  arm(controller, input);
  controller.update(input);
  input.motors_ready = false;
  check(!controller.update(input).enabled && controller.status().fault == Fault::FeedbackLost,
        "feedback loss after start is latched");
  input.motors_ready = true;
  arm(controller, input);
  controller.update(input);
  input.gap_ms = 6U;
  check(!controller.update(input).enabled && controller.status().fault == Fault::Timing,
        "scheduler stall cannot restart or continue autonomous motion");
  controller.abort(Fault::ControlFault);
  check(controller.status().fault == Fault::Timing, "first fault remains latched");
  input.input_valid = false;
  arm(controller, input);
  input.gap_ms = 1U;
  check(!controller.update(input).enabled && controller.status().fault == Fault::InvalidInput,
        "invalid controls are rejected");
  input.input_valid = true;
  input.now_ms = UINT32_MAX - 999U;
  arm(controller, input);
  controller.update(input);
  input.now_ms = mission_config::kDriveDurationMs - 1001U;
  check(controller.update(input).forward_wheel_speed_rad_s > 0.0f,
        "timer wrap preserves the pre-deadline boundary");
  ++input.now_ms;
  check(controller.update(input).forward_wheel_speed_rad_s == 0.0f &&
          controller.status().state == State::Finished,
        "timer wrap preserves the 4000 ms stop boundary");
  input.motors_ready = input.input_valid = false;
  input.gap_ms = 100U;
  arm(controller, input);
  check(controller.status().fault == Fault::None && controller.status().elapsed_ms == 0U,
        "emergency DOWN takes priority over input, feedback and timing errors");
  std::cout << "PASS: 4s deadline, no-repeat stop, reentry, arming, faults and timer wrap\n";
}
