#include "mission.hpp"

#include "mission_config.hpp"

namespace mission
{
void Controller::abort(Fault fault)
{
  if (fault == Fault::None) return;
  if (status_.fault == Fault::None) status_.fault = fault;
  status_.state = State::Fault;
  status_.armed = false;
}

Command Controller::update(const Input & input)
{
  if (input.remote_online && input.disable_requested) {
    status_ = {};
    status_.armed = true;
    return {};
  }
  if (status_.fault != Fault::None) return {};
  if (!input.remote_online) {
    if (status_.state != State::Disabled) abort(Fault::RemoteLost);
    status_.armed = false;
    return {};
  }
  if (!input.selected) {
    const bool armed = status_.armed;
    status_ = {};
    status_.armed = armed;
    return {};
  }
  if (!input.input_valid) {
    abort(Fault::InvalidInput);
    return {};
  }
  if (input.gap_ms == 0U || input.gap_ms > mission_config::kMaxControlGapMs) {
    abort(Fault::Timing);
    return {};
  }
  if (status_.state == State::Disabled) {
    if (!status_.armed) {
      abort(Fault::NotArmed);
      return {};
    }
    status_.state = State::WaitReady;
  }
  if (status_.state == State::WaitReady) {
    if (!input.motors_ready) return {};
    status_.state = State::Running;
    status_.started_ms = input.now_ms;
  }
  if (!input.motors_ready) {
    abort(Fault::FeedbackLost);
    return {};
  }
  if (status_.state == State::Running) {
    const uint32_t elapsed_ms = input.now_ms - status_.started_ms;
    status_.elapsed_ms = elapsed_ms < mission_config::kDriveDurationMs
      ? elapsed_ms : mission_config::kDriveDurationMs;
    if (elapsed_ms >= mission_config::kDriveDurationMs) status_.state = State::Finished;
  }
  Command command{};
  command.enabled = true;
  command.forward_wheel_speed_rad_s = status_.state == State::Running
    ? mission_config::kForwardWheelSpeedRadS : 0.0f;
  return command;
}
}  // namespace mission
