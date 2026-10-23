#pragma once

#include <cstdint>

namespace mission
{
enum class State { Disabled, WaitReady, Running, Finished, Fault };
enum class Fault { None, NotArmed, RemoteLost, InvalidInput, FeedbackLost, Timing, ControlFault };

struct Input
{
  bool selected = false;           // 左下选择自动流程。
  bool remote_online = false;      // 遥控器数据有效且在线。
  bool disable_requested = false;  // 右下优先失能、确认故障并清除计时。
  bool input_valid = false;        // 遥控器输入有限、范围及拨杆枚举有效。
  bool motors_ready = false;       // 四轮在线，速度反馈均为有限值。
  uint32_t now_ms = 0U;
  uint32_t gap_ms = 1U;
};

struct Command
{
  bool enabled = false;  // false 走公共零力矩失能；Finished 则主动控制零速度。
  float vx = 0.0f;       // 车体前进速度目标，m/s；本任务不要求横移或旋转。
};

struct Status
{
  State state = State::Disabled;
  Fault fault = Fault::None;
  bool armed = false;             // 已观察到在线右下，且未因异常撤销启动许可。
  uint32_t started_ms = 0U;       // 四轮反馈就绪后开始运行的时刻。
  uint32_t elapsed_ms = 0U;       // 行驶指令已持续时间，到配置时长后保持。
};

class Controller
{
public:
  // 输入：本周期选择、安全条件和时间；输出：速度请求，不读写电机或 CAN。
  // 仅由 control_task 调用，无阻塞，不创建新 FreeRTOS 任务。
  Command update(const Input & input);

  // 输入：执行轮速控制后产生的故障；输出：锁存首个故障并撤销自动启动许可。
  void abort(Fault fault);

  // 输出：内部只读状态；跨任务发布快照由调用者负责。
  const Status & status() const { return status_; }

private:
  Status status_{};
};
}  // namespace mission
