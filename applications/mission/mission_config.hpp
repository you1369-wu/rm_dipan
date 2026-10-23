#pragma once

#include <cstdint>

namespace mission_config
{
// 自动纯前进时的轮轴目标角速度幅值，rad/s；安装方向由麦轮解算处理。
constexpr float kForwardWheelSpeedRadS = 12.0f;
constexpr uint32_t kDriveDurationMs = 4000U;
constexpr uint32_t kMaxControlGapMs = 5U;
}  // namespace mission_config
