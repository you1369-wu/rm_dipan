#pragma once

#include <cstdint>

namespace mission_config
{
// 自动模式的车体前进速度请求；最终仍受原轮速限幅和功率保护约束。
constexpr float kForwardSpeedMps = 2.0f;
constexpr uint32_t kDriveDurationMs = 4000U;
constexpr uint32_t kMaxControlGapMs = 5U;
}  // namespace mission_config
