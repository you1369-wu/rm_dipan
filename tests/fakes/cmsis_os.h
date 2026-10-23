#pragma once
#include <cstdint>

using TickType_t = uint32_t;
constexpr uint32_t configTICK_RATE_HZ = 1000U;
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
extern "C" {
uint32_t osKernelSysTick();
int osDelay(uint32_t);
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t *, TickType_t);
}
