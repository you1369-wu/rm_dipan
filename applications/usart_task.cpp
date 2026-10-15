#include "usart_task.h"

#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usart.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

extern sp::RM_Motor motor_lr;
extern volatile bool lr_step_recording;
extern volatile uint32_t lr_step_start_ms;
extern volatile int16_t lr_step_current_raw;

extern "C" void usart_task(void const * argument)
{
  (void)argument;

  constexpr TickType_t kSampleTicks = pdMS_TO_TICKS(10);
  static uint8_t header[] = "time,current_raw,rpm\r\n";
  bool was_recording = false;
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    taskENTER_CRITICAL();
    const bool recording = lr_step_recording;
    const uint32_t start_ms = lr_step_start_ms;
    const int16_t current_raw = lr_step_current_raw;
    const float speed_rad_s = motor_lr.speed;
    const uint32_t sample_ms = osKernelSysTick();
    taskEXIT_CRITICAL();

    if (recording && std::isfinite(speed_rad_s)) {
      if (!was_recording) {
        HAL_UART_Transmit(&huart1, header, sizeof(header) - 1U, 10);
      }

      // The M3508 feedback speed is converted to wheel-shaft rad/s by RM_Motor.
      const float rpm = speed_rad_s * (30.0f / sp::SP_PI);
      const long rpm_tenths = static_cast<long>(rpm * 10.0f + (rpm >= 0.0f ? 0.5f : -0.5f));
      const long rpm_abs = rpm_tenths < 0 ? -rpm_tenths : rpm_tenths;
      char line[48];
      const int length = std::snprintf(
        line, sizeof(line), "%lu.%03lu,%d,%s%ld.%01ld\r\n",
        static_cast<unsigned long>((sample_ms - start_ms) / 1000U),
        static_cast<unsigned long>((sample_ms - start_ms) % 1000U),
        static_cast<int>(current_raw), rpm_tenths < 0 ? "-" : "", rpm_abs / 10, rpm_abs % 10);
      if (length > 0 && static_cast<std::size_t>(length) < sizeof(line)) {
        HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t *>(line), length, 10);
      }
    }

    was_recording = recording;
    vTaskDelayUntil(&last_wake, kSampleTicks);
  }
}
