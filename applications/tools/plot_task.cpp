#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

//extern volatile float chassis_predicted_power_w;
extern volatile float chassis_real_power_w;
extern volatile float chassis_power_limit_w;

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  while (true) {
    float predicted_power_w, real_power_w, power_limit_w;

    // 取得同一时刻的功率快照，三条曲线单位均为 W。
    taskENTER_CRITICAL();
    //predicted_power_w = chassis_predicted_power_w;
    real_power_w = chassis_real_power_w;
    power_limit_w = chassis_power_limit_w;
    taskEXIT_CRITICAL();

    plotter.plot(power_limit_w, real_power_w);

    osDelay(10);
  }
}
