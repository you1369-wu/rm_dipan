#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"

extern float wheel_pid_p[4];
extern float wheel_pid_i[4];
extern float wheel_pid_d[4];

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  constexpr unsigned kPlotWheel = 1U;  // 左后轮 LR
  while (true) {
    float set, feedback, p_out, i_out, d_out;

    // 复制目标轮速、反馈轮速及 PID 三项输出。
    taskENTER_CRITICAL();
    set = wheels[kPlotWheel].data.target_speed_set;
    feedback = wheels[kPlotWheel].motor->speed;
    p_out = wheel_pid_p[kPlotWheel];
    i_out = wheel_pid_i[kPlotWheel];
    d_out = wheel_pid_d[kPlotWheel];
    taskEXIT_CRITICAL();

    // 通道顺序：目标轮速、实际轮速、P、I、D。
    plotter.plot(set, feedback, p_out, i_out, d_out);

    osDelay(10);
  }
}
