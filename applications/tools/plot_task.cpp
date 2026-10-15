#include "cmsis_os.h"
#include "control_task.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

extern float wheel_pid_p[4];
extern float wheel_pid_i[4];
extern float wheel_pid_d[4];

sp::Plotter plotter(&huart1, false);

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  while (true) {
    float set, feedback, torque;

    // 复制左前轮数据
    taskENTER_CRITICAL();

    set = wheels[1].data.target_speed_set;
    feedback = wheels[1].motor->speed;
    torque = wheels[1].data.given_torque;
    taskEXIT_CRITICAL();

    // 顺序：目标轮速、实际轮速、给定力矩（pid_out）
    plotter.plot(set, feedback, torque);

    osDelay(10);
  }
}