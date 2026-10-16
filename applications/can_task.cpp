// CAN和电机对象定义
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_lf(4, sp::RM_Motors::M3508, 13.7f);  // 左前
sp::RM_Motor motor_lr(1, sp::RM_Motors::M3508, 13.7f);  // 左后
sp::RM_Motor motor_rf(3, sp::RM_Motors::M3508, 13.7f);  // 右前
sp::RM_Motor motor_rr(2, sp::RM_Motors::M3508, 13.7f);  // 右后
extern "C" void can_task(void const * argument)
{
  (void)argument;

  while (true) {
    // 当前CAN收发由control_task统一负责, 保留空闲任务入口便于CubeMX配置回退
    osDelay(1000);
  }
}
