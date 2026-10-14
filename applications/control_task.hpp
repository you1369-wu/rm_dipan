#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#include "motor/rm_motor/rm_motor.hpp"

struct ChassisMotorData
{
  float target_speed_set;  // 轮轴目标角速度，rad/s
  float given_torque;      // 轮轴给定力矩，N·m
};

struct ChassisWheel
{
  sp::RM_Motor * motor;
  ChassisMotorData data;
};

// 顺序：左前、左后、右前、右后
extern ChassisWheel wheels[4];

#endif