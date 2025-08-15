#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

sp::RM_Motor motor1(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor2(2, sp::RM_Motors::M3508, sp::M3508_P19);

//PID
//                    dt    kp    ki     kd    out iout alpha
sp::PID motor1_pid(0.002, 0.20, 0.2, 0.0007, 3.6, 0.4, 0.001);
sp::PID motor2_pid(0.002, 0.20, 0.2, 0.0007, 3.6, 0.4, 0.001);

move_t move_data;
void speed_calc();
void set_speed_calc();
void motor_calc();
extern "C" void motor_task()
{
  osDelay(1000);
  while (true) {
    speed_calc();
    set_speed_calc();
    motor_calc();
  }
}

void speed_calc()
{
  // TODO
}

void set_speed_calc()
{
  move_data.set_motor_speed[0] = 1.0f;
  move_data.set_motor_speed[1] = 1.0f;
}

void motor_calc()
{
  motor1_pid.calc(move_data.set_motor_speed[0], motor1.speed);
  motor2_pid.calc(move_data.set_motor_speed[1], motor2.speed);

  move_data.set_motor_torque[0] = motor1_pid.out;
  move_data.set_motor_torque[1] = motor2_pid.out;
}