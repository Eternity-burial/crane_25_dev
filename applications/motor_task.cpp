#include "cmsis_os.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"

sp::RM_Motor motoryl(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motoryr(2, sp::RM_Motors::M3508, sp::M3508_P19);

//PID
//                            dt    kp    ki     kd    out iout alpha
sp::PID motoryl_position_pid(T, 30, 4, 0.0007, 5, 3, 0.001);
sp::PID motoryr_position_pid(T, 30, 4, 0.0007, 5, 3, 0.001);
//                        dt    kp    ki     kd    out iout alpha
sp::PID motoryl_speed_pid(T, 0.20, 0.2, 0.0007, 3.6, 0.4, 0.001);
sp::PID motoryr_speed_pid(T, 0.20, 0.2, 0.0007, 3.6, 0.4, 0.001);

move_t move_data;
void angley_calc();
void set_speedy_calc();
void motory_calc();
extern "C" void motor_task()
{
  osDelay(1000);
  while (true) {
    angley_calc();
    set_speedy_calc();
    motory_calc();
    osDelay(1);
  }
}

void angley_calc()
{
  // TODO
  move_data.set_motor_angle[0] = 0.5f / GEAR_RADIUS;
  move_data.set_motor_angle[1] = -0.5f / GEAR_RADIUS;

  // move_data.set_motor_angle[0] = 0.f / GEAR_RADIUS;
  // move_data.set_motor_angle[1] = 0.f / GEAR_RADIUS;
}

void set_speedy_calc()
{
  motoryl_position_pid.calc(move_data.set_motor_angle[0], motoryl.angle);
  motoryr_position_pid.calc(move_data.set_motor_angle[1], motoryr.angle);

  move_data.set_motor_speed[0] = motoryl_position_pid.out;
  move_data.set_motor_speed[1] = motoryr_position_pid.out;
}

void motory_calc()
{
  motoryl_speed_pid.calc(move_data.set_motor_speed[0], motoryl.speed);
  motoryr_speed_pid.calc(move_data.set_motor_speed[1], motoryr.speed);

  move_data.set_motor_torque[0] = motoryl_speed_pid.out;
  move_data.set_motor_torque[1] = motoryr_speed_pid.out;
}