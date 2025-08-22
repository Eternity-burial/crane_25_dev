#include <cmath>

#include "cmsis_os.h"

// ===== 3508 & 工具 =====
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_para.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/pid/pid.hpp"
#include "velocity_smoother.hpp"

// ===== 飞特舵机 =====
#include "servo_link.hpp"
#include "sts3215.h"

// ---------- 你的原有对象 ----------
sp::RM_Motor motoryl(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motoryr(2, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorx(3, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorzf(4, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motorzr(5, sp::RM_Motors::M3508, sp::M3508_P19);

// PID
//                          dt  kp  ki   kd       out          iout      alpha
sp::PID motoryl_position_pid(T, 25, 0, 0.0003, MAX_Y_SPEED, MAX_Y_SPEED, 0.001);
sp::PID motoryr_position_pid(T, 25, 0, 0.0003, MAX_Y_SPEED, MAX_Y_SPEED, 0.001);
//                        dt  kp    ki     kd    out  iout alpha
sp::PID motoryl_speed_pid(T, 0.25, 0.07, 0.0001, 3.6, 0., 0.001);
sp::PID motoryr_speed_pid(T, 0.25, 0.07, 0.0001, 3.6, 0., 0.001);

//                          dt  kp  ki   kd       out          iout      alpha
sp::PID motorx_position_pid(T, 10, 0, 0.0001, MAX_X_SPEED, MAX_X_SPEED, 0.001);
//                        dt  kp    ki     kd    out  iout alpha
sp::PID motorx_speed_pid(T, 0.05, 0., 0.0001, 3.6, 0.4, 0.001);

//                          dt  kp  ki   kd       out          iout      alpha
sp::PID motorzf_position_pid(T, 20, 0, 0.0001, MAX_ZF_SPEED, MAX_ZF_SPEED, 0.001);
sp::PID motorzr_position_pid(T, 25, 0, 0.0001, MAX_ZR_SPEED, MAX_ZR_SPEED, 0.001);
//                        dt  kp    ki     kd    out  iout alpha
sp::PID motorzf_speed_pid(T, 0.07, 0.01, 0.0001, 3.6, 0.4, 0.001);
sp::PID motorzr_speed_pid(T, 0.20, 0.03, 0.0001, 3.6, 0.4, 0.001);

// 速度平滑（如需再开启 plan_speed）
sp::LowPassFilter motoryl_filter(FILUTTER_ALPHA);
sp::LowPassFilter motoryr_filter(FILUTTER_ALPHA);
sp::LowPassFilter motorx_filter(FILUTTER_ALPHA);
sp::LowPassFilter motorzf_filter(FILUTTER_ALPHA);
sp::LowPassFilter motorzr_filter(FILUTTER_ALPHA);

// 全局数据（供 can_task 解析后写入）
move_t move_data;
coordinate_t coordinate_data;

// 舵机“邮箱”
ServoTarget g_servo_target[2];  // [0]=Z1(ID1), [1]=Z2(ID2)

// ---------- 3508 控制函数 ----------
static void angle_calc(float coord_y, float coord_x, float coord_zf, float coord_zr)
{
  move_data.set_motor_angle[0] = -coord_y / Y_RADIUS;
  move_data.set_motor_angle[1] = coord_y / Y_RADIUS;
  move_data.set_motor_angle[2] = coord_x / X_RADIUS;
  move_data.set_motor_angle[3] = -coord_zf / ZF_RADIUS;
  move_data.set_motor_angle[4] = coord_zr / ZR_RADIUS;
}

static void speed_calc()
{
  // Y
  motoryl_position_pid.calc(move_data.set_motor_angle[0], motoryl.angle);
  motoryr_position_pid.calc(move_data.set_motor_angle[1], motoryr.angle);
  move_data.set_motor_speed[0] = motoryl_position_pid.out;
  move_data.set_motor_speed[1] = motoryr_position_pid.out;

  // X
  motorx_position_pid.calc(move_data.set_motor_angle[2], motorx.angle);
  move_data.set_motor_speed[2] = motorx_position_pid.out;

  // Z
  motorzf_position_pid.calc(move_data.set_motor_angle[3], motorzf.angle);
  motorzr_position_pid.calc(move_data.set_motor_angle[4], motorzr.angle);
  move_data.set_motor_speed[3] = motorzf_position_pid.out;
  move_data.set_motor_speed[4] = motorzr_position_pid.out;
}

static void plan_speed()
{
  motoryl_filter.update(move_data.set_motor_speed[0]);
  move_data.set_motor_speed[0] = motoryl_filter.out;
  motoryr_filter.update(move_data.set_motor_speed[1]);
  move_data.set_motor_speed[1] = motoryr_filter.out;

  motorx_filter.update(move_data.set_motor_speed[2]);
  move_data.set_motor_speed[2] = motorx_filter.out;

  motorzf_filter.update(move_data.set_motor_speed[3]);
  move_data.set_motor_speed[3] = motorzf_filter.out;
  motorzr_filter.update(move_data.set_motor_speed[4]);
  move_data.set_motor_speed[4] = motorzr_filter.out;
}

static void torque_calc()
{
  // Y
  motoryl_speed_pid.calc(move_data.set_motor_speed[0], motoryl.speed);
  motoryr_speed_pid.calc(move_data.set_motor_speed[1], motoryr.speed);
  move_data.set_motor_torque[0] = motoryl_speed_pid.out;
  move_data.set_motor_torque[1] = motoryr_speed_pid.out;

  // X
  motorx_speed_pid.calc(move_data.set_motor_speed[2], motorx.speed);
  move_data.set_motor_torque[2] = motorx_speed_pid.out;

  // Z
  motorzf_speed_pid.calc(move_data.set_motor_speed[3], motorzf.speed);
  motorzr_speed_pid.calc(move_data.set_motor_speed[4], motorzr.speed);
  move_data.set_motor_torque[3] = motorzf_speed_pid.out;
  move_data.set_motor_torque[4] = motorzr_speed_pid.out;
}

static inline void move_calc(float coord_y, float coord_x, float coord_zf, float coord_zr)
{
  angle_calc(coord_y, coord_x, coord_zf, coord_zr);
  speed_calc();
  plan_speed();  // 需要速度斜坡时再开启
  torque_calc();
}

// ========== 主任务 ==========
extern "C" void motor_task(void const * argument)
{
  // 上电稳定
  vTaskDelay(pdMS_TO_TICKS(300));

  // 舵机：使能扭矩（一次性）
  STS_Torque(1, 1);
  STS_Torque(2, 1);
  vTaskDelay(pdMS_TO_TICKS(20));

  // 3508 初始
  coordinate_data.y = 0.0f;
  coordinate_data.x = 0.0f;
  coordinate_data.zf = 0.0f;
  coordinate_data.zr = 0.0f;

  for (;;) {
    // ===== 3508 控制链（1kHz）=====
    move_calc(coordinate_data.y, coordinate_data.x, coordinate_data.zf, coordinate_data.zr);

    // ===== 舵机执行（非阻塞）=====
    for (int i = 0; i < 2; ++i) {
      if (g_servo_target[i].pending) {
        const uint8_t id = (i == 0) ? 1 : 2;  // Z1->ID1, Z2->ID2
        STS_SetAngle(
          id, g_servo_target[i].angle_deg,
          g_servo_target[i].speed_dps,  // 0 => 使用默认
          g_servo_target[i].acc_dps2);  // 0 => 使用默认
        g_servo_target[i].pending = false;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));  // 1ms tick
  }
}
