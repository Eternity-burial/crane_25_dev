#include "velocity_smoother.hpp"

#include <algorithm>  // For std::min/max
#include <cmath>      // For std::fabs

namespace sp
{

VelocitySmoother::VelocitySmoother(float max_velocity, float acceleration, float dt)
: current_velocity(0.0f),
  max_velocity_(std::fabs(max_velocity)),
  acceleration_(std::fabs(acceleration)),
  dt_(dt)
{
  max_delta_v_ = acceleration_ * dt_;
}

void VelocitySmoother::update(float target_velocity)
{
  // 1. 限制目标速度不能超过最大速度
  target_velocity = std::max(-max_velocity_, std::min(target_velocity, max_velocity_));

  // 2. 计算当前速度与目标速度的误差
  const float error = target_velocity - this->current_velocity;

  // 3. 计算本次update应该变化的速度量 (不超过最大变化量)
  const float delta_v = std::max(-max_delta_v_, std::min(error, max_delta_v_));

  // 4. 更新当前速度
  this->current_velocity += delta_v;
}

void VelocitySmoother::set_max_velocity(float max_velocity)
{
  max_velocity_ = std::fabs(max_velocity);
}

void VelocitySmoother::set_acceleration(float acceleration)
{
  acceleration_ = std::fabs(acceleration);
  // 关键: 更新依赖于加速度的变量
  max_delta_v_ = acceleration_ * dt_;
}

void VelocitySmoother::reset(float initial_velocity) { this->current_velocity = initial_velocity; }

}  // namespace sp