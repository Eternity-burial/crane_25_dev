#ifndef SP__VELOCITY_SMOOTHER_HPP
#define SP__VELOCITY_SMOOTHER_HPP

namespace sp
{
/**
 * @class VelocitySmoother
 * @brief 一个斜坡速度规划器, 用于平滑目标速度的变化。
 *
 * 该类的所有单位均使用国际单位制(SI):
 * - 速度: rad/s (弧度每秒)
 * - 加速度: rad/s^2 (弧度每平方秒)
 * - 时间: s (秒)
 */
class VelocitySmoother
{
public:
  /**
   * @brief 构造函数
   * @param max_velocity 最大速度 (rad/s)
   * @param acceleration 加速度 (rad/s^2)
   * @param dt 控制周期 (s)
   */
  VelocitySmoother(float max_velocity, float acceleration, float dt);

  // 只读! update()的计算结果, 即当前规划的速度
  float current_velocity;

  /**
   * @brief 更新函数, 传入新的目标速度, 计算当前周期的平滑速度
   * @param target_velocity 目标速度 (rad/s)
   */
  void update(float target_velocity);

  /**
   * @brief 动态设置最大速度
   * @param max_velocity 新的最大速度 (rad/s)
   */
  void set_max_velocity(float max_velocity);

  /**
   * @brief 动态设置加速度
   * @param acceleration 新的加速度 (rad/s^2)
   */
  void set_acceleration(float acceleration);

  /**
   * @brief 重置规划器的状态
   * @param initial_velocity 可选的初始速度, 默认为0
   */
  void reset(float initial_velocity = 0.0f);

private:
  float max_velocity_;
  float acceleration_;
  const float dt_;

  // 每一步的最大速度变化量, 提前计算好以提高效率
  float max_delta_v_;
};

}  // namespace sp

#endif  // SP__VELOCITY_SMOOTHER_HPP