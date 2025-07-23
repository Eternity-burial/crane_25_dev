// --- START OF FILE mpc_controller.hpp ---

#ifndef MPC_CONTROLLER_HPP
#define MPC_CONTROLLER_HPP

#include <cstddef>
#include <vector>

#include "controllers/control_mode.hpp"

// A simple structure to hold the state of the robot (position and orientation)
struct RobotState
{
  float x = 0.0f;
  float y = 0.0f;
  float yaw = 0.0f;
};

// A trajectory is defined as a vector of robot states (waypoints)
using Trajectory = std::vector<RobotState>;

class MpcController
{
public:
  /**
   * @brief Construct a new Mpc Controller object
   * @param dt The time step (in seconds) between control updates.
   * @param prediction_horizon The number of steps to predict into the future.
   * @param q_x Weight for the x-position error in the cost function.
   * @param q_y Weight for the y-position error in the cost function.
   * @param q_yaw Weight for the yaw-orientation error in the cost function.
   * @param r_vx Weight for the linear velocity vx in the cost function (penalizes high effort).
   * @param r_vy Weight for the linear velocity vy in the cost function.
   * @param r_vw Weight for the angular velocity vw in the cost function.
   */
  MpcController(
    float dt, int prediction_horizon, float q_x, float q_y, float q_yaw, float r_vx, float r_vy,
    float r_vw);

  /**
   * @brief Enable the controller.
   */
  void enable();

  /**
   * @brief Disable the controller and reset outputs to zero.
   */
  void disable();

  /**
   * @brief Set the target trajectory for the controller to follow.
   * @param trajectory The desired path for the robot.
   */
  void set_trajectory(const Trajectory & trajectory);

  /**
   * @brief Update the controller with the current state and calculate the optimal control command.
   * @param current_x The current x position from sensors.
   * @param current_y The current y position from sensors.
   * @param current_yaw The current yaw angle from sensors (in radians).
   */
  void update(float current_x, float current_y, float current_yaw);

  // --- Output Getters ---
  float get_vx() const { return output_vx_; }
  float get_vy() const { return output_vy_; }
  float get_vw() const { return output_vw_; }

private:
  /**
   * @brief The core MPC optimization logic.
   * 
   * Finds the best control input [vx, vy, vw] by evaluating a set of candidate inputs
   * and choosing the one that minimizes a cost function over the prediction horizon.
   * 
   * @param current_state The current state of the robot.
   * @param reference_trajectory The segment of the target trajectory to follow.
   */
  void solve(const RobotState & current_state, const Trajectory & reference_trajectory);

  /**
   * @brief Predicts the future state of the robot given a starting state and control inputs.
   * @param initial_state The starting state.
   * @param vx The linear velocity in the robot's x-direction.
   * @param vy The linear velocity in the robot's y-direction.
   * @param vw The angular velocity around the robot's z-axis.
   * @return The predicted state after one time step (dt).
   */
  RobotState predict_state(const RobotState & initial_state, float vx, float vy, float vw) const;

  /**
   * @brief Normalizes an angle to the range [-PI, PI].
   * @param angle The angle in radians.
   * @return The normalized angle.
   */
  float normalize_angle(float angle) const;

  // --- MPC Parameters ---
  float dt_;                // Time step
  int prediction_horizon_;  // Prediction horizon (N)

  // --- Cost Function Weights ---
  float q_x_, q_y_, q_yaw_;   // State error weights
  float r_vx_, r_vy_, r_vw_;  // Control input weights

  // --- Controller State ---
  ControlMode mode_ = ControlMode::DISABLE;
  Trajectory target_trajectory_;
  size_t current_target_idx_ = 0;

  // --- Calculated Outputs ---
  float output_vx_ = 0.0f;
  float output_vy_ = 0.0f;
  float output_vw_ = 0.0f;
};

#endif  // MPC_CONTROLLER_HPP