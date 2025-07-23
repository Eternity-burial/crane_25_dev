// --- START OF FILE mpc_controller.cpp ---

#include "MPC_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MpcController::MpcController(
  float dt, int prediction_horizon, float q_x, float q_y, float q_yaw, float r_vx, float r_vy,
  float r_vw)
: dt_(dt),
  prediction_horizon_(prediction_horizon),
  q_x_(q_x),
  q_y_(q_y),
  q_yaw_(q_yaw),
  r_vx_(r_vx),
  r_vy_(r_vy),
  r_vw_(r_vw)
{
}

void MpcController::enable() { mode_ = ControlMode::VELOCITY; }

void MpcController::disable()
{
  mode_ = ControlMode::DISABLE;
  output_vx_ = 0.0f;
  output_vy_ = 0.0f;
  output_vw_ = 0.0f;
}

void MpcController::set_trajectory(const Trajectory & trajectory)
{
  target_trajectory_ = trajectory;
  current_target_idx_ = 0;
}

void MpcController::update(float current_x, float current_y, float current_yaw)
{
  if (mode_ == ControlMode::DISABLE || target_trajectory_.empty()) {
    disable();  // Ensure outputs are zero
    return;
  }

  RobotState current_state = {current_x, current_y, normalize_angle(current_yaw)};

  // --- Find the current reference point on the trajectory ---
  // This is a simple implementation that just advances along the trajectory.
  // A more robust solution would find the closest point on the path.
  float dist_to_target = std::sqrt(
    std::pow(target_trajectory_[current_target_idx_].x - current_state.x, 2) +
    std::pow(target_trajectory_[current_target_idx_].y - current_state.y, 2));

  // If we are close enough to the current target, move to the next one
  if (dist_to_target < 0.1f && current_target_idx_ < target_trajectory_.size() - 1) {
    current_target_idx_++;
  }

  // --- Create the reference trajectory for the prediction horizon ---
  Trajectory reference_trajectory;
  for (int i = 0; i < prediction_horizon_; ++i) {
    size_t index = std::min(current_target_idx_ + i, target_trajectory_.size() - 1);
    reference_trajectory.push_back(target_trajectory_[index]);
  }

  // --- Solve the MPC optimization problem ---
  solve(current_state, reference_trajectory);
}

void MpcController::solve(const RobotState & current_state, const Trajectory & reference_trajectory)
{
  // This is a simplified MPC solver using a grid search over possible control inputs.
  // For a real-world scenario, a more advanced solver (e.g., QP solver) would be used.

  float best_cost = std::numeric_limits<float>::max();
  float best_vx = 0.0f, best_vy = 0.0f, best_vw = 0.0f;

  // --- Define a discrete set of candidate control inputs to test ---
  // You can adjust these ranges and step sizes based on your robot's capabilities.
  const std::vector<float> vx_candidates = {-0.5f, 0.0f, 0.5f};  // m/s
  const std::vector<float> vy_candidates = {-0.5f, 0.0f, 0.5f};  // m/s
  const std::vector<float> vw_candidates = {-0.8f, 0.0f, 0.8f};  // rad/s

  for (float vx : vx_candidates) {
    for (float vy : vy_candidates) {
      for (float vw : vw_candidates) {
        float current_cost = 0.0f;
        RobotState predicted_state = current_state;

        // --- Calculate cost over the prediction horizon ---
        for (int i = 0; i < prediction_horizon_; ++i) {
          // Predict state one step forward
          predicted_state = predict_state(predicted_state, vx, vy, vw);

          // Get the reference state for this step
          const RobotState & reference_state = reference_trajectory[i];

          // Calculate state error cost
          float error_x = reference_state.x - predicted_state.x;
          float error_y = reference_state.y - predicted_state.y;
          float error_yaw = normalize_angle(reference_state.yaw - predicted_state.yaw);
          current_cost += q_x_ * error_x * error_x;
          current_cost += q_y_ * error_y * error_y;
          current_cost += q_yaw_ * error_yaw * error_yaw;
        }

        // Add control effort cost (we assume control is constant over the horizon)
        current_cost += r_vx_ * vx * vx;
        current_cost += r_vy_ * vy * vy;
        current_cost += r_vw_ * vw * vw;

        // --- Check if this is the best control input so far ---
        if (current_cost < best_cost) {
          best_cost = current_cost;
          best_vx = vx;
          best_vy = vy;
          best_vw = vw;
        }
      }
    }
  }

  // --- Set the best found control input as the output ---
  output_vx_ = best_vx;
  output_vy_ = best_vy;
  output_vw_ = best_vw;
}

RobotState MpcController::predict_state(
  const RobotState & initial_state, float vx, float vy, float vw) const
{
  RobotState predicted_state;
  // Note: vx and vy are in the robot's frame. We must rotate them to the world frame.
  float cos_yaw = std::cos(initial_state.yaw);
  float sin_yaw = std::sin(initial_state.yaw);

  predicted_state.x = initial_state.x + (vx * cos_yaw - vy * sin_yaw) * dt_;
  predicted_state.y = initial_state.y + (vx * sin_yaw + vy * cos_yaw) * dt_;
  predicted_state.yaw = normalize_angle(initial_state.yaw + vw * dt_);

  return predicted_state;
}

float MpcController::normalize_angle(float angle) const
{
  while (angle > M_PI) angle -= 2.0f * M_PI;
  while (angle < -M_PI) angle += 2.0f * M_PI;
  return angle;
}