// --- START OF FILE main.cpp ---

#include <cmath>
#include <iomanip>  // 用于格式化输出
#include <iostream>
#include <vector>

#include "MPC_controller.hpp"  // 包含你的控制器头文件

// --- 假设 control_mode.hpp 包含以下内容 ---
// enum class ControlMode { DISABLE, VELOCITY };
// ---

// (在此处粘贴上面的 create_square_trajectory 函数)
Trajectory create_square_trajectory(float side_length = 2.0f, float point_density = 10.0f)
{
  // ... 函数实现如上 ...
}

int main()
{
  // --- 1. 设置控制器参数 ---
  float dt = 0.1f;                 // 控制周期 (秒)
  int prediction_horizon = 10;     // 预测 N=10 步
  float q_x = 10.0f, q_y = 10.0f;  // 位置误差权重 (我们更关心位置)
  float q_yaw = 2.0f;              // 角度误差权重
  float r_vx = 0.1f, r_vy = 0.1f;  // 线速度控制量权重 (惩罚过大的速度)
  float r_vw = 0.1f;               // 角速度控制量权重

  // --- 2. 实例化控制器 ---
  MpcController controller(dt, prediction_horizon, q_x, q_y, q_yaw, r_vx, r_vy, r_vw);

  // --- 3. 创建并设置轨迹 ---
  std::cout << "Creating a square trajectory..." << std::endl;
  Trajectory target_trajectory = create_square_trajectory();
  controller.set_trajectory(target_trajectory);
  std::cout << "Trajectory created with " << target_trajectory.size() << " points." << std::endl;

  // --- 4. 初始化模拟机器人状态 ---
  RobotState simulated_robot = {0.0f, 0.0f, 0.0f};  // 机器人从原点开始

  // --- 5. 启用控制器并开始模拟 ---
  controller.enable();
  std::cout << "\nStarting simulation..." << std::endl;
  std::cout << std::fixed << std::setprecision(3);

  const int simulation_steps = 300;  // 模拟30秒
  for (int i = 0; i < simulation_steps; ++i) {
    // --- a. 控制器更新 ---
    // 在真实机器人上，这里会传入来自传感器的数据
    controller.update(simulated_robot.x, simulated_robot.y, simulated_robot.y);

    // --- b. 获取控制输出 ---
    float vx_cmd = controller.get_vx();
    float vy_cmd = controller.get_vy();
    float vw_cmd = controller.get_vw();

    // --- c. 打印状态信息 (可选) ---
    // 我们可以从控制器内部获取当前的目标点索引来比较
    // 注意：这是一个简化的访问方式，在实际代码中你可能需要一个getter
    // size_t current_target_idx = controller.current_target_idx_;
    // const auto& target_point = target_trajectory[current_target_idx];
    std::cout << "Step " << i << ": "
              << "Robot(x,y,y)=(" << simulated_robot.x << ", " << simulated_robot.y << ", "
              << simulated_robot.yaw
              << ") "
              // << "Target(x,y)=(" << target_point.x << ", " << target_point.y << ") "
              << "Cmd(vx,vy,vw)=(" << vx_cmd << ", " << vy_cmd << ", " << vw_cmd << ")"
              << std::endl;

    // --- d. 更新模拟机器人的状态 (运动学模型) ---
    // 注意：这个模型必须与控制器内部的 `predict_state` 模型一致
    float cos_yaw = std::cos(simulated_robot.yaw);
    float sin_yaw = std::sin(simulated_robot.yaw);
    simulated_robot.x += (vx_cmd * cos_yaw - vy_cmd * sin_yaw) * dt;
    simulated_robot.y += (vx_cmd * sin_yaw + vy_cmd * cos_yaw) * dt;
    simulated_robot.yaw += vw_cmd * dt;
    // 保持yaw在[-PI, PI]范围内
    while (simulated_robot.yaw > M_PI) simulated_robot.yaw -= 2.0f * M_PI;
    while (simulated_robot.yaw < -M_PI) simulated_robot.yaw += 2.0f * M_PI;

    // --- e. 检查终止条件 (可选) ---
    float dist_to_final = std::sqrt(
      std::pow(target_trajectory.back().x - simulated_robot.x, 2) +
      std::pow(target_trajectory.back().y - simulated_robot.y, 2));
    if (dist_to_final < 0.15f) {
      std::cout << "\nRobot has reached the end of the trajectory." << std::endl;
      break;
    }
  }

  controller.disable();
  std::cout << "Simulation finished. Final command: vx=" << controller.get_vx()
            << ", vw=" << controller.get_vw() << std::endl;

  return 0;
}