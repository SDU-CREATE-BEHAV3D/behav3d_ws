// =============================================================================
//   ____  _____ _   _    ___     _______ ____  
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
                                              
                                              
// Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Maintainers:
//   - Lucas José Helle <luh@iti.sdu.dk>
//   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/kinematic_constraints/utils.hpp>

namespace rt = robot_trajectory;
using RobotTrajectory       = rt::RobotTrajectory;
using RobotTrajectoryPtr    = std::shared_ptr<RobotTrajectory>;

/**
 * @class PilzMotionController
 * @brief High-level helper around MoveIt2 + PILZ planner
 */
class PilzMotionController : public rclcpp::Node
{
public:
  using MoveGroupSequence  = moveit_msgs::action::MoveGroupSequence;

  explicit PilzMotionController(const std::string & group      = "ur_arm",
                                const std::string & root_link  = "ur10e_base_link",
                                const std::string & eef_link   = "ur10e_tool0",
                                bool               debug       = false);

  /// Plan either a PTP or LIN move according to motion_type ("PTP"/"LIN")
  RobotTrajectoryPtr planTarget(const geometry_msgs::msg::PoseStamped & target,
                                const std::string & motion_type = "PTP",
                                double vel_scale = 0.5,
                                double acc_scale = 0.5);

  /// Plan a joint‑space PTP move
  RobotTrajectoryPtr planJoints(const std::vector<double> & joint_positions,
                                double vel_scale = 0.5,
                                double acc_scale = 0.5);

  /// Plan a blended LIN sequence through waypoints
  RobotTrajectoryPtr planSequence(const std::vector<geometry_msgs::msg::PoseStamped> & waypoints,
                                  double blend_radius   = 0.001,
                                  double vel_scale      = 0.5,
                                  double acc_scale      = 0.5,
                                  double pos_tolerance  = 0.001,
                                  double ori_telerance  = 0.001);
  /// Get current end-effector pose in planning frame
  geometry_msgs::msg::PoseStamped getCurrentPose() const;

  /// Get current joint state for the planning group
  sensor_msgs::msg::JointState getCurrentJointState() const;

  bool executeTrajectory(const RobotTrajectoryPtr & traj,
                         bool apply_totg = false);

  moveit::core::RobotStatePtr computeIK(const geometry_msgs::msg::PoseStamped & pose,
                                        double timeout = 0.1) const;

  geometry_msgs::msg::PoseStamped computeFK(const moveit::core::RobotState & state) const;

  bool isReachable(const geometry_msgs::msg::PoseStamped & pose) const;

  /// Cancel all active sequence goals
  void cancelAllGoals();

  /// Accessors for planning‑frame links
  const std::string & getRootLink() const;
  const std::string & getEefLink()  const;

private:
  std::string root_link_;
  std::string eef_link_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr sequence_client_;
};