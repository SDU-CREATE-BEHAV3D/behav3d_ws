// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: Lucas Helle Pessot <luh@iti.sdu.dk>
// Maintainers:
//   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
//   - Joseph Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

// motion_visualizer.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

namespace behav3d::motion_visualizer
{

  class MotionVisualizer : public rclcpp::Node
  {
  public:
    // Single constructor: fully parameterized via ROS 2 NodeOptions/parameters
    explicit MotionVisualizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // Helpers ------------------------------------------------------------------
    void publishTargetPose(const geometry_msgs::msg::PoseStamped &pose,
                           const std::string &label = "target");
    void publishTargetPose(const std::vector<geometry_msgs::msg::PoseStamped> &poses);

    // /// Publish a single trajectory as a “trail”
    // void publishTrail(const moveit_msgs::msg::RobotTrajectory& traj,
    //                   const std::string& label = "trail");

    /// Publish a sequence of trajectories as a combined “trail”
    void publishTrail(const robot_trajectory::RobotTrajectoryPtr &traj_ptr,
                      const std::string &label = "trail");
    void deleteAllMarkers();
    void publishGhost(const moveit_msgs::msg::RobotTrajectory &traj);
    // Pause until press next in Rviz:
    void prompt(const std::string &text);
    void trigger();

  private:
    // Fixed configuration ------------------------------------------------------
    std::string root_link_;
    std::string eef_link_;

    // MoveIt handles -----------------------------------------------------------
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit_visual_tools::MoveItVisualToolsPtr vt_;
  };

} // namespace behav3d::motion_visualizer
