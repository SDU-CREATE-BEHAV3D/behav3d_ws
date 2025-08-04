// =============================================================================
//   ____  _____ _   _    ___     _______ ____  
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
                                              
                                              
// Author: Lucas José Helle <luh@iti.sdu.dk>
// Maintainers:
//   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
//   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
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

namespace behav3d::motion_visualizer {

class MotionVisualizer : public rclcpp::Node
{
public:
  MotionVisualizer(const std::string& planning_group,
                   const std::string& root_link,
                   const std::string& eef_link,
                   bool debug = false);

  // Helpers ------------------------------------------------------------------
  void publishTargetPose(const geometry_msgs::msg::PoseStamped& pose,
                         const std::string& label = "target");
  void deleteAllMarkers();
  void publishGhost(const moveit_msgs::msg::RobotTrajectory& traj);
  void publishTrail(const moveit_msgs::msg::RobotTrajectory& traj,
                    const std::string& label = "trail");

private:
  // Fixed configuration ------------------------------------------------------
  std::string root_link_;
  std::string eef_link_;

  // MoveIt handles -----------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit_visual_tools::MoveItVisualToolsPtr      vt_;
};

} // namespace behav3d::motion_visualizer
