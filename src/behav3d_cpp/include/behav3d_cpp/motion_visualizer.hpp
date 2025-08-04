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

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace behav3d
{

/**
 * \brief Light wrapper around MoveItVisualTools that offers three
 *        helpers. Everything is
 *        forwarded to the supplied MoveItVisualTools instance.
 */
class MotionVisualizer
{
public:
  MotionVisualizer(moveit_visual_tools::MoveItVisualToolsPtr vt,
                   const moveit::core::JointModelGroup*       jmg,
                   const moveit::core::LinkModel*             tip_link);

  // --- API you requested ----------------------------------------------------
  void publishGhost   (const moveit_msgs::msg::RobotTrajectory& traj);
  void publishTrail   (const moveit_msgs::msg::RobotTrajectory& traj);
  void publishTargetPose(const geometry_msgs::msg::PoseStamped& pose);

private:
  moveit_visual_tools::MoveItVisualToolsPtr vt_;
  const moveit::core::JointModelGroup*      jmg_;
  const moveit::core::LinkModel*            tip_;
};

}  // namespace behav3d
