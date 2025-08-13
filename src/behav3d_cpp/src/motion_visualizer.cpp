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
// TODO: Marker Scale
// TODO: Trail Color

#include "behav3d_cpp/motion_visualizer.hpp"

#define PMV_DEBUG(node, fmt, ...) RCLCPP_DEBUG((node)->get_logger(), "[PMV] " fmt, ##__VA_ARGS__)
#define PMV_INFO(node, fmt, ...) RCLCPP_INFO((node)->get_logger(), "[PMV] " fmt, ##__VA_ARGS__)

namespace behav3d::motion_visualizer
{
  // ─────────────────────────────────────────────────────────────────────────────
  MotionVisualizer::MotionVisualizer(const rclcpp::NodeOptions &options)
      : Node("motion_visualizer_cpp", options),
        root_link_(this->declare_parameter<std::string>("root_link", "world")),
        eef_link_(this->declare_parameter<std::string>("eef_link", "femto__depth_optical_frame")),
        // Node → MoveGroupInterface
        move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto *) {}),
                    this->declare_parameter<std::string>("group", "ur_arm"))
  {
    const bool debug = this->declare_parameter<bool>("debug", false);
    if (debug)
      this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    PMV_INFO(this, "MotionVisualizer init: group=%s, root=%s, eef=%s, debug=%s",
             this->get_parameter("group").as_string().c_str(),
             root_link_.c_str(), eef_link_.c_str(),
             debug ? "true" : "false");

    move_group_.setPoseReferenceFrame(root_link_);
    move_group_.setEndEffectorLink(eef_link_);

    // Create & prime MoveItVisualTools ----------------------------------------
    vt_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(), root_link_, "rviz_visual_tools",
        move_group_.getRobotModel());

    vt_->deleteAllMarkers();
    vt_->loadRemoteControl();
    PMV_DEBUG(this, "MoveItVisualTools ready");
  }

  // ── Helpers ─────────────────────────────────────────────────────────────────
  void MotionVisualizer::publishTargetPose(const geometry_msgs::msg::PoseStamped &pose,
                                           const std::string &label)
  {
    PMV_DEBUG(this, "publishTargetPose: (%.3f,%.3f,%.3f)",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    vt_->publishAxisLabeled(pose.pose, label);
    vt_->trigger();
  }

  void MotionVisualizer::publishTargetPose(
      const std::vector<geometry_msgs::msg::PoseStamped> &poses)
  {
    PMV_DEBUG(this, "publishTargetPose batch: %zu poses", poses.size());

    for (size_t i = 0; i < poses.size(); ++i)
    {
      // give each pose a unique label
      const auto &ps = poses[i];
      vt_->publishAxisLabeled(ps.pose, "t" + std::to_string(i));
    }
    vt_->trigger();
  }
  void MotionVisualizer::deleteAllMarkers()
  {
    vt_->deleteAllMarkers();
    vt_->trigger();
  }

  void MotionVisualizer::trigger()
  {
    vt_->trigger();
  }
  // Trail visualization for SEQUENCES:
  void MotionVisualizer::publishTrail(
      const robot_trajectory::RobotTrajectoryPtr &traj_ptr,
      const std::string &label)
  {
    PMV_DEBUG(this, "publishTrail RTTPtr: label=%s, points=%zu",
              label.c_str(), traj_ptr->getWayPointCount());
    // get the tip link once
    const auto *tip_link =
        move_group_.getRobotModel()->getLinkModel(eef_link_);
    // draw the line from the RobotTrajectoryPtr, no JMG needed
    vt_->publishTrajectoryLine(traj_ptr, tip_link);
    // optional: publish a text label
    vt_->publishText(Eigen::Isometry3d::Identity(), label,
                     rviz_visual_tools::WHITE, rviz_visual_tools::LARGE);
    // flush all markers
    vt_->trigger();
  }
  void MotionVisualizer::prompt(const std::string &text)
  {
    PMV_INFO(this, "Prompt RViz: '%s'", text.c_str());
    // Original call to MoveItVisualTools:
    vt_->prompt(text);
    PMV_INFO(this, "Continuando tras prompt");
  }

} // namespace behav3d::motion_visualizer
