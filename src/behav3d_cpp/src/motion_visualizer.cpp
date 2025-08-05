#include "behav3d_cpp/motion_visualizer.hpp"

#define PMV_DEBUG(node, fmt, ...) RCLCPP_DEBUG((node)->get_logger(), "[PMV] " fmt, ##__VA_ARGS__)
#define PMV_INFO(node, fmt, ...) RCLCPP_INFO((node)->get_logger(), "[PMV] " fmt, ##__VA_ARGS__)

namespace behav3d::motion_visualizer
{
  // ─────────────────────────────────────────────────────────────────────────────
  MotionVisualizer::MotionVisualizer(const std::string &planning_group,
                                     const std::string &root_link,
                                     const std::string &eef_link,
                                     bool debug)
      : Node("motion_visualizer_cpp"),
        root_link_(root_link),
        eef_link_(eef_link),
        // Node → MoveGroupInterface
        move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto *) {}), planning_group)
  {
    if (debug)
      this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    PMV_INFO(this, "MotionVisualizer init: group=%s, root=%s, eef=%s, debug=%s",
             planning_group.c_str(), root_link_.c_str(), eef_link_.c_str(),
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

  void MotionVisualizer::prompt(const std::string &text)
  {
    PMV_INFO(this, "Prompt RViz: '%s'", text.c_str());
    // Origianl call to MoveItVisualTools:
    vt_->prompt(text);
    PMV_INFO(this, "Continuando tras prompt");
  }

} // namespace behav3d::motion_visualizer
