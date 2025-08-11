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

#include "behav3d_cpp/session_manager.hpp"
#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/camera_manager.hpp"

#include <algorithm>
#include <cmath>  

#define SESS_INFO(node, fmt, ...) \
  RCLCPP_INFO((node)->get_logger(), "[SessionManager] " fmt, ##__VA_ARGS__)

namespace behav3d::session_manager {

SessionManager::SessionManager(
    const std::shared_ptr<motion_controller::PilzMotionController>& ctrl,
    const std::shared_ptr<motion_visualizer::MotionVisualizer>& viz,
    const std::shared_ptr<camera_manager::CameraManager>& cam)
  : rclcpp::Node("session_manager_cpp"),
    ctrl_(ctrl),
    viz_(viz),
    cam_(cam)
{
  RCLCPP_INFO(this->get_logger(),
              "[SessionManager] initialized (ctrl:%s viz:%s cam:%s)",
              ctrl_ ? "yes" : "no",
              viz_  ? "yes" : "no",
              cam_  ? "yes" : "no");
}
void SessionManager::sayHello(const std::string& who) {
  RCLCPP_INFO(this->get_logger(), "[SessionManager] Hello, %s 👋", who.c_str());
}

void SessionManager::home() {
  // Default joint-space home (degrees)
  const std::vector<double> home_deg = {45.0, -120.0, 120.0, -90.0, 90.0, -180.0};

  std::vector<double> home_rad;
  home_rad.reserve(home_deg.size());
  std::transform(home_deg.begin(), home_deg.end(), std::back_inserter(home_rad),
                 [](double d){ return d * M_PI / 180.0; });

  auto traj = ctrl_->planJoints(home_rad);
  if (!traj) {
    RCLCPP_ERROR(get_logger(), "[SessionManager] home(): planJoints failed");
    return;
  }
  ctrl_->executeTrajectory(traj, /*apply_totg=*/true);
}

void SessionManager::home(
    const geometry_msgs::msg::PoseStamped& target,
    const std::string& motion_type,
    double vel_scale,
    double acc_scale) {

  auto traj = ctrl_->planTarget(target, motion_type, vel_scale, acc_scale);
  if (!traj) {
    RCLCPP_ERROR(get_logger(), "[SessionManager] home(target): planTarget failed");
    return;
  }
  ctrl_->executeTrajectory(traj, /*apply_totg=*/true);
}
//SinglePose Init Scan
bool SessionManager::initScan(const std::string& filename,
                              const geometry_msgs::msg::PoseStamped& pose)
{
  SESS_INFO(this, "initScan(single): file='%s', pose=(%.3f, %.3f, %.3f) in '%s'",
            filename.c_str(),
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            pose.header.frame_id.c_str());

  // Show it in RViz right away so you *see* it landed
  if (viz_) {
    viz_->publishTargetPose(pose, "init_target");
  }
  return true;
}

//Multiple poses Init Scan
bool SessionManager::initScan(const std::string& filename,
                              const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
  SESS_INFO(this, "initScan(batch): file='%s', count=%zu", filename.c_str(), poses.size());

  // Optional: visualize all targets at once
  if (viz_ && !poses.empty()) {
    viz_->publishTargetPose(poses); // your batch overload
  }
  return true;
}
}  // namespace behav3d::session_manager
