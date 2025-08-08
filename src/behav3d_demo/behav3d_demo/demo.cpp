// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Maintainers:
//   - Lucas José Helle <luh@iti.sdu.dk>
//   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/trajectory_builder.hpp"
#include "behav3d_cpp/util.hpp"
#include "behav3d_cpp/camera_manager.hpp"

using behav3d::motion_controller::PilzMotionController;
using behav3d::motion_visualizer::MotionVisualizer;

using behav3d::target_builder::flipTargetAxes;
using behav3d::target_builder::worldXY;
using behav3d::target_builder::worldXZ;
using behav3d::trajectory_builder::fibonacciSphericalCap;
using behav3d::trajectory_builder::sweepZigzag;
using behav3d::util::deg2rad;

using std::placeholders::_1;

// ---------------------------------------------------------------------------
//                                Demo Node
// ---------------------------------------------------------------------------
class Behav3dDemo : public rclcpp::Node
{
public:
  explicit Behav3dDemo(const std::shared_ptr<PilzMotionController> &ctrl,
                    const std::shared_ptr<MotionVisualizer> &viz,
                    const std::shared_ptr<behav3d::camera::CameraManager> &cam)
      : Node("behav3d_demo"), ctrl_(ctrl), viz_(viz), cam_(cam)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&Behav3dDemo::callback, this, _1));

    // Declare and get capture_delay_sec parameter
    capture_delay_sec_ = this->declare_parameter<double>("capture_delay_sec", 0.5);

    RCLCPP_INFO(this->get_logger(),
                "Behav3dDemo ready. Commands: 'fibonacci_cap', 'grid_sweep', 'quit'. Capture delay: %.2fs", capture_delay_sec_);
  }

private:
  std::shared_ptr<MotionVisualizer> viz_;
  std::shared_ptr<PilzMotionController> ctrl_;
  std::shared_ptr<behav3d::camera::CameraManager> cam_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  double capture_delay_sec_;

  void callback(const std_msgs::msg::String &msg)
  {
    const std::string cmd = msg.data;
    if (cmd == "fibonacci_cap")
      fibonacci_cap();
    else if (cmd == "grid_sweep")
      grid_sweep();
    else if (cmd == "quit")
      rclcpp::shutdown();
    else
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  void home()
  {
    // Joint‑space “home” configuration (given in degrees)
    const std::vector<double> home_joints_deg = {45.0, -120.0, 120.0, -90.0, 90.0, -180.0};
    std::vector<double> home_joints_rad;
    home_joints_rad.reserve(home_joints_deg.size());
    std::transform(home_joints_deg.begin(), home_joints_deg.end(),
                   std::back_inserter(home_joints_rad),
                   [](double deg)
                   { return deg * M_PI / 180.0; });
    auto traj = ctrl_->planJoints(home_joints_rad);
    ctrl_->executeTrajectory(traj);
  }

  void fibonacci_cap(double radius = 0.75,
                     double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                     double cap_deg = 30.0, int n_points = 32)
  {
    // 1. Start from home
    home();
    const double cap_rad = deg2rad(cap_deg);
    const auto center = worldXY(center_x, center_y, center_z,
                                ctrl_->getRootLink());
    viz_->publishTargetPose(center);
    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n_points);
    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "fibonacci_cap: no targets generated!");
      return;
    }
    viz_->publishTargetPose(targets);
    // Loop over all targets (including first)
    for (size_t i = 0; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to move to target " + std::to_string(i) + "/" + std::to_string(targets.size()-1));
      std::string motion_type = (i == 0) ? "PTP" : "LIN";
      auto traj = ctrl_->planTarget(targets[i], motion_type);
      ctrl_->executeTrajectory(traj);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(capture_delay_sec_)));
      if (cam_ && !cam_->captureAsync())
      {
        RCLCPP_WARN(this->get_logger(), "CameraManager: capture not ready after %s to target %zu.", motion_type.c_str(), i);
      }
      if (cam_) cam_->waitForIdle();
    }
    viz_->deleteAllMarkers();
    home();
  }

  void grid_sweep(double width = 1.0, double height = 0.5,
                  double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                  double z_off = 0.75,
                  int nx = 10, int ny = 5,
                  bool row_major = false)
  {
    home();
    const auto center = worldXY(center_x, center_y, center_z,
                                ctrl_->getRootLink());
    viz_->publishTargetPose(center);
    nx = std::max(2, nx);
    ny = std::max(2, ny);
    auto targets = sweepZigzag(center, width, height, z_off,
                               nx, ny, row_major);
    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "grid_sweep/sweepZigzag: no targets generated!");
      return;
    }
    viz_->publishTargetPose(targets);
    // Loop over all targets (including first)
    for (size_t i = 0; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to move to target " + std::to_string(i) + "/" + std::to_string(targets.size()-1));
      std::string motion_type = (i == 0) ? "PTP" : "LIN";
      auto traj = ctrl_->planTarget(targets[i], motion_type);
      ctrl_->executeTrajectory(traj);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(capture_delay_sec_)));
      if (cam_ && !cam_->captureAsync())
      {
        RCLCPP_WARN(this->get_logger(), "CameraManager: capture not ready after %s to target %zu.", motion_type.c_str(), i);
      }
      if (cam_) cam_->waitForIdle();
    }
    viz_->deleteAllMarkers();
    home();
  }
};

// ---------------------------------------------------------------------------
//                                   main()
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<PilzMotionController>(
      "ur_arm",
      "world",
      "femto__depth_optical_frame",
      true);
  auto visualizer = std::make_shared<MotionVisualizer>(
      "ur_arm",
      "world",
      "femto__depth_optical_frame");
  auto camera = std::make_shared<behav3d::camera::CameraManager>(
      rclcpp::NodeOptions().use_intra_process_comms(true));
  auto demo = std::make_shared<Behav3dDemo>(controller, visualizer, camera);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);
  exec.add_node(camera);
  exec.add_node(demo);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}