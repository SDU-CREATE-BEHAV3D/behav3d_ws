// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
//                                                
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
#include <std_srvs/srv/trigger.hpp>  // Added for capture service

#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/trajectory_builder.hpp"
#include "behav3d_cpp/util.hpp"

using behav3d::motion_controller::PilzMotionController;
using behav3d::motion_visualizer::MotionVisualizer;
using behav3d::target_builder::flipTarget;
using behav3d::target_builder::worldXY;
using behav3d::trajectory_builder::fibonacciSphericalCap;
using behav3d::trajectory_builder::sweepZigzag;
using behav3d::util::deg2rad;
using std::placeholders::_1;

// ---------------------------------------------------------------------------
//                                Demo Node
// ---------------------------------------------------------------------------
class PilzDemo : public rclcpp::Node
{
public:
  explicit PilzDemo(const std::shared_ptr<PilzMotionController> &ctrl,
                    const std::shared_ptr<MotionVisualizer> &viz)
      : Node("pilz_demo_cpp"), ctrl_(ctrl), viz_(viz)
  {
    // Create subscription for user commands
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&PilzDemo::callback, this, _1));

    // Create client for the /capture service
    capture_client_ = this->create_client<std_srvs::srv::Trigger>("capture");
    // Wait for service to appear
    if (!capture_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Capture service not available after 5s");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "PilzDemo ready. Commands: 'home', 'draw_line', 'draw_square',"
                " 'draw_square_seq', 'draw_circle', 'draw_circle_seq', 'grid_sweep', 'fibonacci_cap', 'quit'");
  }

private:
  std::shared_ptr<MotionVisualizer> viz_;
  std::shared_ptr<PilzMotionController> ctrl_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  // Helper to call the capture service
  bool callCapture()
  {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    RCLCPP_INFO(get_logger(), "Triggering capture service...");
    auto future = capture_client_->async_send_request(req);
    using namespace std::chrono_literals;
    if (future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Capture service call timed out");
      return false;
    }
    auto res = future.get();
    if (res->success) {
      RCLCPP_INFO(get_logger(), "Capture succeeded: %s", res->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Capture failed: %s", res->message.c_str());
    }
    return res->success;
  }

  // Command dispatcher
  void callback(const std_msgs::msg::String &msg)
  {
    const auto &cmd = msg.data;
    if (cmd == "home")             home();
    else if (cmd == "draw_square") draw_square();
    else if (cmd == "draw_square_seq") draw_square_seq();
    else if (cmd == "draw_circle") draw_circle();
    else if (cmd == "draw_circle_seq") draw_circle_seq();
    else if (cmd == "draw_line")   draw_line();
    else if (cmd == "grid_sweep")  grid_sweep();
    else if (cmd == "fibonacci_cap") fibonacci_cap();
    else if (cmd == "quit")        rclcpp::shutdown();
    else RCLCPP_WARN(get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  // Move to home without capturing
  void home()
  {
    const std::vector<double> home_deg = {45,-120,120,-90,90,-180};
    std::vector<double> home_rad;
    home_rad.reserve(home_deg.size());
    std::transform(home_deg.begin(), home_deg.end(), std::back_inserter(home_rad),
                   [](double d){ return d * M_PI / 180.0; });
    auto traj = ctrl_->planJoints(home_rad);
    ctrl_->executeTrajectory(traj);
  }

  // Draw square with capture at each corner (blocking exec)
  void draw_square(double side = 0.4, double z_fixed = 0.4)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()));
    // Block until PTP motion completes
    ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"), true);
    callCapture();
    double half = side/2.0;
    std::vector<std::pair<double,double>> off = {{-half,-half},{-half,half},{half,half},{half,-half},{-half,-half}};
    for (auto [dx,dy] : off) {
      auto p = center;
      p.pose.position.x += dx;
      p.pose.position.y += dy;
      auto t = ctrl_->planTarget(p, "LIN");
      // Block until LIN motion completes
      ctrl_->executeTrajectory(t, true);
      callCapture();
    }
    home();
  }

  // Draw square sequence then capture (already blocking)
  void draw_square_seq(double side=0.4,double z_fixed=0.4,double blend=0.001)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()));
    std::vector<geometry_msgs::msg::PoseStamped> wp;
    double half = side/2.0;
    for (auto d : std::vector<std::pair<double,double>>{{-half,-half},{-half,half},{half,half},{half,-half},{-half,-half}}) {
      auto p = center;
      p.pose.position.x += d.first;
      p.pose.position.y += d.second;
      wp.push_back(p);
    }
    auto traj = ctrl_->planSequence(wp, blend);
    // True to block until done
    ctrl_->executeTrajectory(traj, true);
    callCapture();
    home();
  }

  // Draw circle with capture at each step (blocking)
  void draw_circle(double radius=0.3,double z_fixed=0.4,int div=36)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()));
    ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"), true);
    callCapture();
    for (int i = 0; i <= div; ++i) {
      double a = 2.0 * M_PI * i / div;
      auto p = center;
      p.pose.position.x += radius * std::cos(a);
      p.pose.position.y += radius * std::sin(a);
      auto t = ctrl_->planTarget(p, "LIN");
      ctrl_->executeTrajectory(t, true);
      callCapture();
    }
    home();
  }

  // Draw blended circle then capture
  void draw_circle_seq(double radius=0.3,double z_fixed=0.4,int div=36,double blend=0.001)
  {
    home();
    std::vector<geometry_msgs::msg::PoseStamped> wp;
    for (int i = 0; i <= div; ++i) {
      double a = 2.0 * M_PI * i / div;
      auto p = flipTarget(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()));
      p.pose.position.x += radius * std::cos(a);
      p.pose.position.y += radius * std::sin(a);
      wp.push_back(p);
    }
    auto traj = ctrl_->planSequence(wp, blend);
    ctrl_->executeTrajectory(traj, true);
    callCapture();
    home();
  }

  // Draw a line with two captures (blocking)
  void draw_line()
  {
    home();
    auto start = flipTarget(worldXY(-0.2, 0.4, 0.4, ctrl_->getRootLink()));
    viz_->publishTargetPose(start, "start");
    auto end = flipTarget(worldXY(0.2, 0.8, 0.8, ctrl_->getRootLink()));
    viz_->publishTargetPose(end, "end"); viz_->prompt("Press 'next'...");
    ctrl_->executeTrajectory(ctrl_->planTarget(start, "PTP"), true);
    callCapture();
    ctrl_->executeTrajectory(ctrl_->planTarget(end, "LIN"), true);
    callCapture();
    home(); viz_->deleteAllMarkers();
  }

  // Spherical cap scan with captures (blocking)
  void fibonacci_cap(double radius=0.5,double cx=0.0,double cy=1.0,double cz=0.0,double cap=30.0,int n=10)
  {
    home();
    double cap_rad = deg2rad(cap);
    auto center = worldXY(cx, cy, cz, ctrl_->getRootLink());
    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n);
    if (targets.empty()) { RCLCPP_WARN(get_logger(), "no targets"); return; }
    viz_->publishTargetPose(targets);
    // Block PTP then capture
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"), true);
    callCapture();
    for (size_t i = 1; i < targets.size(); ++i) {
      auto t = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(t, true);
      callCapture();
    }
    viz_->deleteAllMarkers(); home();
  }

  // Grid sweep with captures (blocking)
  void grid_sweep(double width=0.6,double height=0.6,double cx=0.0,double cy=0.7,double cz=0.0,double zoff=0.5,int nx=6,int ny=6,bool row=false)
  {
    home();
    auto center = worldXY(cx, cy, cz, ctrl_->getRootLink());
    auto targets = sweepZigzag(center, width, height, zoff, std::max(2,nx), std::max(2,ny), row);
    if (targets.empty()) { RCLCPP_WARN(get_logger(), "no targets"); return; }
    viz_->publishTargetPose(targets);
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"), true);
    callCapture();
    for (size_t i = 1; i < targets.size(); ++i) {
      auto t = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(t, true);
      callCapture();
    }
    viz_->deleteAllMarkers(); home();
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
  auto demo = std::make_shared<PilzDemo>(controller, visualizer);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);
  exec.add_node(demo);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
