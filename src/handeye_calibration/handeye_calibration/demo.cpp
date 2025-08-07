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
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    // Subscription to user commands
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&PilzDemo::callback, this, _1));

    // Service client for capture API
    capture_client_ = this->create_client<std_srvs::srv::Trigger>("capture");

    RCLCPP_INFO(this->get_logger(),
                "PilzDemo ready. Commands: 'home', 'draw_line', 'draw_square',"
                " 'draw_square_seq', 'draw_circle', 'draw_circle_seq', 'grid_sweep', 'fibonacci_cap', 'quit'");
  }

private:
  std::shared_ptr<PilzMotionController> ctrl_;
  std::shared_ptr<MotionVisualizer> viz_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;

  // Helper to call the capture service (non-blocking, no shutdown)
  bool callCapture()
  {
    // If the service isn't already up, just warn and skip
    if (!capture_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Capture service not available right now; skipping capture.");
      return false;
    }

    // Build & send the request
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    RCLCPP_INFO(get_logger(), "Triggering capture service...");
    capture_client_->async_send_request(req,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(get_logger(), "Capture succeeded: %s", res->message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Capture reported failure: %s", res->message.c_str());
        }
      }
    );

    // Return immediately; the lambda will log the result later
    return true;
  }


  // ------------------------------------------------------------------------
  //  Command dispatcher
  // ------------------------------------------------------------------------
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

  // ------------------------------------------------------------------------
  //  Command implementations
  // ------------------------------------------------------------------------
  void home()
  {
    const std::vector<double> home_deg = {45.0, -120.0, 120.0, -90.0, 90.0, -180.0};
    std::vector<double> home_rad;
    home_rad.reserve(home_deg.size());
    std::transform(home_deg.begin(), home_deg.end(), std::back_inserter(home_rad),
                   [](double d){ return d * M_PI / 180.0; });
    auto traj = ctrl_->planJoints(home_rad);
    ctrl_->executeTrajectory(traj);
  }

  void draw_square(double side = 0.4, double z_fixed = 0.4)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()));
    // blocking PTP
    ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"), true);
    callCapture();
    double half = side / 2.0;
    for (auto [dx, dy] : std::vector<std::pair<double,double>>{{-half,-half},{-half,half},{half,half},{half,-half},{-half,-half}}) {
      auto ps = center;
      ps.pose.position.x += dx;
      ps.pose.position.y += dy;
      // blocking LIN
      ctrl_->executeTrajectory(ctrl_->planTarget(ps, "LIN"), true);
      callCapture();
    }
    home();
  }

  void draw_square_seq(double side = 0.4, double z_fixed = 0.4, double blend = 0.001)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()));
    std::vector<geometry_msgs::msg::PoseStamped> wp;
    double half = side / 2.0;
    for (auto [dx,dy] : std::vector<std::pair<double,double>>{{-half,-half},{-half,half},{half,half},{half,-half},{-half,-half}}) {
      auto ps = center;
      ps.pose.position.x += dx;
      ps.pose.position.y += dy;
      wp.push_back(ps);
    }
    auto traj = ctrl_->planSequence(wp, blend);
    ctrl_->executeTrajectory(traj, true);
    callCapture();
    home();
  }

  void draw_circle(double radius = 0.3, double z_fixed = 0.4, int divisions = 36)
  {
    home();
    const auto center = flipTarget(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()));
    ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"), true);
    callCapture();
    for (int i = 0; i <= divisions; ++i) {
      double angle = 2.0 * M_PI * i / divisions;
      auto ps = center;
      ps.pose.position.x += radius * std::cos(angle);
      ps.pose.position.y += radius * std::sin(angle);
      ctrl_->executeTrajectory(ctrl_->planTarget(ps, "LIN"), true);
      callCapture();
    }
    home();
  }

  void draw_circle_seq(double radius = 0.3, double z_fixed = 0.4, int divisions = 36, double blend = 0.001)
  {
    home();
    std::vector<geometry_msgs::msg::PoseStamped> wp;
    for (int i = 0; i <= divisions; ++i) {
      double angle = 2.0 * M_PI * i / divisions;
      auto ps = flipTarget(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()));
      ps.pose.position.x += radius * std::cos(angle);
      ps.pose.position.y += radius * std::sin(angle);
      wp.push_back(ps);
    }
    auto traj = ctrl_->planSequence(wp, blend);
    ctrl_->executeTrajectory(traj, true);
    callCapture();
    home();
  }

  void draw_line()
  {
    home();
    auto start = flipTarget(worldXY(-0.2, 0.4, 0.4, ctrl_->getRootLink()));
    viz_->publishTargetPose(start, "start");
    auto end = flipTarget(worldXY(0.2, 0.8, 0.8, ctrl_->getRootLink()));
    viz_->publishTargetPose(end, "end");
    viz_->prompt("Press 'next' in the RvizVisualToolsGui window to continue");
    ctrl_->executeTrajectory(ctrl_->planTarget(start, "PTP"), true); callCapture();
    ctrl_->executeTrajectory(ctrl_->planTarget(end, "LIN"), true); callCapture();
    home(); viz_->deleteAllMarkers();
  }

  void fibonacci_cap(double radius = 0.5, double center_x = 0.0, double center_y = 1.0, double center_z = 0.0, double cap_deg = 30.0, int n_points = 32)
  {
    home();
    double cap_rad = deg2rad(cap_deg);
    const auto center = worldXY(center_x, center_y, center_z, ctrl_->getRootLink());
    viz_->publishTargetPose(center);
    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n_points);
    if (targets.empty()) { RCLCPP_WARN(get_logger(), "fibonacci_cap: no targets"); return; }
    viz_->publishTargetPose(targets);
    viz_->prompt("Press 'next' to start cap scan");
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"), true); callCapture();
    for (size_t i = 1; i < targets.size(); ++i) {
      viz_->prompt("Press 'next' to continue to target " + std::to_string(i));
      ctrl_->executeTrajectory(ctrl_->planTarget(targets[i], "LIN"), true); callCapture();
    }
    viz_->deleteAllMarkers(); home();
  }

  void grid_sweep(double width = 0.6, double height = 0.6, double center_x = 0.0, double center_y = 0.7, double center_z = 0.0, double z_off = 0.5, int nx = 6, int ny = 6, bool row_major = false)
  {
    home();
    const auto center = worldXY(center_x, center_y, center_z, ctrl_->getRootLink());
    viz_->publishTargetPose(center);
    nx = std::max(2, nx);
    ny = std::max(2, ny);
    auto targets = sweepZigzag(center, width, height, z_off, nx, ny, row_major);
    if (targets.empty()) { RCLCPP_WARN(get_logger(), "grid_sweep: no targets"); return; }
    viz_->publishTargetPose(targets);
    viz_->prompt("Press 'next' to start grid scan");
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"), true); callCapture();
    for (size_t i = 1; i < targets.size(); ++i) {
      viz_->prompt("Press 'next' to continue to target " + std::to_string(i));
      ctrl_->executeTrajectory(ctrl_->planTarget(targets[i], "LIN"), true); callCapture();
    }
    viz_->deleteAllMarkers(); home();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<PilzMotionController>(
      "ur_arm", "world", "femto__depth_optical_frame", true);
  auto visualizer = std::make_shared<MotionVisualizer>(
      "ur_arm", "world", "femto__depth_optical_frame");
  auto demo = std::make_shared<PilzDemo>(controller, visualizer);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);
  exec.add_node(demo);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
