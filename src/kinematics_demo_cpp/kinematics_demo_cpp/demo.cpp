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
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/trajectory_builder.hpp"
#include "behav3d_cpp/util.hpp"

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
class PilzDemo : public rclcpp::Node
{
public:
  explicit PilzDemo(const std::shared_ptr<PilzMotionController> &ctrl,
                    const std::shared_ptr<MotionVisualizer> &viz)
      : Node("pilz_demo_cpp"), ctrl_(ctrl), viz_(viz)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&PilzDemo::callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "PilzDemo ready. Commands: 'home', 'draw_line', 'draw_square', "
                "'draw_square_seq', 'draw_circle', 'draw_circle_seq', 'draw_line', 'grid_sweep', 'fibonacci_cap', 'quit'");
  }

private:
  std::shared_ptr<MotionVisualizer> viz_;
  // ------------------------------------------------------------------------
  //  Command dispatcher
  // ------------------------------------------------------------------------
  void callback(const std_msgs::msg::String &msg)
  {
    const std::string cmd = msg.data;
    if (cmd == "home")
      home();
    else if (cmd == "draw_square")
      draw_square();
    else if (cmd == "draw_square_seq")
      draw_square_seq();
    else if (cmd == "draw_circle")
      draw_circle();
    else if (cmd == "draw_circle_seq")
      draw_circle_seq();
    else if (cmd == "draw_line")
      draw_line();
    else if (cmd == "grid_sweep")
      grid_sweep();
    else if (cmd == "fibonacci_cap")
      fibonacci_cap();
    else if (cmd == "quit")
      rclcpp::shutdown();
    else
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  // ------------------------------------------------------------------------
  //  Command implementations
  // ------------------------------------------------------------------------
  void home()
  {
    // Joint‑space “home” configuration (given in degrees)
    const std::vector<double> home_joints_deg = {-90.0, -120.0, 120.0, -90.0, 90.0, -180.0};

    // Convert degrees to radians for MoveIt
    std::vector<double> home_joints_rad;
    home_joints_rad.reserve(home_joints_deg.size());
    std::transform(home_joints_deg.begin(), home_joints_deg.end(),
                   std::back_inserter(home_joints_rad),
                   [](double deg)
                   { return deg * M_PI / 180.0; });

    // Plan a PTP joint motion and execute it
    auto traj = ctrl_->planJoints(home_joints_rad);
    ctrl_->executeTrajectory(traj);
  }

  void draw_square(double side = 0.4, double z_fixed = 0.4)
  {
    home();
    const double half = side / 2.0;
    {
      const double half = side / 2.0;
      const auto center = flipTargetAxes(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()), false, true);

      std::vector<std::pair<double, double>> offsets = {
          {-half, -half}, {-half, half}, {half, half}, {half, -half}, {-half, -half}};

      ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"));

      for (auto [dx, dy] : offsets)
      {
        auto ps = center;
        ps.pose.position.x += dx;
        ps.pose.position.y += dy;
        auto traj = ctrl_->planTarget(ps, "LIN");
        ctrl_->executeTrajectory(traj);
      }

      ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"));
    }
    home();
  }

  void draw_square_seq(double side = 0.4,
                       double z_fixed = 0.4,
                       double blend_radius = 0.001)
  {
    home();
    const double half = side / 2.0;
    {
      const double half = side / 2.0;
      const auto center = flipTargetAxes(worldXY(0.0, 0.7, z_fixed, ctrl_->getRootLink()), false, true);

      std::vector<std::pair<double, double>> offsets = {
          {-half, -half}, {-half, half}, {half, half}, {half, -half}, {-half, -half}};

      std::vector<geometry_msgs::msg::PoseStamped> waypoints;
      for (auto [dx, dy] : offsets)
      {
        auto ps = center;
        ps.pose.position.x += dx;
        ps.pose.position.y += dy;
        waypoints.push_back(ps);
      }

      auto traj = ctrl_->planSequence(waypoints, blend_radius);
      ctrl_->executeTrajectory(traj, true);
    }
    home();
  }

  void draw_circle(double radius = 0.3,
                   double z_fixed = 0.4,
                   int divisions = 36)
  {
    home();
    {
      const auto center = flipTargetAxes(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()), false, true);

      ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"));

      for (int i = 0; i <= divisions; ++i)
      {
        double angle = 2.0 * M_PI * i / divisions;
        double dx = radius * std::cos(angle);
        double dy = radius * std::sin(angle);

        auto ps = center;
        ps.pose.position.x += dx;
        ps.pose.position.y += dy;
        auto traj = ctrl_->planTarget(ps, "LIN");
        ctrl_->executeTrajectory(traj);
      }

      ctrl_->executeTrajectory(ctrl_->planTarget(center, "PTP"));
    }
    home();
  }

  void draw_circle_seq(double radius = 0.3,
                       double z_fixed = 0.4,
                       int divisions = 36,
                       double blend_radius = 0.001)
  {
    home();
    {
      const auto center = flipTargetAxes(worldXY(0.0, 0.8, z_fixed, ctrl_->getRootLink()), false, true);

      std::vector<geometry_msgs::msg::PoseStamped> waypoints;
      for (int i = 0; i <= divisions; ++i)
      {
        double angle = 2.0 * M_PI * i / divisions;
        double dx = radius * std::cos(angle);
        double dy = radius * std::sin(angle);

        auto ps = center;
        ps.pose.position.x += dx;
        ps.pose.position.y += dy;
        waypoints.push_back(ps);
      }

      auto traj = ctrl_->planSequence(waypoints, blend_radius);
      viz_->publishTrail(traj);
      viz_->prompt("Press 'next' to start the blended sequence");
      ctrl_->executeTrajectory(traj, true);
      viz_->deleteAllMarkers();
    }
    home();
  }

  void draw_line()
  {
    home();

    auto start = flipTargetAxes(worldXY(-0.2, 0.4, 0.4, ctrl_->getRootLink()), false, true);

    viz_->publishTargetPose(start, "start");

    auto end = flipTargetAxes(worldXY(0.2, 0.8, 0.8, ctrl_->getRootLink()), false, true);

    viz_->publishTargetPose(end, "end");

    viz_->prompt("Press 'next' in the RvizVisualToolsGui window to continue");

    ctrl_->executeTrajectory(ctrl_->planTarget(start, "PTP"));

    ctrl_->executeTrajectory(ctrl_->planTarget(end, "LIN"));

    home();
    viz_->deleteAllMarkers();
  }

  void fibonacci_cap(double radius = 0.75,
                     double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                     double cap_deg = 30.0, int n_points = 32)
  {
    // 1. Start from home
    home();

    // Convert half‑angle from degrees to radians for the builder
    const double cap_rad = deg2rad(cap_deg);

    // 3. Generate way‑points on a spherical cap using Fibonacci sampling
    const auto center = worldXY(center_x, center_y, center_z,
                                ctrl_->getRootLink());

    viz_->publishTargetPose(center);

    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n_points);

    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "fibonacci_cap: no targets generated!");
      return;
    }

    // 4. Visualise every target
    viz_->publishTargetPose(targets);

    // 5. Prompt the user before motion
    viz_->prompt("Press 'next' in the RVizVisualToolsGui window to start the cap scan");

    // 6. PTP to first point, then LIN through the rest
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"));
    for (size_t i = 1; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to continue to target " + std::to_string(i));
      auto traj = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(traj);
    }

    // 7. Clean up
    viz_->deleteAllMarkers();
    home();
  }

  void grid_sweep(double width = 1.0, double height = 0.5,
                  double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                  double z_off = 0.75,
                  int nx = 10, int ny = 5,
                  bool row_major = false)
  {
    // 1. Return to a known joint configuration
    home();

    // 2. Build center pose and generate a zig‑zag raster pattern that
    //    matches the sweepZigzag parameter space.
    const auto center = worldXY(center_x, center_y, center_z,
                                ctrl_->getRootLink());

    viz_->publishTargetPose(center);

    // Enforce a minimum of two waypoints per axis, per sweepZigzag’s contract.
    nx = std::max(2, nx);
    ny = std::max(2, ny);

    // z_off is fixed to 0 here because sweepZigzag now flips the targets internally.
    auto targets = sweepZigzag(center, width, height, z_off,
                               nx, ny, row_major);

    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "grid_sweep/sweepZigzag: no targets generated!");
      return;
    }

    // 3. Visualize all targets
    viz_->publishTargetPose(targets);

    // 4. Prompt the user before starting motion
    viz_->prompt("Press 'next' in the RVizVisualToolsGui window to start grid scan");

    // 5. Move to the first target with a PTP, then traverse the rest with LIN
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"));

    for (size_t i = 1; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to continue to target " + std::to_string(i));
      auto traj = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(traj);
    }

    // 6. Clean up markers and return home
    viz_->deleteAllMarkers();
    home();
  }
  std::shared_ptr<PilzMotionController> ctrl_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// ---------------------------------------------------------------------------
//                                   main()
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions controller_opts;
  controller_opts.use_intra_process_comms(true);
  auto controller = std::make_shared<PilzMotionController>(controller_opts);

  rclcpp::NodeOptions visualizer_opts;
  visualizer_opts.use_intra_process_comms(true);
  auto visualizer = std::make_shared<MotionVisualizer>(visualizer_opts);

  auto demo = std::make_shared<PilzDemo>(controller, visualizer);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);
  exec.add_node(demo);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}