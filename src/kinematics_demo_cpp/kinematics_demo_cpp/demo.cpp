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
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/motion_controller.hpp"

using behav3d::motion_controller::PilzMotionController;
using behav3d::motion_visualizer::MotionVisualizer;

using std::placeholders::_1;

// ---------------------------------------------------------------------------
//  Helper: build a Z‑fixed pose in the world frame, optionally “flipped”
//          (180° rotation around X, as in the Python helper).
// ---------------------------------------------------------------------------
static geometry_msgs::msg::PoseStamped
PoseStamped_WorldXY(double x,
                    double y,
                    double z,
                    const std::string & frame_id,
                    bool flipped = true)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id      = frame_id;
  ps.pose.position.x      = x;
  ps.pose.position.y      = y;
  ps.pose.position.z      = z;

  if (flipped) {
    ps.pose.orientation.x = 1.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = 0.0;
    ps.pose.orientation.w = 0.0;
  } else {
    ps.pose.orientation.w = 1.0;
  }
  return ps;
}

// ---------------------------------------------------------------------------
//                                Demo Node
// ---------------------------------------------------------------------------
class PilzDemo : public rclcpp::Node
{
public:
  explicit   PilzDemo(const std::shared_ptr<PilzMotionController>& ctrl,
           const std::shared_ptr<MotionVisualizer>& viz)
    : Node("pilz_demo_cpp"), ctrl_(ctrl), viz_(viz)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&PilzDemo::callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "PilzDemo ready. Commands: 'home', 'draw_line', 'draw_square', "
                "'draw_square_seq', 'draw_circle', 'draw_circle_seq', 'quit'");
  }

private:
  std::shared_ptr<MotionVisualizer>       viz_;   
  // ------------------------------------------------------------------------
  //  Command dispatcher
  // ------------------------------------------------------------------------
  void callback(const std_msgs::msg::String & msg)
  {
    const std::string cmd = msg.data;
    if      (cmd == "home")             home();
    else if (cmd == "draw_square")      draw_square();
    else if (cmd == "draw_square_seq")  draw_square_seq();
    else if (cmd == "draw_circle")      draw_circle();
    else if (cmd == "draw_circle_seq")  draw_circle_seq();
    else if (cmd == "draw_line")        draw_line();
    else if (cmd == "quit")             rclcpp::shutdown();
    else
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  // ------------------------------------------------------------------------
  //  Command implementations
  // ------------------------------------------------------------------------
  void home()
  {
    // Joint‑space “home” configuration (given in degrees)
    const std::vector<double> home_joints_deg = { 90.0, -120.0, 120.0, -90.0, 90.0, -180.0 };

    // Convert degrees to radians for MoveIt
    std::vector<double> home_joints_rad;
    home_joints_rad.reserve(home_joints_deg.size());
    std::transform(home_joints_deg.begin(), home_joints_deg.end(),
                   std::back_inserter(home_joints_rad),
                   [](double deg) { return deg * M_PI / 180.0; });

    // Plan a PTP joint motion and execute it
    auto traj = ctrl_->planJoints(home_joints_rad);
    ctrl_->executeTrajectory(traj);
  }

  void draw_square(double side = 0.4, double z_fixed = 0.4)
  {
    home();
    const double half   = side / 2.0;
    {
      const double half   = side / 2.0;
      const auto   center = PoseStamped_WorldXY(0.0, 0.7, z_fixed,
                                                ctrl_->getRootLink(), true);

      std::vector<std::pair<double,double>> offsets = {
        {-half, -half}, {-half,  half}, { half,  half},
        { half, -half}, {-half, -half}
      };

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
    const double half   = side / 2.0;
    {
      const double half   = side / 2.0;
      const auto   center = PoseStamped_WorldXY(0.0, 0.7, z_fixed,
                                                ctrl_->getRootLink(), true);

      std::vector<std::pair<double,double>> offsets = {
        {-half, -half}, {-half,  half}, { half,  half},
        { half, -half}, {-half, -half}
      };

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
                   int    divisions = 36)
  {
    home();
    {
      const auto center = PoseStamped_WorldXY(0.0, 0.8, z_fixed,
                                              ctrl_->getRootLink(), true);

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
                       int    divisions = 36,
                       double blend_radius = 0.001)
  {
    home();
    {
      const auto center = PoseStamped_WorldXY(0.0, 0.8, z_fixed,
                                              ctrl_->getRootLink(), true);

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
      ctrl_->executeTrajectory(traj, true);
    }
    home();
  }

  void draw_line()
  {
    home();

    auto start = PoseStamped_WorldXY(-0.2, 0.4, 0.4,
                                     ctrl_->getRootLink(), true); 

    viz_->publishTargetPose(start, "start");        

    auto end = PoseStamped_WorldXY(0.2, 0.8, 0.8,
                                   ctrl_->getRootLink(), true);

    viz_->publishTargetPose(end, "end");

    ctrl_->executeTrajectory(ctrl_->planTarget(start, "PTP"));
    
    ctrl_->executeTrajectory(ctrl_->planTarget(end,   "LIN"));

    home();
    viz_->deleteAllMarkers();
  }

  // ------------------------------------------------------------------------
  //  Members
  // ------------------------------------------------------------------------
  std::shared_ptr<PilzMotionController>                               ctrl_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             sub_;
};

// ---------------------------------------------------------------------------
//                                   main()
// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Instantiate the motion controller: replace "manipulator" and "world" 
  // with your MoveIt group name and root link frame as needed.
  auto controller = std::make_shared<PilzMotionController>(
      "ur_arm",                       // MoveIt planning group name
      "ur10e_base_link",              // Root link frame
      "femto__depth_optical_frame",   // End-effector link frame
      true                            // Debug mode off
  );
  auto visualizer = std::make_shared<MotionVisualizer>(
      "ur_arm",
      "ur10e_base_link",
      "femto__depth_optical_frame"
  );
  auto demo = std::make_shared<PilzDemo>(controller, visualizer);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);  
  exec.add_node(demo);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}