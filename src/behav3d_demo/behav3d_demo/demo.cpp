// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Maintainers:
//   - Lucas Helle Pessot <luh@iti.sdu.dk>
//   - Joseph Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <cstdio>

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
#include "behav3d_cpp/session_manager.hpp"
#include "behav3d_cpp/handeye_calibration.hpp"

using behav3d::camera_manager::CameraManager;
using behav3d::motion_controller::PilzMotionController;
using behav3d::motion_visualizer::MotionVisualizer;
using behav3d::session_manager::SessionManager;

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
                       const std::shared_ptr<behav3d::camera_manager::CameraManager> &cam,
                       const std::shared_ptr<behav3d::session_manager::SessionManager> &sess,
                       const std::shared_ptr<behav3d::handeye::HandeyeCalibration> &calib)
      : Node("behav3d_demo"), ctrl_(ctrl), viz_(viz), cam_(cam), sess_(sess), calib_(calib)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/user_input", 10,
        std::bind(&Behav3dDemo::callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Behav3dDemo subscribing to: %s", "/user_input");

    capture_delay_sec_ = this->declare_parameter<double>("capture_delay_sec", 0.5);

    // Declare home_joints_deg parameter with empty default
    std::vector<double> home_joints_deg = this->declare_parameter<std::vector<double>>(
        "home_joints_deg", std::vector<double>{});
    if (home_joints_deg.empty())
    {
      home_joints_deg = {45.0, -120.0, 120.0, -90.0, 90.0, -180.0};
    }
    home_joints_rad_.reserve(home_joints_deg.size());
    std::transform(home_joints_deg.begin(), home_joints_deg.end(),
                   std::back_inserter(home_joints_rad_),
                   [](double deg)
                   { return deg2rad(deg); });

    RCLCPP_INFO(this->get_logger(),
                "Behav3dDemo ready. Commands: 'fibonacci_cap', 'grid_sweep', 'quit'. Capture delay: %.2fs", capture_delay_sec_);
  }

private:
  std::shared_ptr<PilzMotionController> ctrl_;
  std::shared_ptr<MotionVisualizer> viz_;
  std::shared_ptr<CameraManager> cam_;
  std::shared_ptr<SessionManager> sess_;
  std::shared_ptr<behav3d::handeye::HandeyeCalibration> calib_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  double capture_delay_sec_;

  std::vector<double> home_joints_rad_;

  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string &cmd = msg->data;
    if (cmd == "calib")
      fibonacci_cap();
    else if (cmd == "home")
      home();
    else if (cmd == "quit")
      rclcpp::shutdown();
    else
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  void home()
  {
    // Use home_joints_rad_ from parameter instead of hardcoded values
    auto traj = ctrl_->planJoints(home_joints_rad_);
    ctrl_->executeTrajectory(traj);
  }

  void fibonacci_cap(double radius = 0.6,
                     double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                     double cap_deg = 22.5, int n_points = 32)
  {
    const double cap_rad = deg2rad(cap_deg);
    const auto center = worldXY(center_x, center_y, center_z, ctrl_->getRootLink());
    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n_points);
    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "fibonacci_cap: no targets generated!");
      return;
    }

    behav3d::session_manager::SessionManager::Options opts;
    char tag[128];
    std::snprintf(tag, sizeof(tag), "fibcap_r%.2f_cap%d_n%d", (int)radius*100, (int)cap_deg, n_points);
    opts.session_tag = tag;
    opts.motion_type = "LIN";
    opts.apply_totg = true;
    opts.wait_time_sec = capture_delay_sec_;

    if (!sess_->initSession(opts))
      return;
    sess_->run(targets);
    sess_->finish();

    // Run Hand-Eye calibration for this session
    const std::string session_dir = sess_->getSessionDir();
    if (!session_dir.empty())
    {
      calib_->set_parameters({rclcpp::Parameter("session_dir", session_dir)});
      if (!calib_->run())
      {
        RCLCPP_WARN(this->get_logger(), "HandEyeCalibration run() reported failure for session: %s", session_dir.c_str());
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Session directory is empty; skipping HandEyeCalibration.");
    }
  }
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

    rclcpp::NodeOptions camera_opts;
    camera_opts.use_intra_process_comms(true);
    auto camera = std::make_shared<behav3d::camera_manager::CameraManager>(camera_opts);

    rclcpp::NodeOptions session_opts;
    session_opts.use_intra_process_comms(true);
    auto sess = std::make_shared<behav3d::session_manager::SessionManager>(session_opts, controller, visualizer, camera);

    rclcpp::NodeOptions calib_opts;
    calib_opts.use_intra_process_comms(true);
    auto calib = std::make_shared<behav3d::handeye::HandeyeCalibration>(calib_opts);

    auto demo = std::make_shared<Behav3dDemo>(controller, visualizer, camera, sess, calib);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(controller);
    exec.add_node(visualizer);
    exec.add_node(camera);
    exec.add_node(demo);
    exec.add_node(sess);
    exec.add_node(calib);
    exec.spin();

    rclcpp::shutdown();
    return 0;
  }
