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
// Date: 2025-08
// =============================================================================

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/camera_manager.hpp"

namespace behav3d::session_manager
{

  class SessionManager : public rclcpp::Node
  {
  public:
    struct Options
    {
      std::string session_tag;         // e.g., "fibcap_r0.75_cap30_n32"
      std::string motion_type = "LIN"; // "LIN" | "PTP"
      bool apply_totg = false;         // time-optimal parameterization on execute
      double wait_time_sec = 0.5;      // dwell before capture
    };

    SessionManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(),
                   std::shared_ptr<motion_controller::PilzMotionController> ctrl = nullptr,
                   std::shared_ptr<motion_visualizer::MotionVisualizer> viz = nullptr,
                   std::shared_ptr<behav3d::camera_manager::CameraManager> cam = nullptr,
                   std::optional<std::vector<double>> home_joints_rad = std::nullopt);

    bool initSession(const Options &opts);
    bool run(const std::vector<geometry_msgs::msg::PoseStamped> &targets);
    void finish();

  private:
    void goHome();
    void writeManifestLine(std::size_t i,
                           const geometry_msgs::msg::PoseStamped &tgt,
                           const behav3d::camera_manager::CameraManager::FilePaths &files,
                           const sensor_msgs::msg::JointState &js,
                           const geometry_msgs::msg::PoseStamped &tool0,
                           const geometry_msgs::msg::PoseStamped &eef,
                           bool plan_ok, bool exec_ok, bool cap_ok,
                           const rclcpp::Time &stamp,
                           const std::string &key);

    // dependencies
    std::shared_ptr<motion_controller::PilzMotionController> ctrl_;
    std::shared_ptr<motion_visualizer::MotionVisualizer> viz_;
    std::shared_ptr<behav3d::camera_manager::CameraManager> cam_;

    // configuration
    std::optional<std::vector<double>> home_joints_rad_;
    Options opts_{};
    std::string output_dir_;
    double calib_timeout_sec_ = 2.0;  // used when calling CameraManager::getCalibration

    // paths
    std::filesystem::path session_dir_;
    std::filesystem::path dir_color_;
    std::filesystem::path dir_depth_;
    std::filesystem::path dir_ir_;
    std::filesystem::path dir_d2c_;
    std::filesystem::path dir_c2d_;
    std::filesystem::path dir_calib_;
    std::filesystem::path manifest_path_;

    // manifest aggregation (written once at finish())
    rclcpp::Time start_stamp_;
    std::vector<std::string> manifest_entries_;
  };

} // namespace behav3d::session_manager
