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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <filesystem>

namespace behav3d::handeye
{

class HandeyeCalibration : public rclcpp::Node
{
public:
  using json = nlohmann::json;

  explicit HandeyeCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // Top-level pipeline runner.
  bool run();

private:
  // 1) Define the ChArUco specs
  struct BoardSpec
  {
    int squares_x{5};
    int squares_y{7};
    double square_len_m{0.030};
    double marker_len_m{0.022};
    int aruco_dict_id{cv::aruco::DICT_5X5_1000};
  } board_{};

  // Data for each capture from manifest
  struct CaptureItem
  {
    std::string color_path;   // RGB image path
    bool capture_ok{false};
    geometry_msgs::msg::PoseStamped tool0_pose; // base->tool0 at capture
  };

  // 2) Open the directory folders and find the needed files
  std::filesystem::path resolve_session_dir() const;
  bool load_manifest(const std::filesystem::path &session_dir, std::vector<CaptureItem> &items);

  // 3) Use the util to read the yaml and json file (camera intrinsics)
  bool load_camera_calib(const std::filesystem::path &session_dir);

  // 4) Show on the screen the sequence of images after detecting the charuco
  bool detect_charuco(const cv::Mat &img,
                      cv::Mat &rvec, cv::Mat &tvec,
                      bool visualize);

  // 5) Run the calibration function
  bool calibrate_handeye(const std::vector<cv::Mat> &R_gripper2base,
                         const std::vector<cv::Mat> &t_gripper2base,
                         const std::vector<cv::Mat> &R_target2cam,
                         const std::vector<cv::Mat> &t_target2cam,
                         cv::Mat &R_cam2gripper, cv::Mat &t_cam2gripper);

  // 6) Use the util functions to write the output
  bool write_outputs(const std::filesystem::path &session_dir,
                     const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper) const;

  // Small helpers
  static cv::Mat quat_to_R(const geometry_msgs::msg::Pose &p);
  static cv::Mat vec3_to_t(const geometry_msgs::msg::Pose &p);
  static std::string expand_user(const std::string &p);

private:
  // Config
  bool visualize_{true};
  std::string output_root_{"~/behav3d_ws/captures"};
  std::string session_dir_param_{}; // optional explicit session path

  // Cached intrinsics
  cv::Mat K_; // 3x3
  cv::Mat D_; // distortion

  // Prebuilt aruco objects
  cv::Ptr<cv::aruco::Dictionary> dict_;
  cv::Ptr<cv::aruco::CharucoBoard> board_obj_;
};

} // namespace behav3d::handeye
