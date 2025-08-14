#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/core.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <string>
#include <vector>
#include <filesystem>

#include <nlohmann/json.hpp>

namespace behav3d::handeye
{

class HandeyeCalibration : public rclcpp::Node
{
public:
  explicit HandeyeCalibration(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  // Full pipeline entry-point. Returns true on success, false otherwise.
  bool run();

private:
  struct BoardSpec
  {
    int   squares_x    = 5;
    int   squares_y    = 7;
    double square_len_m = 0.030;  // meters
    double marker_len_m = 0.022;  // meters
    int   aruco_dict_id = cv::aruco::DICT_5X5_1000;
  } board_;

  struct CaptureItem
  {
    bool capture_ok = false;
    std::string color_path;                   // absolute or session-relative
    geometry_msgs::msg::PoseStamped tool0_pose; // base->tool0 pose
  };

  // Parameters
  std::string output_root_      = "~/behav3d_ws/captures"; // where session-* live
  std::string session_dir_param_ = "";                     // explicit session dir
  bool visualize_               = true;

  // Hand-eye method selection
  int calib_method_flag_ = cv::CALIB_HAND_EYE_TSAI; // OpenCV flag
  std::string calib_method_name_ = "tsai";         // normalized lower-case name

  // OpenCV board/dictionary
  cv::Ptr<cv::aruco::Dictionary>  dict_;
  cv::Ptr<cv::aruco::CharucoBoard> board_obj_;

  // Camera intrinsics
  cv::Mat K_;  // 3x3
  cv::Mat D_;  // distortion coefficients

  // Helpers (steps of the pipeline)
  static std::string expand_user(const std::string &p);
  std::filesystem::path resolve_session_dir() const;
  bool load_manifest(const std::filesystem::path &session_dir, std::vector<CaptureItem> &items);
  bool load_camera_calib(const std::filesystem::path &session_dir);
  bool detect_charuco(const cv::Mat &img, cv::Mat &rvec, cv::Mat &tvec, bool visualize);

  static cv::Mat quat_to_R(const geometry_msgs::msg::Pose &p);
  static cv::Mat vec3_to_t(const geometry_msgs::msg::Pose &p);

  static bool calibrate_handeye(const std::vector<cv::Mat> &R_gripper2base,
                                const std::vector<cv::Mat> &t_gripper2base,
                                const std::vector<cv::Mat> &R_target2cam,
                                const std::vector<cv::Mat> &t_target2cam,
                                cv::Mat &R_cam2gripper,
                                cv::Mat &t_cam2gripper,
                                int method_flag,
                                const std::string &method_name);

  static int methodFromString(const std::string &name);
  static std::string methodToString(int flag);

  bool write_outputs(const std::filesystem::path &session_dir,
                     const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper,
                     size_t pairs_used) const;
};

} // namespace behav3d::handeye
