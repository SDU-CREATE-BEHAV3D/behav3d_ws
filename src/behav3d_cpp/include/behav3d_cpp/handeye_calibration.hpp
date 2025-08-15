// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
// Maintainers:
//   - Lucas José Helle <luh@iti.sdu.dk>
//   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-08
// =====================================================

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
    void getSessionParameters();
    bool run();

  private:
    struct BoardSpec
    {
      int squares_x = 5;
      int squares_y = 7;
      double square_len_m = 0.030; // meters
      double marker_len_m = 0.022; // meters
      int aruco_dict_id = cv::aruco::DICT_5X5_1000;
    } board_;

    struct CaptureItem
    {
      bool capture_ok = false;
      std::string color_path;                     // absolute or session-relative
      std::string ir_path;                        // absolute or session-relative
      geometry_msgs::msg::PoseStamped tool0_pose; // base->tool0 pose
    };


    // Parameters
    std::string output_root_ = "~/behav3d_ws/captures"; // where session-* live
    std::string session_dir_param_ = "";                // explicit session dir
    bool visualize_ = true;
    int visualize_pause_ms_ = 2000; // pause between visualization frames
    double visualize_display_scale_ = 0.5; // uniform scale for on-screen visualization

    // Hand-eye method selection
    int calib_method_flag_ = cv::CALIB_HAND_EYE_TSAI; // OpenCV flag
    std::string calib_method_name_ = "tsai";          // normalized lower-case name

    // OpenCV board/dictionary
    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Ptr<cv::aruco::CharucoBoard> board_obj_;

    // IR preprocessing parameters
    int ir_clip_max_ = -1;          // if >0, clamp 16-bit IR to this before normalization
    bool ir_invert_ = false;        // optionally invert IR image after normalization
    bool ir_use_clahe_ = true;      // apply CLAHE to boost contrast
    double ir_clahe_clip_ = 3.0;    // CLAHE clip limit
    int ir_clahe_tiles_ = 8;        // CLAHE tile grid size (NxN)


    // Helpers (steps of the pipeline)
    static std::string expand_user(const std::string &p);
    std::filesystem::path resolve_session_dir() const;
    bool load_manifest(const std::filesystem::path &session_dir, std::vector<CaptureItem> &items);
    bool load_camera_calib(const std::filesystem::path &session_dir,
                           const std::string &camera_name,
                           cv::Mat &K_out,
                           cv::Mat &D_out);
    bool detect_charuco(const cv::Mat &img,
                        const cv::Mat &K,
                        const cv::Mat &D,
                        cv::Mat &rvec,
                        cv::Mat &tvec,
                        bool visualize);

    static cv::Mat quat_to_R(const geometry_msgs::msg::Pose &p);
    static cv::Mat vec3_to_t(const geometry_msgs::msg::Pose &p);

    // IR preprocessing helper
    cv::Mat preprocess_ir_to_gray(const cv::Mat &img) const;

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
    static int arucoDictFromString(const std::string &name);
    static std::string arucoDictToString(int dict_id);

    bool write_outputs(const std::filesystem::path &session_dir,
                       const cv::Mat &R_cam2gripper,
                       const cv::Mat &t_cam2gripper,
                       size_t pairs_used,
                       const std::string &parent_frame,
                       const std::string &child_frame,
                       const std::string &method_name) const;
  };

} // namespace behav3d::handeye
