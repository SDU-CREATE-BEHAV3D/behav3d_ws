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

#include "behav3d_cpp/handeye_calibration.hpp"
#include "behav3d_cpp/util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <filesystem>
#include <optional>
#include <algorithm>
#include <chrono>
#include <cstdlib>

#define HE_DEBUG(node, fmt, ...) RCLCPP_DEBUG((node)->get_logger(), "[HandEyeCalibration] " fmt, ##__VA_ARGS__)
#define HE_INFO(node, fmt, ...) RCLCPP_INFO((node)->get_logger(), "[HandEyeCalibration] " fmt, ##__VA_ARGS__)
#define HE_WARN(node, fmt, ...) RCLCPP_WARN((node)->get_logger(), "[HandEyeCalibration] " fmt, ##__VA_ARGS__)
#define HE_ERROR(node, fmt, ...) RCLCPP_ERROR((node)->get_logger(), "[HandEyeCalibration] " fmt, ##__VA_ARGS__)

namespace fs = std::filesystem;

namespace behav3d::handeye
{

  // ----------------------------------------------------------------------------
  // Constructor
  // ----------------------------------------------------------------------------
  HandeyeCalibration::HandeyeCalibration(const rclcpp::NodeOptions &options)
      : rclcpp::Node("handeye_calibration_cpp", options)
  {
    output_root_ = this->declare_parameter<std::string>("handeye_output_dir", output_root_);
    session_dir_param_ = this->declare_parameter<std::string>("handeye_session_dir", "");

    visualize_ = this->declare_parameter<bool>("handeye_visualize", visualize_);
    visualize_pause_ms_ = this->declare_parameter<int>("handeye_visualize_pause_ms", visualize_pause_ms_);
    visualize_display_scale_ = this->declare_parameter<double>("handeye_visualize_display_scale", visualize_display_scale_);

    std::string method_param = this->declare_parameter<std::string>("handeye_calibration_method", calib_method_name_);
    calib_method_flag_ = methodFromString(method_param);
    calib_method_name_ = methodToString(calib_method_flag_);

    board_.squares_x = this->declare_parameter<int>("handeye_board_squares_x", board_.squares_x);
    board_.squares_y = this->declare_parameter<int>("handeye_board_squares_y", board_.squares_y);
    board_.square_len_m = this->declare_parameter<double>("handeye_square_length_m", board_.square_len_m);
    board_.marker_len_m = this->declare_parameter<double>("handeye_marker_length_m", board_.marker_len_m);
    board_.aruco_dict_id = this->declare_parameter<int>("handeye_aruco_dict_id", board_.aruco_dict_id);
    // Optional string dictionary param takes precedence if provided
    std::string dict_name = this->declare_parameter<std::string>("handeye_aruco_dict", std::string(""));
    if (!dict_name.empty())
      board_.aruco_dict_id = arucoDictFromString(dict_name);

    // IR preprocessing parameters
    ir_clip_max_ = this->declare_parameter<int>("handeye_ir_clip_max", ir_clip_max_);
    ir_invert_ = this->declare_parameter<bool>("handeye_ir_invert", ir_invert_);
    ir_use_clahe_ = this->declare_parameter<bool>("handeye_ir_use_clahe", ir_use_clahe_);
    ir_clahe_clip_ = this->declare_parameter<double>("handeye_ir_clahe_clip", ir_clahe_clip_);
    ir_clahe_tiles_ = this->declare_parameter<int>("handeye_ir_clahe_tiles", ir_clahe_tiles_);

    dict_ = cv::aruco::getPredefinedDictionary(board_.aruco_dict_id);
    board_obj_ = cv::aruco::CharucoBoard::create(board_.squares_x, board_.squares_y,
                                                 static_cast<float>(board_.square_len_m),
                                                 static_cast<float>(board_.marker_len_m),
                                                 dict_);

    HE_INFO(this, "Node ready. Board %dx%d, square=%.3f, marker=%.3f, dict=%s, method=%s",
            board_.squares_x, board_.squares_y,
            board_.square_len_m, board_.marker_len_m, arucoDictToString(board_.aruco_dict_id).c_str(),
            calib_method_name_.c_str());
  }

  // ----------------------------------------------------------------------------
  // Public pipeline entry point
  // ----------------------------------------------------------------------------
  bool HandeyeCalibration::run()
  {
    getSessionParameters();

    // defer namedWindow to visualization
    // 1) Resolve session directory
    fs::path session_dir = resolve_session_dir();
    if (session_dir.empty())
    {
      HE_ERROR(this, "No valid session directory found. Set 'session_dir' or ensure output root exists.");
      if (visualize_) cv::destroyAllWindows();
      return false;
    }
    HE_INFO(this, "Using session: %s", session_dir.string().c_str());

    // 2) Load manifest
    std::vector<CaptureItem> items;
    if (!load_manifest(session_dir, items) || items.empty())
    {
      HE_ERROR(this, "No valid captures found in manifest.json");
      if (visualize_) cv::destroyAllWindows();
      return false;
    }

    // Calibrate for color and ir cameras using a lambda
    auto calibrate_for_camera = [&](const std::string &camera_name,
                                    const std::string &child_frame){
      // 1) Load intrinsics for this camera
      cv::Mat K, D;
      if (!load_camera_calib(session_dir, camera_name, K, D))
      {
        HE_ERROR(this, "Camera intrinsics missing or invalid for %s", camera_name.c_str());
        return false;
      }

      // 2) Build inputs for OpenCV calibrateHandEye
      std::vector<cv::Mat> R_target2cam, t_target2cam;
      std::vector<cv::Mat> R_gripper2base, t_gripper2base;

      std::size_t used_local = 0;
      std::size_t dbg_idx_local = 0;
    for (const auto &it : items)
    {
      const std::string &img_path = (camera_name == "color") ? it.color_path : it.ir_path;
      HE_DEBUG(this, "[%s] Item %zu: capture_ok=%s, path='%s'", camera_name.c_str(), dbg_idx_local, it.capture_ok ? "true" : "false", img_path.c_str());
      if (!it.capture_ok || img_path.empty())
      {
        ++dbg_idx_local;
        continue;
      }

      int imread_flag = (camera_name == "ir") ? cv::IMREAD_UNCHANGED : cv::IMREAD_COLOR;
      // Existence guard before imread to avoid fallback behavior
      if (!fs::exists(img_path)) {
        HE_DEBUG(this, "[%s] Item %zu: file does not exist '%s'", camera_name.c_str(), dbg_idx_local, img_path.c_str());
        ++dbg_idx_local;
        continue;
      }
      cv::Mat img = cv::imread(img_path, imread_flag);
      if (img.empty())
      {
        HE_DEBUG(this, "[%s] Item %zu: cv::imread failed for '%s'", camera_name.c_str(), dbg_idx_local, img_path.c_str());
        ++dbg_idx_local;
        continue;
      }

      cv::Mat rvec, tvec;
      HE_DEBUG(this, "[%s] Item %zu: running Charuco detection", camera_name.c_str(), dbg_idx_local);
      if (!detect_charuco(img, K, D, rvec, tvec, visualize_))
      {
        HE_DEBUG(this, "[%s] Item %zu: Charuco detection failed", camera_name.c_str(), dbg_idx_local);
        ++dbg_idx_local;
        continue;
      }
      HE_DEBUG(this, "[%s] Item %zu: Charuco detection OK", camera_name.c_str(), dbg_idx_local);

      cv::Mat Rtc; // target->cam rotation
      cv::Rodrigues(rvec, Rtc);
      
double norm_t = cv::norm(tvec);
RCLCPP_INFO(this->get_logger(),
            "[Handeye] tvec = (%.3f, %.3f, %.3f), norm = %.3f m",
            tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
            norm_t);


      R_target2cam.push_back(Rtc);
      t_target2cam.push_back(tvec.clone());

      // base->gripper from manifest, invert to gripper->base
      const auto &pose = it.tool0_pose.pose; // base->tool0
      cv::Mat R_b2g = quat_to_R(pose);
      cv::Mat t_b2g = vec3_to_t(pose);
      cv::Mat R_g2b = R_b2g.t();
      cv::Mat t_g2b = -R_b2g.t() * t_b2g;
      R_gripper2base.push_back(R_g2b);
      t_gripper2base.push_back(t_g2b);

      ++used_local;
      ++dbg_idx_local;
    }

      if (R_target2cam.size() < 3 || R_gripper2base.size() != R_target2cam.size())
      {
        HE_ERROR(this, "[%s] Not enough valid image/pose pairs: %zu", camera_name.c_str(), R_target2cam.size());
        return false;
      }

      // 3) Calibrate
      cv::Mat R_cam2gripper, t_cam2gripper;
      if (!calibrate_handeye(R_gripper2base, t_gripper2base,
                             R_target2cam, t_target2cam,
                             R_cam2gripper, t_cam2gripper,
                             calib_method_flag_, calib_method_name_))
      {
        return false;
      }

      // Testtool0→camera, no camera→tool0
      cv::Mat R_g2c = R_cam2gripper.t();
      cv::Mat t_g2c = -R_cam2gripper.t() * t_cam2gripper;

      // 4) Write outputs with correct frames
      const std::string parent_frame = this->declare_parameter<std::string>(
          "handeye_parent_frame", std::string("tool0"));
      if (!write_outputs(session_dir, R_g2c, t_g2c, used_local,
                        parent_frame, child_frame, calib_method_name_))

      {
        return false;
      }

      HE_INFO(this, "[%s] Done. Used %zu image/pose pairs.", camera_name.c_str(), used_local);
      return true;
    };

    bool ok_color = calibrate_for_camera("color", "color_optical");
    bool ok_ir    = calibrate_for_camera("ir",    "ir_optical");

    if (visualize_) cv::destroyAllWindows();
    return ok_color && ok_ir;
  }

  // ----------------------------------------------------------------------------
  // Step helpers
  // ----------------------------------------------------------------------------
  std::string HandeyeCalibration::expand_user(const std::string &p)
  {
    if (!p.empty() && p[0] == '~')
    {
      const char *home = std::getenv("HOME");
      if (home)
        return std::string(home) + p.substr(1);
    }
    return p;
  }

  fs::path HandeyeCalibration::resolve_session_dir() const
  {
    if (!session_dir_param_.empty())
    {
      fs::path sp = expand_user(session_dir_param_);
      if (fs::exists(sp) && fs::is_directory(sp))
        return sp;
    }

    fs::path root = expand_user(output_root_);
    if (!fs::exists(root) || !fs::is_directory(root))
      return {};

    fs::path best;
    fs::file_time_type best_t = fs::file_time_type::min();
    for (auto &e : fs::directory_iterator(root))
    {
      if (!e.is_directory())
        continue;
      const auto name = e.path().filename().string();
      if (name.rfind("session-", 0) != 0)
        continue;
      std::error_code ec;
      auto t = fs::last_write_time(e, ec);
      if (!ec && (best.empty() || t > best_t))
      {
        best = e.path();
        best_t = t;
      }
    }
    return best;
  }

  void HandeyeCalibration::getSessionParameters()
  {
    // Refresh all runtime-tunable parameters from the parameter server (handeye_* only)
    session_dir_param_ = this->get_parameter("handeye_session_dir").as_string();
    output_root_ = this->get_parameter("handeye_output_dir").as_string();

    visualize_ = this->get_parameter("handeye_visualize").as_bool();
    visualize_pause_ms_ = this->get_parameter("handeye_visualize_pause_ms").as_int();
    visualize_display_scale_ = this->get_parameter("handeye_visualize_display_scale").as_double();

    board_.squares_x = this->get_parameter("handeye_board_squares_x").as_int();
    board_.squares_y = this->get_parameter("handeye_board_squares_y").as_int();
    board_.square_len_m = this->get_parameter("handeye_square_length_m").as_double();
    board_.marker_len_m = this->get_parameter("handeye_marker_length_m").as_double();
    board_.aruco_dict_id = this->get_parameter("handeye_aruco_dict_id").as_int();
    std::string dict_name = this->get_parameter("handeye_aruco_dict").as_string();
    if (!dict_name.empty())
      board_.aruco_dict_id = arucoDictFromString(dict_name);

    // Rebuild dictionary/board in case any of the above changed
    dict_ = cv::aruco::getPredefinedDictionary(board_.aruco_dict_id);
    board_obj_ = cv::aruco::CharucoBoard::create(board_.squares_x,
                                                 board_.squares_y,
                                                 static_cast<float>(board_.square_len_m),
                                                 static_cast<float>(board_.marker_len_m),
                                                 dict_);

    std::string method_param = this->get_parameter("handeye_calibration_method").as_string();
    calib_method_flag_ = methodFromString(method_param);
    calib_method_name_ = methodToString(calib_method_flag_);

    // Refresh IR preprocessing parameters
    ir_clip_max_ = this->get_parameter("handeye_ir_clip_max").as_int();
    ir_invert_ = this->get_parameter("handeye_ir_invert").as_bool();
    ir_use_clahe_ = this->get_parameter("handeye_ir_use_clahe").as_bool();
    ir_clahe_clip_ = this->get_parameter("handeye_ir_clahe_clip").as_double();
    ir_clahe_tiles_ = this->get_parameter("handeye_ir_clahe_tiles").as_int();

    HE_DEBUG(this,
             "Params updated: session_dir='%s', output_dir='%s', method=%s, dict=%s, board=%dx%d, square=%.4f, marker=%.4f, visualize=%s, pause_ms=%d, scale=%.2f",
             session_dir_param_.c_str(), output_root_.c_str(),
             calib_method_name_.c_str(), arucoDictToString(board_.aruco_dict_id).c_str(),
             board_.squares_x, board_.squares_y,
             board_.square_len_m, board_.marker_len_m,
             visualize_ ? "true" : "false", visualize_pause_ms_, visualize_display_scale_);
  }

  bool HandeyeCalibration::load_manifest(const fs::path &session_dir, std::vector<CaptureItem> &items)
  {
    const fs::path manifest = session_dir / "manifest.json";
    if (!fs::exists(manifest))
    {
      HE_ERROR(this, "manifest.json not found at %s", manifest.string().c_str());
      return false;
    }

    nlohmann::json root;
    if (!behav3d::util::readJson(manifest.string(), root))
    {
      HE_ERROR(this, "util::readJson failed for manifest");
      return false;
    }

    if (!root.contains("captures") || !root["captures"].is_array())
      return false;

    for (const auto &entry : root["captures"])
    {
      CaptureItem ci;
      try
      {
        ci.capture_ok = entry.value("capture_ok", entry.value("cap_ok", false));
        if (entry.contains("files"))
        {
          const auto &files = entry["files"];
          if (files.contains("color") && !files["color"].is_null())
            ci.color_path = files["color"].get<std::string>();
          if (files.contains("ir") && !files["ir"].is_null())
            ci.ir_path = files["ir"].get<std::string>();
        }

        // Normalize file paths to be absolute under this session directory if needed
        auto make_abs = [&](const std::string &p) -> std::string {
          if (p.empty()) return p;
          fs::path pp(p);
          if (pp.is_absolute()) return pp.string();
          return (session_dir / pp).string();
        };

        ci.color_path = make_abs(ci.color_path);
        ci.ir_path    = make_abs(ci.ir_path);

        // Validate existence to avoid accidentally picking up similarly named files elsewhere
        if (!ci.color_path.empty() && !fs::exists(ci.color_path)) {
          HE_DEBUG(this, "[manifest] Missing color file resolved to '%s' — skipping color for this capture.", ci.color_path.c_str());
          ci.color_path.clear();
        }
        if (!ci.ir_path.empty() && !fs::exists(ci.ir_path)) {
          HE_DEBUG(this, "[manifest] Missing IR file resolved to '%s' — skipping IR for this capture.", ci.ir_path.c_str());
          ci.ir_path.clear();
        }

        const auto &pt = entry.at("pose_tool0");
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = pt.value("frame", std::string(""));
        const auto &pos = pt.at("pos");
        const auto &quat = pt.at("quat");
        ps.pose.position.x = pos[0];
        ps.pose.position.y = pos[1];
        ps.pose.position.z = pos[2];
        ps.pose.orientation.x = quat[0];
        ps.pose.orientation.y = quat[1];
        ps.pose.orientation.z = quat[2];
        ps.pose.orientation.w = quat[3];
        ci.tool0_pose = ps;

        if (!ci.color_path.empty() || !ci.ir_path.empty())
          items.push_back(std::move(ci));
      }
      catch (...)
      {
        continue;
      }
    }
    return !items.empty();
  }

  bool HandeyeCalibration::load_camera_calib(const fs::path &session_dir,
                                             const std::string &camera_name,
                                             cv::Mat &K_out,
                                             cv::Mat &D_out)
  {
    const fs::path calib_dir = session_dir / "calib";
    if (!fs::exists(calib_dir))
    {
      HE_ERROR(this, "Calib dir not found: %s", calib_dir.string().c_str());
      return false;
    }

    // Preferred file names per camera
    fs::path yaml = calib_dir / (camera_name + std::string("_intrinsics.yaml"));
    if (!fs::exists(yaml))
    {
      // Fallbacks
      if (camera_name == "color" && fs::exists(calib_dir / "intrinsics.yaml"))
        yaml = calib_dir / "intrinsics.yaml";
      else
      {
        HE_ERROR(this, "No intrinsics YAML found for camera '%s' in %s", camera_name.c_str(), calib_dir.string().c_str());
        return false;
      }
    }

    cv::FileStorage fsr(yaml.string(), cv::FileStorage::READ);
    if (!fsr.isOpened())
    {
      HE_ERROR(this, "Failed to open intrinsics YAML: %s", yaml.string().c_str());
      return false;
    }

    // Try OpenCV style first: camera_matrix + distortion_coefficients as matrices
    cv::FileNode cm = fsr["camera_matrix"];
    cv::FileNode dc = fsr["distortion_coefficients"];
    if (!cm.empty() && !dc.empty())
    {
      cm >> K_out;
      dc >> D_out;
    }
    else
    {
      // Fallback to ROS CameraInfo style: sequences K (9 elems) and D (N elems)
      cv::FileNode Knode = fsr["K"];
      cv::FileNode Dnode = fsr["D"];

      if (Knode.isSeq() && (int)Knode.size() == 9)
      {
        K_out = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i)
          K_out.at<double>(i / 3, i % 3) = (double)Knode[i];
      }

      if (Dnode.isSeq() && (int)Dnode.size() >= 4)
      {
        D_out = cv::Mat(1, (int)Dnode.size(), CV_64F);
        for (int i = 0; i < (int)Dnode.size(); ++i)
          D_out.at<double>(0, i) = (double)Dnode[i];
      }
    }
    fsr.release();

    if (K_out.empty())
    {
      HE_ERROR(this, "Intrinsics YAML missing usable camera matrix for '%s'.", camera_name.c_str());
      return false;
    }
    if (D_out.empty())
    {
      // Provide a zero-distortion default if absent
      D_out = cv::Mat::zeros(1, 5, CV_64F);
      HE_WARN(this, "[%s] No distortion coefficients found; defaulting to zeros (5).", camera_name.c_str());
    }
    return true;
  }

  cv::Mat HandeyeCalibration::preprocess_ir_to_gray(const cv::Mat &img) const
  {
    // Convert to grayscale if needed
    cv::Mat gray;
    if (img.channels() == 3)
    {
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
      gray = img.clone();
    }

    // If not 8-bit, normalize to 0..255 (with optional clipping)
    if (gray.depth() != CV_8U)
    {
      if (ir_clip_max_ > 0)
      {
        cv::Mat tmp;
        cv::threshold(gray, tmp, ir_clip_max_, ir_clip_max_, cv::THRESH_TRUNC);
        gray = tmp;
      }
      cv::Mat norm8;
      cv::normalize(gray, norm8, 0, 255, cv::NORM_MINMAX, CV_8U);
      gray = norm8;
    }

    // Optional local contrast enhancement
    if (ir_use_clahe_)
    {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(ir_clahe_clip_, cv::Size(std::max(1, ir_clahe_tiles_), std::max(1, ir_clahe_tiles_)));
      cv::Mat cl;
      clahe->apply(gray, cl);
      gray = cl;
    }

    if (ir_invert_)
    {
      cv::bitwise_not(gray, gray);
    }

    // Light denoise
    cv::GaussianBlur(gray, gray, cv::Size(3,3), 0.0);
    return gray;
  }

  bool HandeyeCalibration::detect_charuco(const cv::Mat &img,
                                          const cv::Mat &K,
                                          const cv::Mat &D,
                                          cv::Mat &rvec,
                                          cv::Mat &tvec,
                                          bool visualize)
  {
    // --- Preprocess to 8-bit grayscale (handles 16-bit IR) ---
    cv::Mat gray;
    if (img.channels() == 3 && img.depth() == CV_8U)
      cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
      gray = preprocess_ir_to_gray(img);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(gray, dict_, corners, ids, params);
    if (ids.empty())
      return false;

    cv::Mat charuco_corners, charuco_ids;
    cv::aruco::interpolateCornersCharuco(corners, ids, gray, board_obj_, charuco_corners, charuco_ids);
    if (charuco_corners.empty())
      return false;

    bool pose_ok = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, board_obj_, K, D, rvec, tvec);
    if (!pose_ok)
      return false;

    // Basic sanity: board should be in front of camera (z > 0)
    if (tvec.total() >= 3 && tvec.depth() == CV_64F)
    {
      if (tvec.at<double>(2) <= 0)
        HE_WARN(this, "estimatePoseCharucoBoard yielded tvec.z <= 0 (%.4f). Check dictionary/square sizes.", tvec.at<double>(2));
    }

    if (visualize)
    {
      cv::Mat vis = gray.clone();
      cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

      // Draw raw detections
      cv::aruco::drawDetectedMarkers(vis, corners, ids);
      cv::aruco::drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids, cv::Scalar(0,255,0));

      // 1) Draw camera axes at board origin
      // Axis length in *board units* (meters). If axes look wrong, check board square_len_m!
      const float axis_len = static_cast<float>(board_.square_len_m) * 2.0f;
      cv::drawFrameAxes(vis, K, D, rvec, tvec, axis_len);

      // 2) Reproject board chessboard corners and draw outline to verify geometry
      std::vector<cv::Point3f> obj_pts = board_obj_->chessboardCorners;
      std::vector<cv::Point2f> img_pts;
      if (!obj_pts.empty())
      {
        cv::projectPoints(obj_pts, rvec, tvec, K, D, img_pts);

        // Draw outer border (four corners: (0,0), (Nx-1,0), (Nx-1,Ny-1), (0,Ny-1))
        int Nx = board_.squares_x;
        int Ny = board_.squares_y;
        auto idx = [Nx](int x, int y){ return y * Nx + x; };
        std::vector<cv::Point> border;
        border.push_back(img_pts[idx(0,0)]);
        border.push_back(img_pts[idx(Nx-1,0)]);
        border.push_back(img_pts[idx(Nx-1,Ny-1)]);
        border.push_back(img_pts[idx(0,Ny-1)]);
        const cv::Point *bp = border.data();
        int nbp = (int)border.size();
        cv::polylines(vis, &bp, &nbp, 1, true, cv::Scalar(255,0,0), 2);

        // Draw a grid of small dots for every board corner
        for (const auto &pt : img_pts)
          cv::circle(vis, pt, 2, cv::Scalar(0,255,255), -1, cv::LINE_AA);
      }

      // (Removed RMSE overlay)

      // Board spec overlay
      char spec[256];
      std::snprintf(spec, sizeof(spec), "Nx=%d Ny=%d sq=%.1fmm mk=%.1fmm dict=%s",
                    board_.squares_x, board_.squares_y,
                    board_.square_len_m*1000.0, board_.marker_len_m*1000.0,
                    arucoDictToString(board_.aruco_dict_id).c_str());
      cv::putText(vis, spec, {15,60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 3, cv::LINE_AA);
      cv::putText(vis, spec, {15,60}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 1, cv::LINE_AA);

      // Scale and show
      double s = std::clamp(visualize_display_scale_, 0.05, 4.0);
      cv::Mat vis_scaled;
      int interp = (s < 1.0) ? cv::INTER_AREA : cv::INTER_LINEAR;
      cv::resize(vis, vis_scaled, cv::Size(), s, s, interp);
      cv::imshow("charuco", vis_scaled);
      cv::waitKey(visualize_pause_ms_);
    }
    return true;
  }

  cv::Mat HandeyeCalibration::quat_to_R(const geometry_msgs::msg::Pose &p)
  {
    const double x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
    cv::Mat R = (cv::Mat_<double>(3, 3) << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
                 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
    return R;
  }

  cv::Mat HandeyeCalibration::vec3_to_t(const geometry_msgs::msg::Pose &p)
  {
    return (cv::Mat_<double>(3, 1) << p.position.x, p.position.y, p.position.z);
  }

  bool HandeyeCalibration::calibrate_handeye(const std::vector<cv::Mat> &R_gripper2base,
                                             const std::vector<cv::Mat> &t_gripper2base,
                                             const std::vector<cv::Mat> &R_target2cam,
                                             const std::vector<cv::Mat> &t_target2cam,
                                             cv::Mat &R_cam2gripper, cv::Mat &t_cam2gripper,
                                             int method_flag,
                                             const std::string &method_name)
  {
    try
    {
      cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                           R_target2cam, t_target2cam,
                           R_cam2gripper, t_cam2gripper,
                           static_cast<cv::HandEyeCalibrationMethod>(method_flag));
      RCLCPP_INFO(rclcpp::get_logger("handeye_calibration_cpp"),
                  "[HandEyeCalibration] calibration solved (method=%s).", method_name.c_str());
      return true;
    }
    catch (const cv::Exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handeye_calibration_cpp"),
                   "[HandEyeCalibration] OpenCV calibrateHandEye failed (method=%s): %s",
                   method_name.c_str(), e.what());
      return false;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("handeye_calibration_cpp"),
                   "[HandEyeCalibration] calibrateHandEye threw std::exception (method=%s): %s",
                   method_name.c_str(), e.what());
      return false;
    }
  }

  int HandeyeCalibration::methodFromString(const std::string &name)
  {
    // normalize to lower-case without spaces/underscores
    std::string s;
    s.reserve(name.size());
    for (char c : name)
      if (c != ' ' && c != '_')
        s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    if (s == "tsai")
      return cv::CALIB_HAND_EYE_TSAI;
    if (s == "park")
      return cv::CALIB_HAND_EYE_PARK;
    if (s == "horaud")
      return cv::CALIB_HAND_EYE_HORAUD;
    if (s == "daniilidis")
      return cv::CALIB_HAND_EYE_DANIILIDIS;
    if (s == "andreff")
      return cv::CALIB_HAND_EYE_ANDREFF;
    RCLCPP_WARN(rclcpp::get_logger("handeye_calibration_cpp"),
                "[HandEyeCalibration] Unknown calibration_method '%s', defaulting to 'tsai'", name.c_str());
    return cv::CALIB_HAND_EYE_TSAI;
  }

  std::string HandeyeCalibration::methodToString(int flag)
  {
    switch (flag)
    {
    case cv::CALIB_HAND_EYE_TSAI:
      return "tsai";
    case cv::CALIB_HAND_EYE_PARK:
      return "park";
    case cv::CALIB_HAND_EYE_HORAUD:
      return "horaud";
    case cv::CALIB_HAND_EYE_DANIILIDIS:
      return "daniilidis";
    case cv::CALIB_HAND_EYE_ANDREFF:
      return "andreff";
    default:
      return "tsai";
    }
  }

  bool HandeyeCalibration::write_outputs(const fs::path &session_dir,
                                         const cv::Mat &R_cam2gripper,
                                         const cv::Mat &t_cam2gripper,
                                         size_t pairs_used,
                                         const std::string &parent_frame,
                                         const std::string &child_frame,
                                         const std::string &method_name) const
  {
    const fs::path calib_dir = session_dir / "calib";
    std::error_code ec;
    fs::create_directories(calib_dir, ec);

    // Get current time ISO string
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%FT%TZ", std::gmtime(&now_c));
    std::string iso_date(buf);

    // Convert rotation matrix to quaternion and RPY
    cv::Mat R = R_cam2gripper;
    double qw, qx, qy, qz;
    {
      double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
      if (trace > 0)
      {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
        qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
        qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
      }
      else if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2))
      {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
        qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        qx = 0.25 * s;
        qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
      }
      else if (R.at<double>(1, 1) > R.at<double>(2, 2))
      {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
        qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qy = 0.25 * s;
        qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
      }
      else
      {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
        qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
        qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        qz = 0.25 * s;
      }
    }

    double roll, pitch, yaw;
    {
      roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
      pitch = std::atan2(-R.at<double>(2, 0), std::sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2)));
      yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }

    // Choose filename from child frame
    std::string base_name = std::string("handeye_") + child_frame + std::string("_to_") + parent_frame;
    for (auto &c : base_name) if (c == '/') c = '_';

    // YAML export
    const fs::path yaml = calib_dir / (base_name + std::string(".yaml"));
    {
      cv::FileStorage fsw(yaml.string(), cv::FileStorage::WRITE);
      fsw << "date" << iso_date;
      fsw << "method" << ("handeye_" + method_name);
      fsw << "pairs_used" << static_cast<int>(pairs_used);

      fsw << "transform" << "{";
      fsw << "parent" << parent_frame;
      fsw << "child" << child_frame;
      fsw << "translation" << "[:"
          << t_cam2gripper.at<double>(0)
          << t_cam2gripper.at<double>(1)
          << t_cam2gripper.at<double>(2) << "]";
      fsw << "rotation_quat_xyzw" << "[:"
          << qx << qy << qz << qw << "]";
      fsw << "rotation_rpy_rad" << "[:"
          << roll << pitch << yaw << "]";
      fsw << "}";
      fsw.release();
    }

    // JSON export for programmatic use
    nlohmann::json j;
    j["timestamp_ns"] = rclcpp::Clock().now().nanoseconds();
    j["board"] = {
        {"squares_x", board_.squares_x},
        {"squares_y", board_.squares_y},
        {"square_length_m", board_.square_len_m},
        {"marker_length_m", board_.marker_len_m},
        {"aruco_dict_id", board_.aruco_dict_id}};
    std::vector<double> Rv(9), tv(3);
    for (int r = 0, k = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        Rv[k++] = R_cam2gripper.at<double>(r, c);
    for (int r = 0; r < 3; ++r)
      tv[r] = t_cam2gripper.at<double>(r);
    j["R_cam2gripper"] = Rv;
    j["t_cam2gripper"] = tv;
    // Removed ax_xb_error block from JSON output
    j["frames"] = { {"parent", parent_frame}, {"child", child_frame} };

    const fs::path jsonp = calib_dir / (base_name + std::string(".json"));
    if (!behav3d::util::writeJson(jsonp.string(), j))
    {
      HE_WARN(this, "util::writeJson failed for %s", jsonp.string().c_str());
    }

    HE_INFO(this, "Wrote %s and %s", yaml.string().c_str(), jsonp.string().c_str());
    return true;
  }

  int HandeyeCalibration::arucoDictFromString(const std::string &name)
  {
    std::string s;
    s.reserve(name.size());
    for (char c : name) { if (c != ' ' && c != '"') s.push_back((char)std::toupper((unsigned char)c)); }
    // normalize underscores
    for (char &c : s) if (c == '-') c = '_';
    // Accept short forms like 5X5_1000 or DICT_5X5_1000
    auto ensure_prefix = [&](const std::string &t){ return (s.rfind("DICT_",0)==0)? s : std::string("DICT_")+s; };
    s = ensure_prefix(s);
    // Map known OpenCV dict names
    struct Map { const char* n; int v; } table[] = {
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_APRILTAG_16H5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25H9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36H10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36H11", cv::aruco::DICT_APRILTAG_36h11}
    };
    for (const auto &m : table) if (s == m.n) return m.v;
    RCLCPP_WARN(rclcpp::get_logger("handeye_calibration_cpp"), "Unknown aruco dict '%s', defaulting to DICT_5X5_1000", name.c_str());
    return cv::aruco::DICT_5X5_1000;
  }
  
  std::string HandeyeCalibration::arucoDictToString(int dict_id)
  {
    switch(dict_id){
      case cv::aruco::DICT_4X4_50: return "DICT_4X4_50";
      case cv::aruco::DICT_4X4_100: return "DICT_4X4_100";
      case cv::aruco::DICT_4X4_250: return "DICT_4X4_250";
      case cv::aruco::DICT_4X4_1000: return "DICT_4X4_1000";
      case cv::aruco::DICT_5X5_50: return "DICT_5X5_50";
      case cv::aruco::DICT_5X5_100: return "DICT_5X5_100";
      case cv::aruco::DICT_5X5_250: return "DICT_5X5_250";
      case cv::aruco::DICT_5X5_1000: return "DICT_5X5_1000";
      case cv::aruco::DICT_6X6_50: return "DICT_6X6_50";
      case cv::aruco::DICT_6X6_100: return "DICT_6X6_100";
      case cv::aruco::DICT_6X6_250: return "DICT_6X6_250";
      case cv::aruco::DICT_6X6_1000: return "DICT_6X6_1000";
      case cv::aruco::DICT_APRILTAG_16h5: return "DICT_APRILTAG_16H5";
      case cv::aruco::DICT_APRILTAG_25h9: return "DICT_APRILTAG_25H9";
      case cv::aruco::DICT_APRILTAG_36h10: return "DICT_APRILTAG_36H10";
      case cv::aruco::DICT_APRILTAG_36h11: return "DICT_APRILTAG_36H11";
      default: return "UNKNOWN";
    }
  }
  
} // namespace behav3d::handeye
