#include "behav3d_cpp/handeye_calibration.hpp"
#include "behav3d_cpp/util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

#include <filesystem>
#include <optional>
#include <algorithm>
#include <chrono>
#include <cstdlib>

namespace fs = std::filesystem;

namespace behav3d::handeye
{

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
HandeyeCalibration::HandeyeCalibration(const rclcpp::NodeOptions &options)
  : rclcpp::Node("handeye_calibration_cpp", options)
{
  // Parameters (do NOT change any other packages)
  output_root_        = this->declare_parameter<std::string>("output_dir", output_root_);
  session_dir_param_  = this->declare_parameter<std::string>("session_dir", "");
  visualize_          = this->declare_parameter<bool>("visualize", true);

  // Hand-eye method ("tsai", "park", "horaud", "daniilidis", "andreff")
  std::string method_param = this->declare_parameter<std::string>("calibration_method", calib_method_name_);
  calib_method_flag_ = methodFromString(method_param);
  calib_method_name_ = methodToString(calib_method_flag_);

  board_.squares_x    = this->declare_parameter<int>("board_squares_x", board_.squares_x);
  board_.squares_y    = this->declare_parameter<int>("board_squares_y", board_.squares_y);
  board_.square_len_m = this->declare_parameter<double>("square_length_m", board_.square_len_m);
  board_.marker_len_m = this->declare_parameter<double>("marker_length_m", board_.marker_len_m);
  board_.aruco_dict_id= this->declare_parameter<int>("aruco_dict_id", board_.aruco_dict_id);

  dict_ = cv::aruco::getPredefinedDictionary(board_.aruco_dict_id);
  board_obj_ = cv::aruco::CharucoBoard::create(board_.squares_x, board_.squares_y,
                                               static_cast<float>(board_.square_len_m),
                                               static_cast<float>(board_.marker_len_m),
                                               dict_);

  RCLCPP_INFO(get_logger(), "[HandEye] Node ready. Board %dx%d, square=%.3f, marker=%.3f, dict=%d, method=%s",
              board_.squares_x, board_.squares_y,
              board_.square_len_m, board_.marker_len_m, board_.aruco_dict_id,
              calib_method_name_.c_str());
}

// ----------------------------------------------------------------------------
// Pipeline
// ----------------------------------------------------------------------------
bool HandeyeCalibration::run()
{
  // 2) Resolve session directory
  const fs::path session_dir = resolve_session_dir();
  if (session_dir.empty()) {
    RCLCPP_ERROR(get_logger(), "[HandEye] No session directory found.");
    return false;
  }
  RCLCPP_INFO(get_logger(), "[HandEye] Using session: %s", session_dir.string().c_str());

  // Load manifest
  std::vector<CaptureItem> items;
  if (!load_manifest(session_dir, items) || items.empty()) {
    RCLCPP_ERROR(get_logger(), "[HandEye] No valid captures found in manifest.json");
    return false;
  }

  // 3) Intrinsics
  if (!load_camera_calib(session_dir)) {
    RCLCPP_ERROR(get_logger(), "[HandEye] Camera intrinsics missing or invalid");
    return false;
  }

  // Prepare containers for OpenCV calibrateHandEye
  std::vector<cv::Mat> R_target2cam, t_target2cam;
  std::vector<cv::Mat> R_gripper2base, t_gripper2base;

  std::size_t used = 0;
  for (const auto &it : items) {
    if (!it.capture_ok || it.color_path.empty()) continue;

    cv::Mat img = cv::imread(it.color_path, cv::IMREAD_COLOR);
    if (img.empty()) continue;

    cv::Mat rvec, tvec;
    if (!detect_charuco(img, rvec, tvec, visualize_)) continue;

    cv::Mat Rtc; cv::Rodrigues(rvec, Rtc);
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
    used++;
  }

  if (R_target2cam.size() < 3 || R_gripper2base.size() != R_target2cam.size()) {
    RCLCPP_ERROR(get_logger(), "[HandEye] Not enough valid image/pose pairs: %zu", R_target2cam.size());
    return false;
  }

  // 5) Calibrate
  cv::Mat R_cam2gripper, t_cam2gripper;
  if (!calibrate_handeye(R_gripper2base, t_gripper2base,
                         R_target2cam, t_target2cam,
                         R_cam2gripper, t_cam2gripper,
                         calib_method_flag_, calib_method_name_)) {
    return false;
  }

  // 6) Write outputs
  if (!write_outputs(session_dir, R_cam2gripper, t_cam2gripper, used)) return false;

  RCLCPP_INFO(get_logger(), "[HandEye] Done. Used %zu image/pose pairs.", used);
  return true;
}

// ----------------------------------------------------------------------------
// Step helpers
// ----------------------------------------------------------------------------
std::string HandeyeCalibration::expand_user(const std::string &p)
{
  if (!p.empty() && p[0] == '~') {
    const char *home = std::getenv("HOME");
    if (home) return std::string(home) + p.substr(1);
  }
  return p;
}

fs::path HandeyeCalibration::resolve_session_dir() const
{
  if (!session_dir_param_.empty()) {
    fs::path sp = expand_user(session_dir_param_);
    if (fs::exists(sp) && fs::is_directory(sp)) return sp;
  }

  fs::path root = expand_user(output_root_);
  if (!fs::exists(root) || !fs::is_directory(root)) return {};

  fs::path best; fs::file_time_type best_t = fs::file_time_type::min();
  for (auto &e : fs::directory_iterator(root)) {
    if (!e.is_directory()) continue;
    const auto name = e.path().filename().string();
    if (name.rfind("session-", 0) != 0) continue;
    std::error_code ec; auto t = fs::last_write_time(e, ec);
    if (!ec && (best.empty() || t > best_t)) { best = e.path(); best_t = t; }
  }
  return best;
}

bool HandeyeCalibration::load_manifest(const fs::path &session_dir, std::vector<CaptureItem> &items)
{
  const fs::path manifest = session_dir / "manifest.json";
  if (!fs::exists(manifest)) {
    RCLCPP_ERROR(get_logger(), "[HandEye] manifest.json not found at %s", manifest.string().c_str());
    return false;
  }

  nlohmann::json root;
  if (!behav3d::util::readJson(manifest.string(), root)) {
    RCLCPP_ERROR(get_logger(), "[HandEye] util::readJson failed for manifest");
    return false;
  }

  if (!root.contains("captures") || !root["captures"].is_array()) return false;

  for (const auto &entry : root["captures"]) {
    CaptureItem ci;
    try {
      ci.capture_ok = entry.value("capture_ok", entry.value("cap_ok", false));
      if (entry.contains("files")) {
        const auto &files = entry["files"];
        if (files.contains("color") && !files["color"].is_null())
          ci.color_path = files["color"].get<std::string>();
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

      if (!ci.color_path.empty()) items.push_back(std::move(ci));
    } catch (...) {
      continue;
    }
  }
  return !items.empty();
}

bool HandeyeCalibration::load_camera_calib(const fs::path &session_dir)
{
  const fs::path calib_dir = session_dir / "calib";
  if (!fs::exists(calib_dir)) return false;

  fs::path yaml = calib_dir / "color_intrinsics.yaml";
  if (!fs::exists(yaml)) {
    if (fs::exists(calib_dir / "intrinsics.yaml")) yaml = calib_dir / "intrinsics.yaml";
  }
  if (!fs::exists(yaml)) return false;

  cv::FileStorage fsr(yaml.string(), cv::FileStorage::READ);
  if (!fsr.isOpened()) return false;
  fsr["camera_matrix"] >> K_;
  fsr["distortion_coefficients"] >> D_;
  fsr.release();

  return !K_.empty();
}

bool HandeyeCalibration::detect_charuco(const cv::Mat &img,
                                        cv::Mat &rvec, cv::Mat &tvec,
                                        bool visualize)
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(img, dict_, corners, ids, params);
  if (ids.empty()) return false;

  cv::Mat charuco_corners, charuco_ids;
  cv::aruco::interpolateCornersCharuco(corners, ids, img, board_obj_, charuco_corners, charuco_ids);
  if (charuco_corners.empty()) return false;

  bool pose_ok = cv::aruco::estimatePoseCharucoBoard(charuco_corners, charuco_ids, board_obj_, K_, D_, rvec, tvec);
  if (!pose_ok) return false;

  if (visualize) {
    cv::Mat vis = img.clone();
    cv::aruco::drawDetectedMarkers(vis, corners, ids);
    cv::drawFrameAxes(vis, K_, D_, rvec, tvec, static_cast<float>(board_.square_len_m) * 2.0f);
    cv::imshow("charuco", vis);
    cv::waitKey(2000);
  }
  return true;
}

cv::Mat HandeyeCalibration::quat_to_R(const geometry_msgs::msg::Pose &p)
{
  const double x=p.orientation.x, y=p.orientation.y, z=p.orientation.z, w=p.orientation.w;
  cv::Mat R = (cv::Mat_<double>(3,3) <<
    1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w),
    2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w),
    2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y));
  return R;
}

cv::Mat HandeyeCalibration::vec3_to_t(const geometry_msgs::msg::Pose &p)
{
  return (cv::Mat_<double>(3,1) << p.position.x, p.position.y, p.position.z);
}

bool HandeyeCalibration::calibrate_handeye(const std::vector<cv::Mat> &R_gripper2base,
                                           const std::vector<cv::Mat> &t_gripper2base,
                                           const std::vector<cv::Mat> &R_target2cam,
                                           const std::vector<cv::Mat> &t_target2cam,
                                           cv::Mat &R_cam2gripper, cv::Mat &t_cam2gripper,
                                           int method_flag,
                                           const std::string &method_name)
{
  try {
    cv::calibrateHandEye(R_gripper2base, t_gripper2base,
                         R_target2cam, t_target2cam,
                         R_cam2gripper, t_cam2gripper,
                         static_cast<cv::HandEyeCalibrationMethod>(method_flag));
    RCLCPP_INFO(rclcpp::get_logger("handeye_calibration_cpp"), "[HandEye] calibration solved (method=%s).", method_name.c_str());
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("handeye_calibration_cpp"), "[HandEye] calibrateHandEye failed: %s", e.what());
    return false;
  }
}

bool HandeyeCalibration::write_outputs(const fs::path &session_dir,
                                       const cv::Mat &R_cam2gripper, const cv::Mat &t_cam2gripper,
                                       size_t pairs_used) const
{
  const fs::path calib_dir = session_dir / "calib";
  std::error_code ec; fs::create_directories(calib_dir, ec);

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
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    if (trace > 0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (R.at<double>(2,1) - R.at<double>(1,2)) * s;
      qy = (R.at<double>(0,2) - R.at<double>(2,0)) * s;
      qz = (R.at<double>(1,0) - R.at<double>(0,1)) * s;
    } else {
      if (R.at<double>(0,0) > R.at<double>(1,1) && R.at<double>(0,0) > R.at<double>(2,2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(0,0) - R.at<double>(1,1) - R.at<double>(2,2));
        qw = (R.at<double>(2,1) - R.at<double>(1,2)) / s;
        qx = 0.25 * s;
        qy = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        qz = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
      } else if (R.at<double>(1,1) > R.at<double>(2,2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(1,1) - R.at<double>(0,0) - R.at<double>(2,2));
        qw = (R.at<double>(0,2) - R.at<double>(2,0)) / s;
        qx = (R.at<double>(0,1) + R.at<double>(1,0)) / s;
        qy = 0.25 * s;
        qz = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
      } else {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(2,2) - R.at<double>(0,0) - R.at<double>(1,1));
        qw = (R.at<double>(1,0) - R.at<double>(0,1)) / s;
        qx = (R.at<double>(0,2) + R.at<double>(2,0)) / s;
        qy = (R.at<double>(1,2) + R.at<double>(2,1)) / s;
        qz = 0.25 * s;
      }
    }
  }

  // Convert rotation matrix to roll, pitch, yaw
  double roll, pitch, yaw;
  {
    roll = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
    pitch = std::atan2(-R.at<double>(2,0), std::sqrt(R.at<double>(2,1)*R.at<double>(2,1) + R.at<double>(2,2)*R.at<double>(2,2)));
    yaw = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
  }

  // YAML export with new format
  const fs::path yaml = calib_dir / "handeye_cam_to_gripper.yaml";
  {
    cv::FileStorage fsw(yaml.string(), cv::FileStorage::WRITE);
    fsw << "date" << iso_date;
    fsw << "method" << ("handeye_" + calib_method_name_);
    fsw << "pairs_used" << static_cast<int>(pairs_used);
    fsw << "reprojection_rms_px" << 0.0;

    fsw << "color_optical_calibrated" << "{";
    fsw << "parent" << "gripper";
    fsw << "child" << "color_optical";
    fsw << "translation" << "[:" << t_cam2gripper.at<double>(0) << t_cam2gripper.at<double>(1) << t_cam2gripper.at<double>(2) << "]";
    fsw << "rotation_quat" << "[:" << qx << qy << qz << qw << "]";
    fsw << "rotation_rpy" << "[:" << roll << pitch << yaw << "]";
    fsw << "}";
    fsw.release();
  }

  // JSON via util
  nlohmann::json j;
  j["timestamp_ns"] = rclcpp::Clock().now().nanoseconds();
  j["board"] = {
    {"squares_x", board_.squares_x},
    {"squares_y", board_.squares_y},
    {"square_length_m", board_.square_len_m},
    {"marker_length_m", board_.marker_len_m},
    {"aruco_dict_id", board_.aruco_dict_id}
  };
  std::vector<double> Rv(9), tv(3);
  for (int r=0,k=0; r<3; ++r) for (int c=0; c<3; ++c) Rv[k++] = R_cam2gripper.at<double>(r,c);
  for (int r=0; r<3; ++r) tv[r] = t_cam2gripper.at<double>(r);
  j["R_cam2gripper"] = Rv;
  j["t_cam2gripper"] = tv;

  const fs::path jsonp = calib_dir / "handeye_results.json";
  if (!behav3d::util::writeJson(jsonp.string(), j)) {
    RCLCPP_WARN(rclcpp::get_logger("handeye_calibration_cpp"), "[HandEye] util::writeJson failed for %s", jsonp.string().c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("handeye_calibration_cpp"), "[HandEye] Wrote %s and %s",
              yaml.string().c_str(), jsonp.string().c_str());
  return true;
}

int HandeyeCalibration::methodFromString(const std::string &name)
{
  // normalize to lower-case without spaces/underscores
  std::string s; s.reserve(name.size());
  for (char c : name) if (c != ' ' && c != '_') s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  if (s == "tsai") return cv::CALIB_HAND_EYE_TSAI;
  if (s == "park") return cv::CALIB_HAND_EYE_PARK;
  if (s == "horaud") return cv::CALIB_HAND_EYE_HORAUD;
  if (s == "daniilidis") return cv::CALIB_HAND_EYE_DANIILIDIS;
  if (s == "andreff") return cv::CALIB_HAND_EYE_ANDREFF;
  RCLCPP_WARN(rclcpp::get_logger("handeye_calibration_cpp"), "[HandEye] Unknown calibration_method '%s', defaulting to 'tsai'", name.c_str());
  return cv::CALIB_HAND_EYE_TSAI;
}

std::string HandeyeCalibration::methodToString(int flag)
{
  switch (flag) {
    case cv::CALIB_HAND_EYE_TSAI: return "tsai";
    case cv::CALIB_HAND_EYE_PARK: return "park";
    case cv::CALIB_HAND_EYE_HORAUD: return "horaud";
    case cv::CALIB_HAND_EYE_DANIILIDIS: return "daniilidis";
    case cv::CALIB_HAND_EYE_ANDREFF: return "andreff";
    default: return "tsai";
  }
}

} // namespace behav3d::handeye
