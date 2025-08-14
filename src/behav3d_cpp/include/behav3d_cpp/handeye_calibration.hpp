#include "behav3d_cpp/handeye_calibration.hpp"

#include <opencv2/aruco.hpp>
#include <yaml-cpp/yaml.h>

namespace behav3d::handeye
{

bool HandeyeCalibration::load_camera_calib(const std::filesystem::path &session_dir)
{
  const std::filesystem::path calib_dir = session_dir / "calib";
  if (!std::filesystem::exists(calib_dir)) return false;

  // Candidate filenames in priority order
  const std::vector<std::string> candidates = {
    "color_intrinsics.yaml",   // OpenCV format (preferred)
    "intrinsics.yaml",         // OpenCV format (fallback)
    "color_camera_info.yaml",  // ROS CameraInfo format
    "camera_info.yaml",        // ROS CameraInfo format (generic)
    "ir_camera_info.yaml"      // ROS CameraInfo format (if color not present)
  };

  auto try_read_opencv = [&](const std::filesystem::path &p) -> bool {
    cv::FileStorage fsr(p.string(), cv::FileStorage::READ);
    if (!fsr.isOpened()) return false;
    cv::Mat K, D;
    try {
      fsr["camera_matrix"] >> K;
      fsr["distortion_coefficients"] >> D;
      fsr.release();
      if (K.empty() || K.rows != 3 || K.cols != 3) return false;
      // D can be 1xN or Nx1; accept any length >= 4
      if (D.empty() || (D.total() < 4)) return false;
      K_ = K; D_ = D.reshape(1, (int)D.total()); // make it Nx1
      return true;
    } catch (...) {
      return false;
    }
  };

  auto try_read_camerainfo = [&](const std::filesystem::path &p) -> bool {
    try {
      YAML::Node root = YAML::LoadFile(p.string());
      // Typical ROS camera_info layout
      YAML::Node km = root["camera_matrix"];     // has rows, cols, data (size 9)
      YAML::Node dm = root["distortion_coefficients"]; // has rows, cols, data (size 4/5/8...)

      // Some dumps might use short keys K / D
      if (!km) km = root["K"]; // if present as a flat array size 9
      if (!dm) dm = root["D"]; // flat array

      std::vector<double> kdata;
      if (km) {
        if (km["data"]) {
          kdata = km["data"].as<std::vector<double>>();
        } else if (km.IsSequence()) {
          kdata = km.as<std::vector<double>>();
        }
      }
      if (kdata.size() != 9) return false;

      std::vector<double> ddata;
      if (dm) {
        if (dm["data"]) {
          ddata = dm["data"].as<std::vector<double>>();
        } else if (dm.IsSequence()) {
          ddata = dm.as<std::vector<double>>();
        }
      }
      if (ddata.size() < 4) return false;

      // Fill cv::Mat
      K_ = (cv::Mat_<double>(3,3) <<
        kdata[0], kdata[1], kdata[2],
        kdata[3], kdata[4], kdata[5],
        kdata[6], kdata[7], kdata[8]);

      D_ = cv::Mat((int)ddata.size(), 1, CV_64F);
      for (int i = 0; i < (int)ddata.size(); ++i) D_.at<double>(i,0) = ddata[i];

      return true;
    } catch (...) {
      return false;
    }
  };

  for (const auto &name : candidates) {
    const std::filesystem::path p = calib_dir / name;
    if (!std::filesystem::exists(p)) continue;
    if (try_read_opencv(p) || try_read_camerainfo(p)) {
      RCLCPP_INFO(get_logger(), "[HandEye] Loaded intrinsics from %s", p.string().c_str());
      return true;
    }
  }

  RCLCPP_ERROR(get_logger(), "[HandEye] No usable intrinsics found under %s", calib_dir.string().c_str());
  return false;
}

} // namespace behav3d::handeye
