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

#include "behav3d_cpp/session_manager.hpp"
#include "behav3d_cpp/handeye_calibration.hpp"
#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/camera_manager.hpp"
#include "behav3d_cpp/util.hpp"

#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <algorithm>
#include <cmath>
#include <filesystem>

#define SESS_INFO(node, fmt, ...) RCLCPP_INFO((node)->get_logger(), "[SessionManager] " fmt, ##__VA_ARGS__)

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstdio>

namespace fs = std::filesystem;

namespace behav3d::session_manager
{

  // Robot link naming: driven by the `robot_prefix` parameter declared in the constructor.
  // This makes the "*_tool0" link configurable from CLI/launch.
  static std::string g_robot_prefix = "ur10e";
  static inline std::string tool0Link() { return g_robot_prefix + "_tool0"; }

  // ─────────────────────────────────────────────────────────────────────────────
  // SessionManager
  // ─────────────────────────────────────────────────────────────────────────────
  SessionManager::SessionManager(const rclcpp::NodeOptions &options,
                                 std::shared_ptr<motion_controller::PilzMotionController> ctrl,
                                 std::shared_ptr<motion_visualizer::MotionVisualizer> viz,
                                 std::shared_ptr<behav3d::camera_manager::CameraManager> cam,
                                 std::optional<std::vector<double>> home_joints_rad)
      : rclcpp::Node("session_manager_cpp", options),
        ctrl_(std::move(ctrl)), viz_(std::move(viz)), cam_(std::move(cam)),
        home_joints_rad_(std::move(home_joints_rad))
  {
    RCLCPP_INFO(this->get_logger(), "[SessionManager] initialized");
    // Declare robot_prefix parameter once (default: "ur10e") and store it
    g_robot_prefix = this->declare_parameter<std::string>("robot_prefix", "ur10e");
    RCLCPP_INFO(this->get_logger(), "[SessionManager] robot_prefix: %s", g_robot_prefix.c_str());
    // Where to create session directories (keep same default as CameraManager)
    output_dir_ = this->declare_parameter<std::string>("output_dir", "~/behav3d_ws/captures");

    // Declare home_joints_deg parameter with empty vector default
    std::vector<double> home_joints_deg = this->declare_parameter<std::vector<double>>(
        "home_joints_deg", std::vector<double>{});

    // If home_joints_rad_ not provided, convert home_joints_deg to radians or fallback to default
    if (!home_joints_rad_)
    {
      if (home_joints_deg.empty())
      {
        home_joints_deg = {0.0, -120.0, 120.0, -90.0, 90.0, -180.0};
      }
      home_joints_rad_.emplace();
      home_joints_rad_->reserve(home_joints_deg.size());
      for (double deg : home_joints_deg)
        home_joints_rad_->push_back(deg * (3.14159265358979323846 / 180.0));
    }
  }

  bool SessionManager::initSession(const Options &opts)
  {
    opts_ = opts;

    // Build session directory name
    auto now = this->now();
    start_stamp_ = now;
    const std::string ts = behav3d::util::timeStringDateTime(now);
    const std::string tag = opts_.session_tag.empty() ? std::string("untitled") : opts_.session_tag;

    // Expand ~ if present
    std::string root = output_dir_;
    if (!root.empty() && root[0] == '~')
    {
      const char *home = std::getenv("HOME");
      if (home)
        root = std::string(home) + root.substr(1);
    }
    std::error_code ec;
    fs::create_directories(root, ec);
    if (ec)
    {
      RCLCPP_ERROR(this->get_logger(), "[Session] Failed to create output root '%s': %s",
                   root.c_str(), ec.message().c_str());
      return false;
    }

    session_dir_ = fs::path(root) / ("session-" + ts + "_" + tag + "-" + behav3d::util::timeStringDateTime(now));
    fs::create_directories(session_dir_, ec);
    if (ec)
    {
      RCLCPP_ERROR(this->get_logger(), "[Session] Failed to create session_dir '%s': %s",
                   session_dir_.string().c_str(), ec.message().c_str());
      return false;
    }

    dir_color_ = session_dir_ / "color_raw";
    dir_depth_ = session_dir_ / "depth_raw";
    dir_ir_ = session_dir_ / "ir_raw";
    dir_d2c_ = session_dir_ / "depth_to_color";
    dir_c2d_ = session_dir_ / "color_to_depth";
    dir_calib_ = session_dir_ / "calib";
    fs::create_directories(dir_color_, ec);
    fs::create_directories(dir_depth_, ec);
    fs::create_directories(dir_ir_, ec);
    fs::create_directories(dir_d2c_, ec);
    fs::create_directories(dir_c2d_, ec);
    fs::create_directories(dir_calib_, ec);

    // Prime the CameraManager to write under this directory and to *not* write its own manifest
    cam_->initSession(session_dir_.string(), tag);

    // Write calibration YAMLs once at session start (best-effort)
    const double calib_timeout_sec = this->declare_parameter<double>("calib_timeout_sec", 2.0);
    if (!cam_->getCalibration(calib_timeout_sec, /*write_yaml=*/true))
    {
      RCLCPP_WARN(this->get_logger(), "[Session] Calibration not available within %.3f s; YAMLs not written for this session.", calib_timeout_sec);
    }

    // Manifest path (single JSON written at finish())
    manifest_path_ = session_dir_ / "manifest.json";

    // Clear RViz markers, show that a new session is starting
    if (viz_)
    {
      viz_->deleteAllMarkers();
    }

    RCLCPP_INFO(this->get_logger(), "[Session] Ready at %s", session_dir_.string().c_str());
    return true;
  }

  bool SessionManager::run(const std::vector<geometry_msgs::msg::PoseStamped> &targets)
  {
    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "[Session] No targets to run.");
      return false;
    }

    // Visualize all target axes (optional)
    if (viz_)
    {
      viz_->publishTargetPose(targets);
    }

    // Start from home configuration
    goHome();

    for (size_t i = 0; i < targets.size(); ++i)
    {
      const auto &tgt = targets[i];
      const std::string key = std::string("t") + behav3d::util::indexString(i, 3);

      // ---------------- PLAN ----------------
      auto traj = ctrl_ ? ctrl_->planTarget(tgt, opts_.motion_type) : nullptr;
      const bool plan_ok = (traj != nullptr);

      if (!plan_ok)
      {
        // Log attempt with current state & no files
        auto js = ctrl_->getCurrentJointState();
        auto eef = ctrl_->getCurrentPose();
        auto tool0 = ctrl_->getCurrentPose(tool0Link());
        behav3d::camera_manager::CameraManager::FilePaths files{};
        writeManifestLine(i, tgt, files, js, tool0, eef,
                          /*plan_ok=*/false, /*exec_ok=*/false, /*cap_ok=*/false,
                          this->now(), key);
        continue;
      }

      // Optional: user prompt/marker
      if (viz_)
      {
        viz_->publishTargetPose(tgt, std::string("t") + std::to_string(i));
        viz_->prompt(std::string("Move to target ") + std::to_string(i) + "/" + std::to_string(targets.size() - 1) + "?");
      }

      // ---------------- EXECUTE ----------------
      bool exec_ok = ctrl_->executeTrajectory(traj, /*apply_totg=*/opts_.apply_totg);

      // ---------------- DWELL ----------------
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(opts_.wait_time_sec)));

      // ---------------- CAPTURE ----------------
      behav3d::camera_manager::CameraManager::FilePaths files{};
      rclcpp::Time stamp = this->now();
      bool cap_ok = cam_->capture(key, files, &stamp);

      // ---------------- LOG ----------------
      auto js = ctrl_->getCurrentJointState();
      auto tool0 = ctrl_->getCurrentPose(tool0Link());
      auto eef = ctrl_->getCurrentPose(ctrl_->getEefLink());

      writeManifestLine(i, tgt, files, js, tool0, eef,
                        plan_ok, exec_ok, cap_ok,
                        stamp, key);
    }

    return true;
  }

  void SessionManager::finish()
  {
    if (cam_)
      cam_->waitForIdle();
    if (viz_)
      viz_->deleteAllMarkers();

    // Compose and write manifest.json
    rclcpp::Time end_stamp = this->now();
    try
    {
      nlohmann::json root;
      root["session_tag"] = opts_.session_tag;
      root["motion_type"] = opts_.motion_type;
      root["apply_totg"] = opts_.apply_totg;
      root["wait_time_sec"] = opts_.wait_time_sec;
      root["start_stamp_ns"] = start_stamp_.nanoseconds();
      root["end_stamp_ns"] = end_stamp.nanoseconds();
      root["root_dir"] = session_dir_.string();

      nlohmann::json caps = nlohmann::json::array();
      for (const auto &s : manifest_entries_)
      {
        try
        {
          caps.push_back(nlohmann::json::parse(s));
        }
        catch (...)
        {
          // If a single entry is malformed, keep going (optional: log a warning)
        }
      }
      root["captures"] = std::move(caps);

      if (!behav3d::util::writeJson(manifest_path_.string(), root))
      {
        RCLCPP_ERROR(this->get_logger(), "[Session] Failed to write manifest: %s", manifest_path_.string().c_str());
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "[Session] Wrote manifest: %s", manifest_path_.string().c_str());
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "[Session] Error writing manifest: %s", e.what());
    }

    goHome();
  }

  void SessionManager::goHome()
  {
    if (ctrl_ && home_joints_rad_)
    {
      if (auto traj = ctrl_->planJoints(*home_joints_rad_))
      {
        (void)ctrl_->executeTrajectory(traj, /*apply_totg=*/false);
      }
    }
  }

  void SessionManager::writeManifestLine(std::size_t i,
                                         const geometry_msgs::msg::PoseStamped &tgt,
                                         const behav3d::camera_manager::CameraManager::FilePaths &files,
                                         const sensor_msgs::msg::JointState &js,
                                         const geometry_msgs::msg::PoseStamped &tool0,
                                         const geometry_msgs::msg::PoseStamped &eef,
                                         bool plan_ok, bool exec_ok, bool cap_ok,
                                         const rclcpp::Time &stamp,
                                         const std::string &key)
  {
    // Compose a single JSON object string for this capture
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);
    oss << "{";
    oss << "\"index\":" << i << ",";
    oss << "\"stamp_ns\":" << stamp.nanoseconds() << ",";
    oss << "\"plan_ok\":" << (plan_ok ? "true" : "false") << ",";
    oss << "\"exec_ok\":" << (exec_ok ? "true" : "false") << ",";
    oss << "\"capture_ok\":" << (cap_ok ? "true" : "false") << ",";
    oss << "\"key\":\"" << key << "\",";
    oss << "\"files\":{"
        << "\"ir\":" << (files.ir.empty() ? "null" : ("\"" + files.ir + "\"")) << ","
        << "\"color\":" << (files.color.empty() ? "null" : ("\"" + files.color + "\"")) << ","
        << "\"depth\":" << (files.depth.empty() ? "null" : ("\"" + files.depth + "\"")) << ","
        << "\"d2c\":" << (files.d2c.empty() ? "null" : ("\"" + files.d2c + "\"")) << ","
        << "\"c2d\":" << (files.c2d.empty() ? "null" : ("\"" + files.c2d + "\""))
        << "},";
    oss << "\"target\":" << behav3d::util::toJsonPose(tgt) << ",";
    oss << "\"pose_tool0\":" << behav3d::util::toJsonPose(tool0) << ",";
    oss << "\"pose_eef\":" << behav3d::util::toJsonPose(eef) << ",";
    oss << "\"joints\":" << behav3d::util::toJsonJoints(js);
    oss << "}";

    manifest_entries_.push_back(oss.str());
  }

  std::string SessionManager::getSessionDir() const
  {
    return session_dir_.string();
  }

} // namespace behav3d::session_manager
