// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author:
//   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Maintainers:
//   - Lucas José Helle <luh@iti.sdu.dk>
//   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-08
// =============================================================================

#include "behav3d_cpp/session_manager.hpp"

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

  // ─────────────────────────────────────────────────────────────────────────────
  // Helpers (local to this translation unit)
  // ─────────────────────────────────────────────────────────────────────────────
  static std::string indexString(std::size_t idx, int width)
  {
    std::ostringstream oss;
    oss << std::setw(width) << std::setfill('0') << idx;
    return oss.str();
  }

  static std::string timeStringDateTime(const rclcpp::Time &t)
  {
    std::time_t tt = static_cast<time_t>(t.seconds());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d-%02d%02d%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buf);
  }

  static std::string toJsonPose(const geometry_msgs::msg::PoseStamped &ps)
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);
    oss << "{\"frame\":\"" << ps.header.frame_id << "\",";
    oss << "\"pos\":[" << ps.pose.position.x << "," << ps.pose.position.y << "," << ps.pose.position.z << "],";
    oss << "\"quat\":[" << ps.pose.orientation.x << "," << ps.pose.orientation.y << ","
        << ps.pose.orientation.z << "," << ps.pose.orientation.w << "]}";
    return oss.str();
  }

  static std::string toJsonJoints(const sensor_msgs::msg::JointState &js)
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);
    oss << "{\"names\":[";
    for (size_t i = 0; i < js.name.size(); ++i)
    {
      if (i)
        oss << ",";
      oss << "\"" << js.name[i] << "\"";
    }
    oss << "],\"pos\":[";
    for (size_t i = 0; i < js.position.size(); ++i)
    {
      if (i)
        oss << ",";
      oss << js.position[i];
    }
    oss << "]}";
    return oss.str();
  }

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
    // Where to create session directories (keep same default as CameraManager)
    output_dir_ = this->declare_parameter<std::string>("output_dir", "~/behav3d_captures");
  }

  bool SessionManager::initSession(const Options &opts)
  {
    opts_ = opts;

    // Build session directory name
    auto now = this->now();
    const std::string ts = timeStringDateTime(now);
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

    session_dir_ = fs::path(root) / ("session-" + ts + "_" + tag);
    fs::create_directories(session_dir_, ec);
    if (ec)
    {
      RCLCPP_ERROR(this->get_logger(), "[Session] Failed to create session_dir '%s': %s",
                   session_dir_.string().c_str(), ec.message().c_str());
      return false;
    }

    // Create standard subfolders (match CameraManager convention)
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
    cam_->beginSession(session_dir_.string(), tag);

    // Open manifest JSONL (single source of truth)
    manifest_path_ = session_dir_ / "captures.jsonl";
    manifest_.open(manifest_path_, std::ios::out | std::ios::app);
    if (!manifest_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "[Session] Failed to open manifest: %s", manifest_path_.string().c_str());
      return false;
    }

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
      const std::string key = std::string("t") + indexString(i, 3);

      // ---------------- PLAN ----------------
      auto traj = ctrl_ ? ctrl_->planTarget(tgt, opts_.motion_type) : nullptr;
      const bool plan_ok = (traj != nullptr);

      if (!plan_ok)
      {
        // Log attempt with current state & no files
        auto js = ctrl_->getCurrentJointState();
        auto tool0 = ctrl_->getCurrentPose("ur10e_tool0");
        auto eef = ctrl_->getCurrentPose(ctrl_->getEefLink());
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
      bool cap_ok = cam_->capture(key, &files, &stamp);

      // ---------------- LOG ----------------
      auto js = ctrl_->getCurrentJointState();
      auto tool0 = ctrl_->getCurrentPose("tool0");
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
    goHome();
    if (manifest_.is_open())
      manifest_.close();
  }

  void SessionManager::goHome()
  {
    std::vector<double> home_rad;
    if (home_joints_rad_)
    {
      home_rad = *home_joints_rad_;
    }
    else
    {
      // Same default as demo.cpp
      const std::vector<double> home_deg = {45.0, -120.0, 120.0, -90.0, 90.0, -180.0};
      home_rad.reserve(home_deg.size());
      for (double d : home_deg)
        home_rad.push_back(d * (3.14159265358979323846 / 180.0));
    }
    if (ctrl_)
    {
      if (auto traj = ctrl_->planJoints(home_rad))
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
    if (!manifest_.is_open())
      return;

    // Compose a single JSON line (no external deps)
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);
    oss << "{";
    oss << "\"i\":" << i << ",";
    oss << "\"stamp_ns\":" << stamp.nanoseconds() << ",";
    oss << "\"session_tag\":\"" << opts_.session_tag << "\",";
    oss << "\"motion_type\":\"" << opts_.motion_type << "\",";
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
    oss << "\"target\":" << toJsonPose(tgt) << ",";
    oss << "\"pose_tool0\":" << toJsonPose(tool0) << ",";
    oss << "\"pose_eef\":" << toJsonPose(eef) << ",";
    oss << "\"joints\":" << toJsonJoints(js);
    oss << "}";

    manifest_ << oss.str() << "\n";
    manifest_.flush();
  }

} // namespace behav3d::session_manager
