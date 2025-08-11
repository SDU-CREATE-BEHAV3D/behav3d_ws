// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: Lucas Helle Pessot <luh@iti.sdu.dk>
// Maintainers:
//   - Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
//   - Joseph Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <string>
#include <filesystem>

namespace behav3d::motion_controller { class PilzMotionController; }
namespace behav3d::motion_visualizer { class MotionVisualizer; }
namespace behav3d::camera_manager   { class CameraManager; }

namespace behav3d::session_manager {

class SessionManager : public rclcpp::Node {
public:
  // Constructor;
  explicit SessionManager(
      const std::shared_ptr<motion_controller::PilzMotionController>& ctrl,
      const std::shared_ptr<motion_visualizer::MotionVisualizer>& viz,
      const std::shared_ptr<camera_manager::CameraManager>& cam);
  // Simple test method to verify wiring
  void sayHello(const std::string& who = "world");

  // Joint-space "home" using your default angles
  void home();

  // Go to a specific pose (LIN by default)
  void home(const geometry_msgs::msg::PoseStamped& target,
            const std::string& motion_type = "LIN",
            double vel_scale = 0.5,
            double acc_scale = 0.5);

// Initialize a session from a file + a single target
bool initScan(const std::string& filename,
          const geometry_msgs::msg::PoseStamped& pose);

// Initialize a session from a file + multiple targets
bool initScan(const std::string& filename,
          const std::vector<geometry_msgs::msg::PoseStamped>& poses);

const std::filesystem::path& getSessionDir() const { return session_dir_; }
const std::filesystem::path& getCapturesRoot() const { return captures_root_; }

private:

  // helper to create captures/ and session dir
  bool initScanDirs(const std::string& prefix);
  // paths we keep around
  std::filesystem::path captures_root_;
  std::filesystem::path session_dir_;

  std::shared_ptr<motion_controller::PilzMotionController> ctrl_;
  std::shared_ptr<motion_visualizer::MotionVisualizer>     viz_;
  std::shared_ptr<camera_manager::CameraManager>           cam_;
};

}  // namespace behav3d::session_manager
