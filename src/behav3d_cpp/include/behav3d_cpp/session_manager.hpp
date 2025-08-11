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
      const std::shared_ptr<camera_manager::CameraManager>& cam,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  // Simple test method to verify wiring
  void sayHello(const std::string& who = "world");

private:
  std::shared_ptr<motion_controller::PilzMotionController> ctrl_;
  std::shared_ptr<motion_visualizer::MotionVisualizer>     viz_;
  std::shared_ptr<camera_manager::CameraManager>           cam_;
};

}  // namespace behav3d::session_manager
