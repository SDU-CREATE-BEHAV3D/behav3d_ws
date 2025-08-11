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

#include "behav3d_cpp/session_manager.hpp"
#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/camera_manager.hpp"

namespace behav3d::session_manager {

SessionManager::SessionManager( const std::shared_ptr<motion_controller::PilzMotionController>& ctrl,
                                const std::shared_ptr<motion_visualizer::MotionVisualizer>& viz,
                                const std::shared_ptr<camera_manager::CameraManager>& cam,
                                const rclcpp::NodeOptions& options): rclcpp::Node("session_manager_cpp", options),
  ctrl_(ctrl),
  viz_(viz),
  cam_(cam)
{
  RCLCPP_INFO(this->get_logger(),
              "[SessionManager] initialized (ctrl:%s viz:%s cam:%s)",
              ctrl_ ? "yes" : "no",
              viz_  ? "yes" : "no",
              cam_  ? "yes" : "no");
}

void SessionManager::sayHello(const std::string& who) {
  RCLCPP_INFO(this->get_logger(), "[SessionManager] Hello, %s 👋", who.c_str());
}

}  // namespace behav3d::session_manager
