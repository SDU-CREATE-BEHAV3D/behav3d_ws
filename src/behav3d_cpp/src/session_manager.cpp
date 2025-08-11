#include "behav3d_cpp/session_manager.hpp"

namespace behav3d::session_manager {

SessionManager::SessionManager(const rclcpp::NodeOptions& options)
: rclcpp::Node("session_manager_cpp", options) {
  RCLCPP_INFO(this->get_logger(), "[SessionManager] initialized");
}

}  // namespace behav3d::session_manager
