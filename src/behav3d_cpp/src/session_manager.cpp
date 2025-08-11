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

namespace behav3d::session_manager {

SessionManager::SessionManager(const rclcpp::NodeOptions& options)
: rclcpp::Node("session_manager_cpp", options) {
  RCLCPP_INFO(this->get_logger(), "[SessionManager] initialized");
}

void SessionManager::sayHello(const std::string& who) {
  RCLCPP_INFO(this->get_logger(), "[SessionManager] Hello, %s 👋", who.c_str());
}

}  // namespace behav3d::session_manager
