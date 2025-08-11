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

namespace behav3d::session_manager {

class SessionManager : public rclcpp::Node {
public:
  // Basic constructor; expand later as needed
  explicit SessionManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Simple test method to verify wiring
  void sayHello(const std::string& who = "world");

};

}  // namespace behav3d::session_manager
