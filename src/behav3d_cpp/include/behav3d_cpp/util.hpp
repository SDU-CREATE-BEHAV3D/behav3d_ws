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
// Date: 2025-07
// =============================================================================

#pragma once

#include <random>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace behav3d::util
{

    /// Degrees → radians
    double deg2rad(double deg);

    /// Radians → degrees
    double rad2deg(double rad);

    /// Wrap angle to (‑π, π]
    double wrapAngle(double rad);

    /// Random unit vector (seedable, deterministic).  Default seed uses std::random_device.
    Eigen::Vector3d randomUnitVector(unsigned seed = std::random_device{}());

    /// Roll‑pitch‑yaw (XYZ extrinsic) to quaternion.
    /// If @p degrees == true, @p rpy is interpreted in degrees.
    Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy, bool degrees = false);

    // ─────────────────────────────────────────────────────────────────────────────
    // Tiny JSON/YAML wrapper (uses nlohmann::json and yaml-cpp directly)
    // ─────────────────────────────────────────────────────────────────────────────

    /// Load a JSON file into a nlohmann::json
    bool readJson(const std::string &path, nlohmann::json &out);

    /// Write a nlohmann::json to disk (pretty-printed)
    bool writeJson(const std::string &path, const nlohmann::json &j);

    /// Load a YAML file into a YAML::Node
    bool readYaml(const std::string &path, YAML::Node &out);

    /// Write a YAML::Node to disk
    bool writeYaml(const std::string &path, const YAML::Node &node);

    /// Zero-padded integer to string, width >= 1
    std::string indexString(std::size_t idx, int width);

    /// Format rclcpp::Time as YYYYMMDD-HHMMSS
    std::string timeStringDateTime(const rclcpp::Time &t);

    /// Serialize a PoseStamped to a compact JSON string.
    std::string toJsonPose(const geometry_msgs::msg::PoseStamped &ps);

    /// Serialize a JointState to a compact JSON string.
    std::string toJsonJoints(const sensor_msgs::msg::JointState &js);

} // namespace behav3d::util
