// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
// Author: behav3d team
// =============================================================================

#include "behav3d_cpp/util.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <random>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

namespace behav3d::util
{
    // ─────────────────────────────────────────────────────────────────────────
    // Angles & random
    // ─────────────────────────────────────────────────────────────────────────
    double deg2rad(double deg) { return deg * M_PI / 180.0; }
    double rad2deg(double rad) { return rad * 180.0 / M_PI; }

    double wrapAngle(double rad)
    {
        // wrap to (-pi, pi]
        double x = std::fmod(rad + M_PI, 2.0 * M_PI);
        if (x < 0) x += 2.0 * M_PI;
        return x - M_PI;
    }

    Eigen::Vector3d randomUnitVector(unsigned seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> unif(-1.0, 1.0);
        std::uniform_real_distribution<double> angle(0.0, 2.0 * M_PI);

        // Marsaglia method
        double z = unif(rng);
        double t = angle(rng);
        double r = std::sqrt(std::max(0.0, 1.0 - z*z));
        return { r * std::cos(t), r * std::sin(t), z };
    }

    Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy, bool degrees)
    {
        const double rx = degrees ? deg2rad(rpy.x()) : rpy.x();
        const double ry = degrees ? deg2rad(rpy.y()) : rpy.y();
        const double rz = degrees ? deg2rad(rpy.z()) : rpy.z();

        Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());

        // XYZ extrinsic (roll, pitch, yaw)
        Eigen::Quaterniond q = Rz * Ry * Rx;
        q.normalize();
        return q;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Tiny JSON/YAML helpers
    // ─────────────────────────────────────────────────────────────────────────
    bool readJson(const std::string &path, nlohmann::json &out)
    {
        try {
            std::ifstream ifs(path);
            if (!ifs.is_open()) return false;
            ifs >> out;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool writeJson(const std::string &path, const nlohmann::json &j)
    {
        try {
            std::ofstream ofs(path);
            if (!ofs.is_open()) return false;
            ofs << std::setw(2) << j;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool readYaml(const std::string &path, YAML::Node &out)
    {
        try {
            out = YAML::LoadFile(path);
            return true;
        } catch (...) {
            return false;
        }
    }

    bool writeYaml(const std::string &path, const YAML::Node &node)
    {
        try {
            std::ofstream ofs(path);
            if (!ofs.is_open()) return false;
            ofs << node;
            return true;
        } catch (...) {
            return false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Misc utils
    // ─────────────────────────────────────────────────────────────────────────
    std::string indexString(std::size_t idx, int width)
    {
        std::ostringstream oss;
        oss << std::setw(width) << std::setfill('0') << idx;
        return oss.str();
    }

    std::string timeStringDateTime(const rclcpp::Time &t)
    {
        // Format as YYYYMMDD-HHMMSS
        int64_t nsec_total = t.nanoseconds();
        int64_t sec = nsec_total / 1000000000LL;

        std::time_t tt = static_cast<std::time_t>(sec);
        std::tm bt{};
    #ifdef _WIN32
        localtime_s(&bt, &tt);
    #else
        localtime_r(&tt, &bt);
    #endif
        std::ostringstream oss;
        oss << std::put_time(&bt, "%Y%m%d-%H%M%S");
        return oss.str();
    }

    std::string toJsonPose(const geometry_msgs::msg::PoseStamped &ps)
    {
        nlohmann::json j;
        j["frame"] = ps.header.frame_id;
        j["stamp"] = {
            {"sec",  ps.header.stamp.sec},
            {"nsec", ps.header.stamp.nanosec}
        };
        j["pos"]  = { ps.pose.position.x, ps.pose.position.y, ps.pose.position.z };
        j["quat"] = { ps.pose.orientation.x, ps.pose.orientation.y,
                      ps.pose.orientation.z, ps.pose.orientation.w };
        return j.dump();
    }

    std::string toJsonJoints(const sensor_msgs::msg::JointState &js)
    {
        nlohmann::json j;
        j["stamp"] = {
            {"sec",  js.header.stamp.sec},
            {"nsec", js.header.stamp.nanosec}
        };
        j["names"]    = js.name;
        j["position"] = js.position;
        j["velocity"] = js.velocity;
        j["effort"]   = js.effort;
        return j.dump();
    }

} // namespace behav3d::util
