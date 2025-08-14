#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

#include <cmath>
#include <random>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/time.hpp>
#include <ctime>
#include <cstdio>
#include <iomanip>
#include <sstream>

namespace behav3d::util
{

    constexpr double PI = 3.14159265358979323846;

    /// Degrees → radians
    double deg2rad(double deg) { return deg * PI / 180.0; }

    /// Radians → degrees
    double rad2deg(double rad) { return rad * 180.0 / PI; }

    /// Wrap angle to (‑π, π]
    double wrapAngle(double rad)
    {
        while (rad > PI)
            rad -= 2.0 * PI;
        while (rad <= -PI)
            rad += 2.0 * PI;
        return rad;
    }

    /// Random unit vector (seedable, deterministic)
    Eigen::Vector3d randomUnitVector(unsigned seed = std::random_device{}())
    {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        const double z = 2.0 * dist(gen) - 1.0;
        const double theta = 2.0 * PI * dist(gen);
        const double r = std::sqrt(1.0 - z * z);

        return {r * std::cos(theta), r * std::sin(theta), z};
    }

    /// Roll‑pitch‑yaw (XYZ extrinsic) to quaternion
    Eigen::Quaterniond fromRPY(const Eigen::Vector3d &rpy, bool degrees = false)
    {
        Eigen::Vector3d v = degrees ? rpy.unaryExpr([](double d)
                                                    { return deg2rad(d); })
                                    : rpy;

        Eigen::AngleAxisd rx(v.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(v.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(v.z(), Eigen::Vector3d::UnitZ());

        return rz * ry * rx;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Tiny JSON/YAML wrapper (nlohmann::json + yaml-cpp)
    // ─────────────────────────────────────────────────────────────────────────────

    bool readJson(const std::string &path, nlohmann::json &out)
    {
        try
        {
            std::ifstream ifs(path);
            if (!ifs.is_open())
                return false;
            ifs >> out; // throws on parse error
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool writeJson(const std::string &path, const nlohmann::json &j)
    {
        try
        {
            std::ofstream ofs(path);
            if (!ofs.is_open())
                return false;
            ofs << j.dump(2); // pretty-print with 2-space indent
            return static_cast<bool>(ofs);
        }
        catch (...)
        {
            return false;
        }
    }

    bool readYaml(const std::string &path, YAML::Node &out)
    {
        try
        {
            out = YAML::LoadFile(path); // throws on I/O/parse error
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool writeYaml(const std::string &path, const YAML::Node &node)
    {
        try
        {
            YAML::Emitter emitter;
            emitter << node;
            std::ofstream ofs(path);
            if (!ofs.is_open())
                return false;
            ofs << emitter.c_str();
            return static_cast<bool>(ofs);
        }
        catch (...)
        {
            return false;
        }
    }

    std::string indexString(std::size_t idx, int width)
    {
        std::ostringstream oss;
        oss << std::setw(width) << std::setfill('0') << idx;
        return oss.str();
    }

    std::string timeStringDateTime(const rclcpp::Time &t)
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

    std::string toJsonPose(const geometry_msgs::msg::PoseStamped &ps)
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

    std::string toJsonJoints(const sensor_msgs::msg::JointState &js)
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

} // namespace behav3d::util
