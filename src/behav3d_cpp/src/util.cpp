// =============================================================================
//   ____  _____ _   _    ___     _______ ____
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/
//
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
#include <Eigen/Dense>

namespace behav3d
{
    namespace util
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

    } // namespace util
} // namespace behav3d