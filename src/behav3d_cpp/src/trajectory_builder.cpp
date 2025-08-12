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
#include <vector>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/util.hpp"

namespace behav3d::trajectory_builder
{

    using behav3d::target_builder::adjustTarget;
    using behav3d::target_builder::alignTarget;
    using behav3d::target_builder::changeBasis;
    using behav3d::target_builder::flipTargetAxes;
    using behav3d::target_builder::fromIso;
    using behav3d::target_builder::poseBetween;
    using behav3d::target_builder::rebase;
    using behav3d::target_builder::toIso;
    using behav3d::target_builder::translate;
    using behav3d::target_builder::worldXY;
    using behav3d::target_builder::xAxis;
    using behav3d::target_builder::yAxis;
    using behav3d::target_builder::zAxis;
    using behav3d::util::deg2rad;
    using geometry_msgs::msg::PoseStamped;

    // Generate uniformly-distributed unit directions (Fibonacci spiral)
    namespace
    {
        // Return `n` nearly‑uniform directions on the unit sphere (Fibonacci spiral)
        inline std::vector<Eigen::Vector3d> fibonacciDir(int n)
        {
            constexpr double golden = (1.0 + std::sqrt(5.0)) * 0.5;
            std::vector<Eigen::Vector3d> v;
            v.reserve(n);

            for (int i = 0; i < n; ++i)
            {
                const double lat = std::acos(1.0 - 2.0 * (i + 0.5) / n);
                const double lon = 2.0 * M_PI * (i + 0.5) / golden;
                const double x = std::sin(lat) * std::cos(lon);
                const double y = std::sin(lat) * std::sin(lon);
                const double z = std::cos(lat);
                v.emplace_back(x, y, z);
            }
            return v;
        }
    }

    // Place n camera poses on a spherical cap of radius r around tgt
    //
    // 1. Sample directions on a unit sphere in the target frame (Fibonacci order).
    // 2. Build each pose locally so +Z looks at the target and +X aligns with it.
    // 3. Scale positions by `r` and transform once to world with changeBasis().
    std::vector<PoseStamped> fibonacciSphericalCap(const PoseStamped &tgt,
                                                   double r,
                                                   double cap,
                                                   int n)
    {
        // Build poses locally on a unit sphere, orient them, then scale and rebase.

        std::vector<PoseStamped> out;
        if (n <= 0)
            return out;
        out.reserve(n);

        const double cos_cap = std::cos(cap);                   // cap half-angle in radians
        constexpr double golden = (1.0 + std::sqrt(5.0)) * 0.5; // φ

        for (int i = n - 1; i >= 0; --i)
        {
            // --- 1. Direction on unit-sphere cap in *target-local* coordinates
            //    using a Fibonacci spiral.
            const double z_local = cos_cap + (1.0 - cos_cap) *
                                                 ((i + 0.5) / static_cast<double>(n));
            const double lon = 2.0 * M_PI * (i + 0.5) / golden;
            const double r_xy = std::sqrt(std::max(0.0, 1.0 - z_local * z_local));

            const Eigen::Vector3d d_local(r_xy * std::cos(lon),
                                          r_xy * std::sin(lon),
                                          z_local); // outward

            // --- 2. Pose at radius r, expressed in target frame.
            PoseStamped p_local = worldXY(r * d_local.x(),
                                          r * d_local.y(),
                                          r * d_local.z(),
                                          tgt.header.frame_id);

            // +Z must look toward the target centre ⇒ inward (‑d_local)
            p_local = adjustTarget(p_local, -d_local);

            // Roll so +X aligns (minimally) with target +X (== local +X)
            p_local = alignTarget(p_local, Eigen::Vector3d::UnitX());

            // --- 3. Express pose in the world frame of tgt.
            out.emplace_back(changeBasis(tgt, p_local));
        }

        return out;
    }

    // Generate a zig‑zag raster sweep in tgt’s XY plane at z_off
    std::vector<PoseStamped> sweepZigzag(const PoseStamped &tgt,
                                         double width, double height, double z_off,
                                         int nx, int ny, bool row_major)
    {
        std::vector<PoseStamped> out;
        if (nx < 2 || ny < 2)
            return out;
        out.reserve(nx * ny);

        // Pre-compute a canonical local pose
        PoseStamped p_base_local = flipTargetAxes(worldXY(0.0, 0.0, 0.0, tgt.header.frame_id), false, true);

        const double dx = width / (nx - 1);
        const double dy = height / (ny - 1);

        for (int j = 0; j < ny; ++j)
        {
            std::vector<PoseStamped> row;
            row.reserve(nx);

            for (int i = 0; i < nx; ++i)
            {
                // --- Coordinates in the target‑local XY plane
                const double x = -0.5 * width + i * dx;
                const double y = -0.5 * height + j * dy;

                // Translate the canonical local pose by (x, y, z_off)
                PoseStamped p_local = translate(p_base_local,
                                                Eigen::Vector3d(x, y, -z_off));

                row.emplace_back(changeBasis(tgt, p_local));
            }

            // Optional serpentine (zig‑zag) ordering
            if (!row_major && (j % 2 == 1))
                std::reverse(row.begin(), row.end());

            out.insert(out.end(), row.begin(), row.end());
        }
        return out;
    }

    // Add uncorrelated Gaussian translation/rotation noise to each pose
    std::vector<PoseStamped> addJitter(const std::vector<PoseStamped> &poses,
                                       double trans_std,
                                       double rot_std_deg,
                                       unsigned seed)
    {
        std::mt19937 gen(seed);
        std::normal_distribution<double> dt(0.0, trans_std);
        std::normal_distribution<double> dr(0.0, rot_std_deg);

        std::vector<PoseStamped> out;
        out.reserve(poses.size());

        for (const auto &p : poses)
        {
            Eigen::Vector3d d(dt(gen), dt(gen), dt(gen));
            Eigen::Vector3d r(dr(gen), dr(gen), dr(gen));
            out.emplace_back(
                behav3d::target_builder::rotateEuler(
                    translate(p, d),
                    r,
                    true)); // degrees
        }
        return out;
    }

    // Insert `steps` evenly‑spaced SLERP/lerp blends between consecutive key poses
    std::vector<PoseStamped> blend(const std::vector<PoseStamped> &poses,
                                   size_t steps)
    {
        if (poses.size() < 2 || steps == 0)
            return poses;

        std::vector<PoseStamped> out;
        for (size_t i = 0; i + 1 < poses.size(); ++i)
        {
            out.push_back(poses[i]); // keyframe
            for (size_t s = 1; s <= steps; ++s)
            {
                const double t = static_cast<double>(s) / (steps + 1);
                out.emplace_back(poseBetween(poses[i], poses[i + 1], t));
            }
        }
        out.push_back(poses.back());
        return out;
    }

    // Distribute poses uniformly on a sphere shell of given radius (Fibonacci)
    std::vector<PoseStamped> uniformSphere(const PoseStamped &center,
                                           double radius, int n)
    {
        std::vector<PoseStamped> out;
        out.reserve(n);

        for (const auto &d : fibonacciDir(n))
            out.emplace_back(translate(center, radius * d));

        return out;
    }

} // namespace behav3d::trajectory_builder
