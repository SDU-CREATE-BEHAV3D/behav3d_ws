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
#include <vector>
#include <random>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/util.hpp"

namespace behav3d
{
    namespace trajectory_builder
    {

        using behav3d::target_builder::adjustTarget;
        using behav3d::target_builder::alignTarget;
        using behav3d::target_builder::changeBasis;
        using behav3d::target_builder::flipTarget;
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

        // -----------------------------------------------------------------------------
        // Fibonacci sphere
        // -----------------------------------------------------------------------------
        namespace
        {
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

        // Fibonacci distribution on a spherical cap (outside‑in)
        std::vector<PoseStamped> fibonacciSphericalCap(const PoseStamped &tgt,
                                                       double r,
                                                       double cap,
                                                       int n)
        {
            // ---------------------------------------------------------------------
            // Generate camera poses on a spherical cap around the target such that
            //   • the camera position lies on a sphere of radius r centerd at tgt,
            //   • the camera +Z axis looks directly at tgt,
            //   • the camera is rolled minimally so that its +X axis matches the
            //     +X axis of the target frame (up to the ambiguity when the two
            //     axes are parallel to +Z).
            //
            // This follows the same logic as the original Grasshopper/Python script
            // but fully in *world* coordinates to avoid the frame‑mixing bug that
            // caused +Z not to point to the center when the target frame was
            // rotated relative to the world frame.
            // ---------------------------------------------------------------------

            std::vector<PoseStamped> out;
            if (n <= 0)
                return out;
            out.reserve(n);

            // Target → world transform (constant for all samples)
            const Eigen::Isometry3d iso_tgt = toIso(tgt);
            const Eigen::Vector3d center_world = iso_tgt.translation();
            const Eigen::Matrix3d R_tgt_world = iso_tgt.rotation();

            const double cos_cap = std::cos(cap);
            constexpr double golden = (1.0 + std::sqrt(5.0)) * 0.5; // φ

            for (int i = n - 1; i >= 0; --i)
            {
                // -----------------------------------------------------------------
                // 1. Direction (in *local target* coordinates) on the spherical cap
                //    using a Fibonacci spiral.
                // -----------------------------------------------------------------
                const double z_local = cos_cap + (1.0 - cos_cap) *
                                                     ((i + 0.5) / static_cast<double>(n));
                const double lon = 2.0 * M_PI * (i + 0.5) / golden;
                const double r_xy = std::sqrt(std::max(0.0, 1.0 - z_local * z_local));

                const Eigen::Vector3d d_local(r_xy * std::cos(lon),
                                              r_xy * std::sin(lon),
                                              z_local); // outward

                // -----------------------------------------------------------------
                // 2. Camera position in *world* coordinates.
                // -----------------------------------------------------------------
                const Eigen::Vector3d d_world = R_tgt_world * d_local;
                const Eigen::Vector3d cam_pos_world = center_world + r * d_world;

                // -----------------------------------------------------------------
                // 3. Begin pose with identity orientation in world frame.
                // -----------------------------------------------------------------
                PoseStamped p = worldXY(cam_pos_world.x(),
                                        cam_pos_world.y(),
                                        cam_pos_world.z(),
                                        tgt.header.frame_id);

                // -----------------------------------------------------------------
                // 4. Adjust +Z so it looks at the target center.
                // -----------------------------------------------------------------
                const Eigen::Vector3d z_world = (center_world - cam_pos_world).normalized();
                p = adjustTarget(p, z_world);

                // -----------------------------------------------------------------
                // 5. Roll about +Z so +X aligns (minimally) with the target’s +X.
                // -----------------------------------------------------------------
                const Eigen::Vector3d x_tgt_world = R_tgt_world.col(0); // target +X in world
                const Eigen::Vector3d x_proj = x_tgt_world - x_tgt_world.dot(z_world) * z_world;
                p = alignTarget(p, x_proj);

                out.emplace_back(p);
            }

            return out;
        }

        // -----------------------------------------------------------------------------
        // Raster sweep (zig‑zag) in local target frame
        // -----------------------------------------------------------------------------
        std::vector<PoseStamped> sweepZigzag(const PoseStamped &tgt,
                                             double width, double height, double z_off,
                                             int nx, int ny, bool row_major)
        {
            std::vector<PoseStamped> out;

            if (nx < 2 || ny < 2)
                return out;

            const double dx = width / (nx - 1);
            const double dy = height / (ny - 1);

            for (int j = 0; j < ny; ++j)
            {
                std::vector<PoseStamped> row;
                row.reserve(nx);
                for (int i = 0; i < nx; ++i)
                {
                    const double x = -0.5 * width + i * dx;
                    const double y = -0.5 * height + j * dy;
                    Eigen::Vector3d off = x * xAxis(tgt) +
                                          y * yAxis(tgt) +
                                          z_off * zAxis(tgt); // Offset along +Z of center target

                    auto p = translate(tgt, off);
                    p = flipTarget(p); // Flip so +Z points toward –Z of center
                    row.emplace_back(p);
                }
                if (!row_major && (j % 2 == 1))
                    std::reverse(row.begin(), row.end());
                out.insert(out.end(), row.begin(), row.end());
            }
            return out;
        }

        // -----------------------------------------------------------------------------
        // Add Gaussian noise to poses
        // -----------------------------------------------------------------------------
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

        // -----------------------------------------------------------------------------
        // Linear blend insertion between key poses
        // -----------------------------------------------------------------------------
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

        // -----------------------------------------------------------------------------
        // Uniform distribution on sphere shell
        // -----------------------------------------------------------------------------
        std::vector<PoseStamped> uniformSphere(const PoseStamped &center,
                                               double radius, int n)
        {
            std::vector<PoseStamped> out;
            out.reserve(n);

            for (const auto &d : fibonacciDir(n))
                out.emplace_back(translate(center, radius * d));

            return out;
        }

    } // namespace trajectory_builder
} // namespace behav3d