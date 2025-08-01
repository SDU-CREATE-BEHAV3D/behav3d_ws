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

#include <vector>
#include <algorithm>
#include <random>
#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "behav3d/target_builder.hpp"
#include "behav3d/util.hpp"

namespace behav3d {
namespace trajectory_builder {

using geometry_msgs::msg::PoseStamped;
using behav3d::target_builder::translate;
using behav3d::target_builder::poseBetween;
using behav3d::target_builder::xAxis;
using behav3d::target_builder::yAxis;
using behav3d::target_builder::zAxis;
using behav3d::target_builder::worldXY;
using behav3d::target_builder::flipTarget;
using behav3d::target_builder::adjustTarget;
using behav3d::target_builder::alignTarget;
using behav3d::target_builder::toIso;
using behav3d::target_builder::fromIso;
using behav3d::util::deg2rad;
using behav3d::target_builder::changeBasis;
using behav3d::target_builder::rebase;

// -----------------------------------------------------------------------------
// Grid in XY plane
// -----------------------------------------------------------------------------
std::vector<PoseStamped> gridXY(const std::pair<double, double>& x_range,
                                const std::pair<double, double>& y_range,
                                double z,
                                double spacing,
                                const std::string& frame,
                                bool flipped)
{
    std::vector<PoseStamped> out;
    for (double x = x_range.first; x <= x_range.second + 1e-9; x += spacing)
        for (double y = y_range.first; y <= y_range.second + 1e-9; y += spacing)
        {
            PoseStamped p = worldXY(x, y, z, frame);
            if (flipped)
                p = flipTarget(p);
            out.emplace_back(p);
        }
    return out;
}

// -----------------------------------------------------------------------------
// Fibonacci sphere
// -----------------------------------------------------------------------------
namespace {
inline std::vector<Eigen::Vector3d> fibonacciDir(int n)
{
    constexpr double golden = (1.0 + std::sqrt(5.0)) * 0.5;
    std::vector<Eigen::Vector3d> v;
    v.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        const double lat  = std::acos(1.0 - 2.0 * (i + 0.5) / n);
        const double lon  = 2.0 * M_PI * (i + 0.5) / golden;
        const double x    = std::sin(lat) * std::cos(lon);
        const double y    = std::sin(lat) * std::sin(lon);
        const double z    = std::cos(lat);
        v.emplace_back(x, y, z);
    }
    return v;
}
}

// Fibonacci distribution on a spherical cap (outside‑in)
std::vector<PoseStamped> fibonacciSphericalCap(const PoseStamped& tgt,
                                               double r,
                                               double cap,
                                               int n)
{
    // cos(cap) is the Z‑threshold for the cap
    const double cos_cap = std::cos(cap);

    // ---------------------------------------------------------------------
    // 1. Generate exactly n unit directions on the spherical cap
    //    using a Fibonacci spiral restricted to z ∈ [cos_cap, 1].
    // ---------------------------------------------------------------------
    constexpr double golden = (1.0 + std::sqrt(5.0)) * 0.5;   // φ
    std::vector<Eigen::Vector3d> dirs;
    dirs.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        // Uniform area sampling on the cap:
        //   Choose z uniformly in [cos_cap, 1] and longitude by golden‑angle.
        double z   = cos_cap + (1.0 - cos_cap) * ((i + 0.5) / static_cast<double>(n));
        double lon = 2.0 * M_PI * (i + 0.5) / golden;
        double r_xy = std::sqrt(std::max(0.0, 1.0 - z * z));
        double x = r_xy * std::cos(lon);
        double y = r_xy * std::sin(lon);
        dirs.emplace_back(x, y, z);
    }

    // ---------------------------------------------------------------------
    // 2. Convert directions into camera poses.
    // ---------------------------------------------------------------------
    std::vector<PoseStamped> out;
    out.reserve(n);

    for (int i = n - 1; i >= 0; --i)
    {
        const Eigen::Vector3d& d = dirs[i];

        //------------------------------------------------------------------
        // 1) Build pose in the target's *local* XY frame
        //------------------------------------------------------------------
        Eigen::Vector3d pos_local = r * d;                 // on unit shell
        PoseStamped p_local = worldXY(pos_local.x(),
                                      pos_local.y(),
                                      pos_local.z(),
                                      tgt.header.frame_id);

        // Orient camera so +Z looks at the center and roll so +X ≈ world X
        p_local = adjustTarget(p_local, -d);               // new Z
        p_local = alignTarget(p_local, Eigen::Vector3d::UnitX());

        //------------------------------------------------------------------
        // 2) Change basis: transform local pose into the target frame
        //------------------------------------------------------------------
        PoseStamped p = changeBasis(tgt, p_local);

        out.emplace_back(p);
    }
    return out;
}

// -----------------------------------------------------------------------------
// Raster sweep (zig‑zag) in local target frame
// -----------------------------------------------------------------------------
std::vector<PoseStamped> sweepZigzag(const PoseStamped& tgt,
                                     double width, double height, double z_off,
                                     int nx, int ny, bool row_major)
{
    std::vector<PoseStamped> out;

    if (nx < 2 || ny < 2)
        return out;

    const double dx = width  / (nx - 1);
    const double dy = height / (ny - 1);

    for (int j = 0; j < ny; ++j)
    {
        std::vector<PoseStamped> row;
        row.reserve(nx);
        for (int i = 0; i < nx; ++i)
        {
            const double x = -0.5 * width  + i * dx;
            const double y = -0.5 * height + j * dy;
            Eigen::Vector3d off =  x * xAxis(tgt) +
                                   y * yAxis(tgt) -
                                   z_off * zAxis(tgt);
            row.emplace_back(translate(tgt, off));
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
std::vector<PoseStamped> addJitter(const std::vector<PoseStamped>& poses,
                                   double trans_std,
                                   double rot_std_deg,
                                   unsigned seed)
{
    std::mt19937 gen(seed);
    std::normal_distribution<double> dt(0.0, trans_std);
    std::normal_distribution<double> dr(0.0, rot_std_deg);

    std::vector<PoseStamped> out;
    out.reserve(poses.size());

    for (const auto& p : poses)
    {
        Eigen::Vector3d d(dt(gen), dt(gen), dt(gen));
        Eigen::Vector3d r(dr(gen), dr(gen), dr(gen));
        out.emplace_back(
            behav3d::target_builder::rotateEuler(
                translate(p, d),
                r,
                true));            // degrees
    }
    return out;
}

// -----------------------------------------------------------------------------
// Linear blend insertion between key poses
// -----------------------------------------------------------------------------
std::vector<PoseStamped> blend(const std::vector<PoseStamped>& poses,
                               size_t steps)
{
    if (poses.size() < 2 || steps == 0) return poses;

    std::vector<PoseStamped> out;
    for (size_t i = 0; i + 1 < poses.size(); ++i)
    {
        out.push_back(poses[i]);          // keyframe
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
std::vector<PoseStamped> uniformSphere(const PoseStamped& center,
                                       double radius, int n)
{
    std::vector<PoseStamped> out;
    out.reserve(n);

    for (const auto& d : fibonacciDir(n))
        out.emplace_back(translate(center, radius * d));

    return out;
}

} // namespace trajectory_builder
} // namespace behav3d