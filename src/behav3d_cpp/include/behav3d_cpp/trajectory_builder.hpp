// =============================================================================
//   ____  _____ _   _    ___     _______ ____  
//  | __ )| ____| | | |  / \ \   / /___ /|  _ \ 
//  |  _ \|  _| | |_| | / _ \ \ / /  |_ \| | | |
//  | |_) | |___|  _  |/ ___ \ V /  ___) | |_| |
//  |____/|_____|_| |_/_/   \_\_/  |____/|____/ 
                                              
                                              
// Author: Özgüç Bertuğ Çapunaman <ozca@iti.sdu.dk>
// Maintainers:
//   - Lucas José Helle <luh@iti.sdu.dk>
//   - Joseph Milad Wadie Naguib <jomi@iti.sdu.dk>
// Institute: University of Southern Denmark (Syddansk Universitet)
// Date: 2025-07
// =============================================================================

#pragma once

#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace behav3d {
namespace trajectory_builder {

using geometry_msgs::msg::PoseStamped;

// ---------------------------------------------------------------------------
// Regular XY grid (world frame)
// ---------------------------------------------------------------------------
std::vector<PoseStamped> gridXY(const std::pair<double, double>& x_range,
                                const std::pair<double, double>& y_range,
                                double z,
                                double spacing,
                                const std::string& frame = "world",
                                bool flipped = false);

// ---------------------------------------------------------------------------
// Fibonacci distribution on spherical cap (camera +Z looks at tgt)
//   @param tgt  Target pose (center + orientation)
//   @param r    Radius of sphere shell
//   @param cap  Spherical cap half‑angle (rad)
//   @param n    Number of viewpoints
// ---------------------------------------------------------------------------
std::vector<PoseStamped> fibonacciSphericalCap(const PoseStamped& tgt,
                                               double r,
                                               double cap,
                                               int n);

// ---------------------------------------------------------------------------
// Raster sweep (zig‑zag) in local target frame.  +X right, +Y up, −Z at tgt.
// ---------------------------------------------------------------------------
std::vector<PoseStamped> sweepZigzag(const PoseStamped& tgt,
                                     double width, double height, double z_off,
                                     int nx, int ny,
                                     bool row_major = false);

// ---------------------------------------------------------------------------
// Add translational + rotational Gaussian noise to each pose
// ---------------------------------------------------------------------------
std::vector<PoseStamped> addJitter(const std::vector<PoseStamped>& poses,
                                   double trans_std,
                                   double rot_std_deg,
                                   unsigned seed = 42U);

// ---------------------------------------------------------------------------
// Linear blend insertion between consecutive keyframes
// ---------------------------------------------------------------------------
std::vector<PoseStamped> blend(const std::vector<PoseStamped>& poses,
                               size_t steps);

// ---------------------------------------------------------------------------
// Uniform distribution on sphere shell of radius @p radius around @p center
// ---------------------------------------------------------------------------
std::vector<PoseStamped> uniformSphere(const PoseStamped& center,
                                       double radius,
                                       int n);

}  // namespace trajectory_builder
}  // namespace behav3d