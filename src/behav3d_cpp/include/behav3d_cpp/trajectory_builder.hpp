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

#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace behav3d
{
    namespace trajectory_builder
    {

        using geometry_msgs::msg::PoseStamped;

       // Place n camera poses on a spherical cap of radius r around tgt
        //
        // 1. Sample directions on a unit sphere in the target frame (Fibonacci order).
        // 2. Build each pose locally so +Z looks at the target and +X aligns with it.
        // 3. Scale positions by `r` and transform once to world with changeBasis().
        std::vector<PoseStamped> fibonacciSphericalCap(const PoseStamped &tgt,
                                                       double r,
                                                       double cap,
                                                       int n);

        // Generate a zig‑zag raster sweep in tgt’s XY plane at z_off
        std::vector<PoseStamped> sweepZigzag(const PoseStamped &tgt,
                                             double width, double height, double z_off,
                                             int nx, int ny,
                                             bool row_major = false);

        // Add uncorrelated Gaussian translation/rotation noise to each pose
        std::vector<PoseStamped> addJitter(const std::vector<PoseStamped> &poses,
                                           double trans_std,
                                           double rot_std_deg,
                                           unsigned seed = 42U);

        // Insert `steps` evenly‑spaced SLERP/lerp blends between consecutive key poses
        std::vector<PoseStamped> blend(const std::vector<PoseStamped> &poses,
                                       size_t steps);

        // Distribute poses uniformly on a sphere shell of given radius (Fibonacci)
        std::vector<PoseStamped> uniformSphere(const PoseStamped &center,
                                               double radius,
                                               int n);

    } // namespace trajectory_builder
} // namespace behav3d