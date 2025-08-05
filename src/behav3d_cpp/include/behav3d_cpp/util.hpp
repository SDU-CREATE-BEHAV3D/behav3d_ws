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
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace behav3d
{
    namespace util
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

    } // namespace util
} // namespace behav3d