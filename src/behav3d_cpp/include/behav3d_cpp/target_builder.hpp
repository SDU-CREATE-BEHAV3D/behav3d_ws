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
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace behav3d
{
    namespace target_builder
    {

        using geometry_msgs::msg::PoseStamped;

        // Build a timestamped PoseStamped with explicit position & quaternion
        PoseStamped poseStamped(double x, double y, double z,
                                double qx, double qy, double qz, double qw,
                                const std::string &frame);

        PoseStamped worldXY(double x, double y, double z,
                            const std::string &frame = "world");

        PoseStamped worldXZ(double x, double z, double y,
                            const std::string &frame = "world");

        PoseStamped worldYZ(double y, double z, double x,
                            const std::string &frame = "world");

        // Convert PoseStamped → Eigen::Isometry3d  (pose → matrix)
        Eigen::Isometry3d toIso(const PoseStamped &p);

        // Convert Eigen::Isometry3d → PoseStamped  (matrix → pose)
        PoseStamped fromIso(const Eigen::Isometry3d &iso,
                            const std::string &frame);

        // Translate pose by vector d in its own frame
        PoseStamped translate(const PoseStamped &in,
                              const Eigen::Vector3d &d);

        // Rotate pose by RPY (degrees if `degrees==true`)
        PoseStamped rotateEuler(const PoseStamped &in,
                                const Eigen::Vector3d &rpy,
                                bool degrees = false);

        // Apply relative translation & rotation to pose
        PoseStamped transformRel(const PoseStamped &in,
                                 const Eigen::Vector3d &t,
                                 const Eigen::Quaterniond &q);

        // change-basis: embed a pose defined in a local frame into another pose's frame
        PoseStamped changeBasis(const PoseStamped &basis,
                                const PoseStamped &local);

        // rebase: transform a pose from one frame to another
        PoseStamped rebase(const PoseStamped &pose,
                           const PoseStamped &src_frame,
                           const PoseStamped &dst_frame);

        // Axis helpers (unit vectors expressed in pose frame but returned in world)
        Eigen::Vector3d xAxis(const PoseStamped &p);
        Eigen::Vector3d yAxis(const PoseStamped &p);
        Eigen::Vector3d zAxis(const PoseStamped &p);

        // Pure roll: align +X with desired axis, keep +Z fixed
        PoseStamped alignTarget(const PoseStamped &in,
                                const Eigen::Vector3d &new_x);

        // Re-orient pose so +Z = new_normal, keep basis orthonormal
        PoseStamped adjustTarget(const PoseStamped &in,
                                 const Eigen::Vector3d &new_normal);

        // Return orientation as (axis, angle)
        std::pair<Eigen::Vector3d, double> axisAngle(const PoseStamped &p);

        // Linear/SLERP blend between poses A and B (t∈[0,1])
        PoseStamped poseBetween(const PoseStamped &A,
                                const PoseStamped &B,
                                double t);

        // Mirror pose across plane defined by point p0 and normal n
        PoseStamped mirrorAcrossPlane(const PoseStamped &pose,
                                      const Eigen::Vector3d &n,
                                      const Eigen::Vector3d &p0);

        // Average a vector of poses (mean position, hemiconstrained mean quaternion)
        PoseStamped average(const std::vector<PoseStamped> &poses);

        /// Move the pose along its *local* +Z axis by `offset_dist` metres.
        PoseStamped offsetTarget(const PoseStamped &in,
                                 double offset_dist);

        /// Replace the position (x, y, z) while preserving orientation.
        PoseStamped setTargetOrigin(const PoseStamped &in,
                                    double x, double y, double z);

        /// Optionally flip the local X and/or Y axes; recompute +Z to keep a
        /// right‑handed, orthonormal basis.
        PoseStamped flipTargetAxes(const PoseStamped &in,
                                   bool flip_x,
                                   bool flip_y);

        /// Swap the local X and Y axes; adjust Z to restore orthonormality.
        PoseStamped swapTargetAxes(const PoseStamped &in);

    } // namespace target_builder
} // namespace behav3d