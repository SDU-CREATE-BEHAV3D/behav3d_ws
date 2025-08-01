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

        // ---------------------------------------------------------------------------
        // Pose primitives
        // ---------------------------------------------------------------------------
        /// Create a stamped pose from explicit components.
        PoseStamped poseStamped(double x, double y, double z,
                                double qx, double qy, double qz, double qw,
                                const std::string &frame);

        // ---------------------------------------------------------------------------
        // Canonical world‑frame helper constructors
        // ---------------------------------------------------------------------------
        PoseStamped worldXY(double x, double y, double z,
                            const std::string &frame = "world");

        PoseStamped worldXZ(double x, double z, double y,
                            const std::string &frame = "world");

        PoseStamped worldYZ(double y, double z, double x,
                            const std::string &frame = "world");

        /// 180° roll about camera X axis (i.e. flip optical frame).
        PoseStamped flipTarget(const PoseStamped &in);

        // ---------------------------------------------------------------------------
        // Conversions
        // ---------------------------------------------------------------------------
        /// PoseStamped → Eigen::Isometry3d
        Eigen::Isometry3d toIso(const PoseStamped &p);

        /// Eigen::Isometry3d → PoseStamped (uses @p frame for header.frame_id)
        PoseStamped fromIso(const Eigen::Isometry3d &iso,
                            const std::string &frame);

        // ---------------------------------------------------------------------------
        // Transformations in the same coordinate frame
        // ---------------------------------------------------------------------------
        PoseStamped translate(const PoseStamped &in,
                              const Eigen::Vector3d &d);

        /// Rotate by extrinsic XYZ Euler angles (roll‑pitch‑yaw).
        /// If @p degrees == true, input is interpreted in degrees.
        PoseStamped rotateEuler(const PoseStamped &in,
                                const Eigen::Vector3d &rpy,
                                bool degrees = false);

        /// Concatenate a relative transform (translation + rotation) on the right.
        PoseStamped transformRel(const PoseStamped &in,
                                 const Eigen::Vector3d &t,
                                 const Eigen::Quaterniond &q);

        // ---------------------------------------------------------------------------
        // Coordinate‑frame utilities
        // ---------------------------------------------------------------------------
        /// Embed @p local (expressed in its own local frame) inside @p basis.
        PoseStamped changeBasis(const PoseStamped &basis,
                                const PoseStamped &local);

        /// Express @p pose originally in @p src.frame ← world → @p dst.frame
        PoseStamped rebase(const PoseStamped &pose,
                           const PoseStamped &src_frame,
                           const PoseStamped &dst_frame);

        // Axis helpers (unit vectors expressed in pose frame but returned in world)
        Eigen::Vector3d xAxis(const PoseStamped &p);
        Eigen::Vector3d yAxis(const PoseStamped &p);
        Eigen::Vector3d zAxis(const PoseStamped &p);

        /// Re‑orient pose so its +X axis aligns with @p new_x (keeps position).
        PoseStamped alignTarget(const PoseStamped &in,
                                const Eigen::Vector3d &new_x);

        /// Adjust pose so its +Z axis aligns with @p new_normal (keeps position).
        PoseStamped adjustTarget(const PoseStamped &in,
                                 const Eigen::Vector3d &new_normal);

        /// Axis‑angle decomposition of pose orientation
        std::pair<Eigen::Vector3d, double> axisAngle(const PoseStamped &p);

        /// Linear interpolation between two poses (t ∈ [0,1])
        PoseStamped poseBetween(const PoseStamped &A,
                                const PoseStamped &B,
                                double t);

        /// Mirror pose across the plane defined by normal @p n passing through @p p0.
        PoseStamped mirrorAcrossPlane(const PoseStamped &pose,
                                      const Eigen::Vector3d &n,
                                      const Eigen::Vector3d &p0);

        /// Arithmetic mean (average) of a collection of poses.
        PoseStamped average(const std::vector<PoseStamped> &poses);

    } // namespace target_builder
} // namespace behav3d