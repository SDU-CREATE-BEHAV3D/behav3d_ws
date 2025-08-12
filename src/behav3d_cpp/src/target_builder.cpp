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

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "behav3d_cpp/util.hpp"

namespace behav3d::target_builder
{

    using behav3d::util::deg2rad;
    using behav3d::util::fromRPY;
    using geometry_msgs::msg::PoseStamped;

    // Build a timestamped PoseStamped with explicit position & quaternion
    PoseStamped poseStamped(double x, double y, double z,
                            double qx, double qy, double qz, double qw,
                            const std::string &frame)
    {
        PoseStamped ps;
        ps.header.frame_id = frame;
        ps.header.stamp = rclcpp::Clock().now();
        ps.pose.position.x = x;
        ps.pose.position.y = y;
        ps.pose.position.z = z;
        ps.pose.orientation.x = qx;
        ps.pose.orientation.y = qy;
        ps.pose.orientation.z = qz;
        ps.pose.orientation.w = qw;
        return ps;
    }

    PoseStamped worldXY(double x, double y, double z,
                        const std::string &frame)
    {
        tf2::Quaternion q(0, 0, 0, 1);
        return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
    }

    PoseStamped worldXZ(double x, double z, double y,
                        const std::string &frame)
    {
        tf2::Quaternion q(0.7071067811865475, 0, 0, 0.7071067811865475);
        return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
    }

    PoseStamped worldYZ(double y, double z, double x,
                        const std::string &frame)
    {
        tf2::Quaternion q(0.5, 0.5, 0.5, 0.5);
        return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
    }

    // Convert PoseStamped → Eigen::Isometry3d  (pose → matrix)
    Eigen::Isometry3d toIso(const PoseStamped &p)
    {
        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.translation() = Eigen::Vector3d(p.pose.position.x,
                                            p.pose.position.y,
                                            p.pose.position.z);
        Eigen::Quaterniond q(p.pose.orientation.w,
                             p.pose.orientation.x,
                             p.pose.orientation.y,
                             p.pose.orientation.z);
        iso.linear() = q.toRotationMatrix();
        return iso;
    }

    // Convert Eigen::Isometry3d → PoseStamped  (matrix → pose)
    PoseStamped fromIso(const Eigen::Isometry3d &iso,
                        const std::string &frame)
    {
        PoseStamped ps;
        ps.header.frame_id = frame;
        ps.header.stamp = rclcpp::Clock().now();
        ps.pose.position.x = iso.translation().x();
        ps.pose.position.y = iso.translation().y();
        ps.pose.position.z = iso.translation().z();

        Eigen::Quaterniond q(iso.rotation());
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        return ps;
    }

    // Translate pose by vector d in its own frame
    PoseStamped translate(const PoseStamped &in,
                          const Eigen::Vector3d &d)
    {
        Eigen::Isometry3d iso = toIso(in);
        iso.translate(d);
        return fromIso(iso, in.header.frame_id);
    }

    // Rotate pose by RPY (degrees if `degrees==true`)
    PoseStamped rotateEuler(const PoseStamped &in,
                            const Eigen::Vector3d &rpy,
                            bool degrees)
    {
        Eigen::Isometry3d iso = toIso(in);
        iso.rotate(fromRPY(rpy, degrees));
        return fromIso(iso, in.header.frame_id);
    }

    // Apply relative translation & rotation to pose
    PoseStamped transformRel(const PoseStamped &in,
                             const Eigen::Vector3d &t,
                             const Eigen::Quaterniond &q)
    {
        Eigen::Isometry3d iso = toIso(in);
        iso.translate(t);
        iso.rotate(q);
        return fromIso(iso, in.header.frame_id);
    }

    // change-basis: embed a pose defined in a local frame into another pose's frame
    PoseStamped changeBasis(const PoseStamped &basis,
                            const PoseStamped &local)
    {
        Eigen::Isometry3d iso_basis = toIso(basis); // basis → world
        Eigen::Isometry3d iso_local = toIso(local); // pose  → basis
        Eigen::Isometry3d iso_world = iso_basis * iso_local;
        return fromIso(iso_world, basis.header.frame_id);
    }

    // rebase: transform a pose from one frame to another
    PoseStamped rebase(const PoseStamped &pose,
                       const PoseStamped &src,
                       const PoseStamped &dst)
    {
        // src.header.frame_id → world
        Eigen::Isometry3d iso_src = toIso(src);
        // pose in src frame
        Eigen::Isometry3d iso_pose = toIso(pose);
        // world pose
        Eigen::Isometry3d iso_world = iso_src * iso_pose;
        // dst.header.frame_id → world
        Eigen::Isometry3d iso_dst = toIso(dst);
        // express world in dst frame
        Eigen::Isometry3d iso_new = iso_dst.inverse() * iso_world;
        return fromIso(iso_new, dst.header.frame_id);
    }

    // axis helpers
    static Eigen::Vector3d axis(const PoseStamped &p, int idx)
    {
        tf2::Quaternion q(p.pose.orientation.x,
                          p.pose.orientation.y,
                          p.pose.orientation.z,
                          p.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        tf2::Vector3 v = m.getColumn(idx);
        return {v.x(), v.y(), v.z()};
    }

    Eigen::Vector3d xAxis(const PoseStamped &p) { return axis(p, 0); }
    Eigen::Vector3d yAxis(const PoseStamped &p) { return axis(p, 1); }
    Eigen::Vector3d zAxis(const PoseStamped &p) { return axis(p, 2); }

    // Pure roll: align +X with desired axis, keep +Z fixed
    PoseStamped alignTarget(const PoseStamped &in,
                            const Eigen::Vector3d &desired_x)
    {
        // Keep current +Z so the operation is a *pure roll*
        Eigen::Isometry3d iso = toIso(in);

        const Eigen::Vector3d z = iso.linear().col(2).normalized();

        // Project desired X into the plane orthogonal to Z
        Eigen::Vector3d x_proj = desired_x - desired_x.dot(z) * z;

        // If projection is too small (desired_x ‖ Z) choose a fallback axis
        if (x_proj.squaredNorm() < 1e-10)
        {
            Eigen::Vector3d fallback = iso.linear().col(1); // camera +Y
            x_proj = fallback - fallback.dot(z) * z;
        }

        const Eigen::Vector3d x = x_proj.normalized();
        const Eigen::Vector3d y = z.cross(x).normalized(); // completes right‑handed basis

        iso.linear().col(0) = x;
        iso.linear().col(1) = y;
        iso.linear().col(2) = z; // unchanged

        return fromIso(iso, in.header.frame_id);
    }

    // Re-orient pose so +Z = new_normal, keep basis orthonormal
    PoseStamped adjustTarget(const PoseStamped &in,
                             const Eigen::Vector3d &new_normal)
    {
        Eigen::Isometry3d iso = toIso(in);

        const Eigen::Vector3d z = new_normal.normalized();
        Eigen::Vector3d x = iso.linear().col(0);

        // If old X is almost parallel to new Z, pick a safe world axis
        if (std::abs(x.dot(z)) > 0.95)
        {
            x = std::abs(z.dot(Eigen::Vector3d::UnitX())) < 0.8
                    ? Eigen::Vector3d::UnitX()
                    : Eigen::Vector3d::UnitY();
        }

        Eigen::Vector3d y = z.cross(x).normalized();
        x = y.cross(z); // ensure orthonormality

        iso.linear().col(0) = x;
        iso.linear().col(1) = y;
        iso.linear().col(2) = z;

        return fromIso(iso, in.header.frame_id);
    }

    // Return orientation as (axis, angle)
    std::pair<Eigen::Vector3d, double> axisAngle(const PoseStamped &p)
    {
        Eigen::Quaterniond q(p.pose.orientation.w,
                             p.pose.orientation.x,
                             p.pose.orientation.y,
                             p.pose.orientation.z);
        Eigen::AngleAxisd aa(q);
        return {aa.axis(), aa.angle()};
    }

    // Linear/SLERP blend between poses A and B (t∈[0,1])
    PoseStamped poseBetween(const PoseStamped &A,
                            const PoseStamped &B,
                            double t)
    {
        Eigen::Isometry3d a = toIso(A);
        Eigen::Isometry3d b = toIso(B);

        Eigen::Vector3d p = a.translation() * (1.0 - t) + b.translation() * t;
        Eigen::Quaterniond qa(a.rotation()), qb(b.rotation());
        Eigen::Quaterniond q = qa.slerp(t, qb);

        Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
        iso.linear() = q.toRotationMatrix();
        iso.translation() = p;

        return fromIso(iso, A.header.frame_id);
    }

    // Mirror pose across plane defined by point p0 and normal n
    PoseStamped mirrorAcrossPlane(const PoseStamped &pose,
                                  const Eigen::Vector3d &n,
                                  const Eigen::Vector3d &p0)
    {
        const Eigen::Vector3d n_n = n.normalized();

        Eigen::Vector3d pos(pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z);
        Eigen::Vector3d mirrored_pos =
            pos - 2.0 * (pos - p0).dot(n_n) * n_n;

        Eigen::Quaterniond q(pose.pose.orientation.w,
                             pose.pose.orientation.x,
                             pose.pose.orientation.y,
                             pose.pose.orientation.z);

        const Eigen::Quaterniond reflect(Eigen::AngleAxisd(M_PI, n_n));
        Eigen::Quaterniond mirrored_q = reflect * q;

        return poseStamped(mirrored_pos.x(), mirrored_pos.y(), mirrored_pos.z(),
                           mirrored_q.x(), mirrored_q.y(), mirrored_q.z(), mirrored_q.w(),
                           pose.header.frame_id);
    }

    // Average a vector of poses (mean position, hemiconstrained mean quaternion)
    PoseStamped average(const std::vector<PoseStamped> &poses)
    {
        if (poses.empty())
            return PoseStamped{};

        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        Eigen::Quaterniond accum(0, 0, 0, 0);

        for (const auto &p : poses)
        {
            c += Eigen::Vector3d(p.pose.position.x,
                                 p.pose.position.y,
                                 p.pose.position.z);
            Eigen::Quaterniond q(p.pose.orientation.w,
                                 p.pose.orientation.x,
                                 p.pose.orientation.y,
                                 p.pose.orientation.z);
            if (q.w() < 0)
                q.coeffs() = -q.coeffs(); // hemiconstraint
            accum.coeffs() += q.coeffs();
        }

        c /= poses.size();
        accum.normalize();

        return poseStamped(c.x(), c.y(), c.z(),
                           accum.x(), accum.y(), accum.z(), accum.w(),
                           poses.front().header.frame_id);
    }

    /// Move the pose along its *local* +Z axis by `offset_dist` metres.
    PoseStamped offsetTarget(const PoseStamped &in,
                             double offset_dist)
    {
        // `translate` already interprets the vector in the pose’s own frame.
        return translate(in, Eigen::Vector3d(0.0, 0.0, offset_dist));
    }

    /// Replace the position (x, y, z) while preserving orientation.
    PoseStamped setTargetOrigin(const PoseStamped &in,
                                double x, double y, double z)
    {
        PoseStamped out = in; // copy orientation, header, etc.
        out.pose.position.x = x;
        out.pose.position.y = y;
        out.pose.position.z = z;
        out.header.stamp = rclcpp::Clock().now();
        return out;
    }

    /// Optionally flip the local X and/or Y axes; recompute +Z to keep a
    /// right‑handed, orthonormal basis.
    PoseStamped flipTargetAxes(const PoseStamped &in,
                               bool flip_x,
                               bool flip_y)
    {
        Eigen::Isometry3d iso = toIso(in);
        Eigen::Matrix3d R = iso.rotation();

        if (flip_x)
            R.col(0) *= -1.0;
        if (flip_y)
            R.col(1) *= -1.0;

        Eigen::Vector3d x = R.col(0).normalized();
        Eigen::Vector3d y = R.col(1).normalized();
        Eigen::Vector3d z = x.cross(y).normalized();

        // Ensure a right‑handed frame (determinant > 0)
        if (x.cross(y).dot(z) < 0.0)
            z *= -1.0;

        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        iso.linear() = R;

        return fromIso(iso, in.header.frame_id);
    }

    /// Swap the local X and Y axes; adjust Z to restore orthonormality.
    PoseStamped swapTargetAxes(const PoseStamped &in)
    {
        Eigen::Isometry3d iso = toIso(in);
        Eigen::Matrix3d R = iso.rotation();

        R.col(0).swap(R.col(1)); // X ↔ Y

        Eigen::Vector3d x = R.col(0).normalized();
        Eigen::Vector3d y = R.col(1).normalized();
        Eigen::Vector3d z = x.cross(y).normalized();

        // Maintain right‑handed orientation.
        if (x.cross(y).dot(z) < 0.0)
            z *= -1.0;

        R.col(0) = x;
        R.col(1) = y;
        R.col(2) = z;
        iso.linear() = R;

        return fromIso(iso, in.header.frame_id);
    }

} // namespace behav3d::target_builder
