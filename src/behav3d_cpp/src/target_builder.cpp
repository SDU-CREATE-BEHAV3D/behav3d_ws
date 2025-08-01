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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "behav3d/math_util.hpp"

namespace behav3d {
namespace target_builder {

using geometry_msgs::msg::PoseStamped;
using behav3d::util::deg2rad;
using behav3d::util::fromRPY;

// -----------------------------------------------------------------------------
// helpers
// -----------------------------------------------------------------------------
static PoseStamped makePose(const Eigen::Isometry3d& iso,
                            const std::string& frame)
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

// -----------------------------------------------------------------------------
// public API
// -----------------------------------------------------------------------------
PoseStamped poseStamped(double x, double y, double z,
                        double qx, double qy, double qz, double qw,
                        const std::string& frame)
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

// -----------------------------------------------------------------------------
// canonical world‑frame poses
// -----------------------------------------------------------------------------
PoseStamped worldXY(double x, double y, double z,
                    const std::string& frame)
{
    // Identity rotation: camera +Z == world +Z
    tf2::Quaternion q(0, 0, 0, 1);
    return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
}

PoseStamped worldXZ(double x, double z, double y,
                    const std::string& frame)
{
    // Identity rotation.  User can re‑orient later if needed.
    tf2::Quaternion q(0, 0, 0, 1);
    return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
}

PoseStamped worldYZ(double y, double z, double x,
                    const std::string& frame)
{
    tf2::Quaternion q(0, 0, 0, 1);
    return poseStamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame);
}

// -----------------------------------------------------------------------------
// roll 180° around camera X axis (optical frame flip)
// -----------------------------------------------------------------------------
PoseStamped flipTarget(const PoseStamped& in)
{
    Eigen::Isometry3d iso = toIso(in);
    iso.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    return fromIso(iso, in.header.frame_id);
}

Eigen::Isometry3d toIso(const PoseStamped& p)
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

PoseStamped fromIso(const Eigen::Isometry3d& iso,
                    const std::string& frame)
{
    return makePose(iso, frame);
}

PoseStamped translate(const PoseStamped& in,
                      const Eigen::Vector3d& d)
{
    Eigen::Isometry3d iso = toIso(in);
    iso.translate(d);
    return fromIso(iso, in.header.frame_id);
}

PoseStamped rotateEuler(const PoseStamped& in,
                        const Eigen::Vector3d& rpy,
                        bool degrees)
{
    Eigen::Isometry3d iso = toIso(in);
    iso.rotate(fromRPY(rpy, degrees));
    return fromIso(iso, in.header.frame_id);
}

PoseStamped transformRel(const PoseStamped& in,
                         const Eigen::Vector3d& t,
                         const Eigen::Quaterniond& q)
{
    Eigen::Isometry3d iso = toIso(in);
    iso.translate(t);
    iso.rotate(q);
    return fromIso(iso, in.header.frame_id);
}

// -----------------------------------------------------------------------------
// change‑basis: embed a pose defined in a local frame into another pose's frame
// -----------------------------------------------------------------------------
PoseStamped changeBasis(const PoseStamped& basis,
                        const PoseStamped& local)
{
    Eigen::Isometry3d iso_basis = toIso(basis);   // basis → world
    Eigen::Isometry3d iso_local = toIso(local);   // pose  → basis
    Eigen::Isometry3d iso_world = iso_basis * iso_local;
    return fromIso(iso_world, basis.header.frame_id);
}

// -----------------------------------------------------------------------------
// rebase: transform a pose from one frame to another
// -----------------------------------------------------------------------------
PoseStamped rebase(const PoseStamped& pose,
                   const PoseStamped& src,
                   const PoseStamped& dst)
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

// -----------------------------------------------------------------------------
// axis helpers
// -----------------------------------------------------------------------------
static Eigen::Vector3d axis(const PoseStamped& p, int idx)
{
    tf2::Quaternion q(p.pose.orientation.x,
                      p.pose.orientation.y,
                      p.pose.orientation.z,
                      p.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    tf2::Vector3 v = m.getColumn(idx);
    return {v.x(), v.y(), v.z()};
}

Eigen::Vector3d xAxis(const PoseStamped& p) { return axis(p, 0); }
Eigen::Vector3d yAxis(const PoseStamped& p) { return axis(p, 1); }
Eigen::Vector3d zAxis(const PoseStamped& p) { return axis(p, 2); }

PoseStamped alignTarget(const PoseStamped& in,
                        const Eigen::Vector3d& new_x)
{
    Eigen::Isometry3d iso = toIso(in);

    Eigen::Vector3d x = new_x.normalized();
    Eigen::Vector3d z = iso.linear().col(2);
    Eigen::Vector3d y = z.cross(x).normalized();
    z = x.cross(y);

    iso.linear().col(0) = x;
    iso.linear().col(1) = y;
    iso.linear().col(2) = z;

    return fromIso(iso, in.header.frame_id);
}

PoseStamped adjustTarget(const PoseStamped& in,
                         const Eigen::Vector3d& new_normal)
{
    Eigen::Isometry3d iso = toIso(in);

    Eigen::Vector3d z = new_normal.normalized();
    Eigen::Vector3d x = iso.linear().col(0);
    Eigen::Vector3d y = z.cross(x).normalized();
    x = y.cross(z);

    iso.linear().col(0) = x;
    iso.linear().col(1) = y;
    iso.linear().col(2) = z;

    return fromIso(iso, in.header.frame_id);
}

std::pair<Eigen::Vector3d, double> axisAngle(const PoseStamped& p)
{
    Eigen::Quaterniond q(p.pose.orientation.w,
                         p.pose.orientation.x,
                         p.pose.orientation.y,
                         p.pose.orientation.z);
    Eigen::AngleAxisd aa(q);
    return { aa.axis(), aa.angle() };
}

PoseStamped poseBetween(const PoseStamped& A,
                        const PoseStamped& B,
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

PoseStamped mirrorAcrossPlane(const PoseStamped& pose,
                              const Eigen::Vector3d& n,
                              const Eigen::Vector3d& p0)
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

PoseStamped average(const std::vector<PoseStamped>& poses)
{
    if (poses.empty()) return PoseStamped{};

    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    Eigen::Quaterniond accum(0, 0, 0, 0);

    for (const auto& p : poses)
    {
        c += Eigen::Vector3d(p.pose.position.x,
                             p.pose.position.y,
                             p.pose.position.z);
        Eigen::Quaterniond q(p.pose.orientation.w,
                             p.pose.orientation.x,
                             p.pose.orientation.y,
                             p.pose.orientation.z);
        if (q.w() < 0) q.coeffs() = -q.coeffs(); // hemiconstraint
        accum.coeffs() += q.coeffs();
    }

    c /= poses.size();
    accum.normalize();

    return poseStamped(c.x(), c.y(), c.z(),
                       accum.x(), accum.y(), accum.z(), accum.w(),
                       poses.front().header.frame_id);
}

} // namespace target_builder
} // namespace behav3d