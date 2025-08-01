#include "behav3d_cpp/motion_visualizer.hpp"

namespace behav3d
{

MotionVisualizer::MotionVisualizer(moveit_visual_tools::MoveItVisualToolsPtr vt,
                                   const moveit::core::JointModelGroup*      jmg,
                                   const moveit::core::LinkModel*            tip_link)
  : vt_{std::move(vt)}, jmg_{jmg}, tip_{tip_link}
{
  // nothing else – no publishers here
}

/* -------------------------------------------------------------------------- */
/*  empty stubs – start filling them as you port logic from Python            */
/* -------------------------------------------------------------------------- */

void MotionVisualizer::publishGhost(const moveit_msgs::msg::RobotTrajectory& traj)
{
  // e.g.:
  // vt_->publishTrajectoryPath(traj, jmg_);
  // vt_->trigger();
}

void MotionVisualizer::publishTrail(const moveit_msgs::msg::RobotTrajectory& traj)
{
  // e.g. vt_->publishTrajectoryLine(traj, tip_, jmg_);
}

void MotionVisualizer::publishTargetPose(const geometry_msgs::msg::PoseStamped& pose)
{
  // e.g. vt_->publishAxisLabeled(pose.pose, "target");
}

}  // namespace behav3d
