#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <thread>
#include <tuple>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

// High-level planning block: move from current pose to a new pose using IK + joint-space planning
void pose2pose(const rclcpp::Node::SharedPtr& node)
{
  const std::string PLANNING_GROUP = "ur_manipulator";
  const std::string EEF_LINK       = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EEF_LINK);

  // Wait for current state to be available (up to 2 seconds)
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
  if (!current_state)
  {
    RCLCPP_ERROR(LOGGER, "Timed out waiting for /joint_states – aborting.");
    return;
  }

  // Define a new pose target based on the current pose + 5 cm offset in X, Y, Z
  geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
  target.position.x += 0.05;
  target.position.y += 0.05;
  target.position.z += 0.05;
  move_group.setPoseTarget(target);

  // Plan and execute motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(plan);
    RCLCPP_INFO(LOGGER, "Motion executed successfully.");
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Planning failed.");
  }
}

/* void cartesian_path(const rclcpp::Node::SharedPtr& node)
{
  const std::string PLANNING_GROUP = "ur_manipulator";
  const std::string EEF_LINK       = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EEF_LINK);

  // Wait for current state to be available (up to 2 seconds)
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(2.0);
  if (!current_state)
  {
    RCLCPP_ERROR(LOGGER, "Timed out waiting for /joint_states – aborting.");
    return;
  }

  // Define a start pose based on the current pose
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose);

  geometry_msgs::msg::Pose target_pose = start_pose;

  target_pose.position.z -= 0.2;
  waypoints.push_back(target_pose);  // down

  target_pose.position.y -= 0.2;
  waypoints.push_back(target_pose);  // right

  target_pose.position.z += 0.2;
  target_pose.position.y += 0.2;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);  // up and left
  //Set speed and accel:
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  // Plan Cartesian path
  const double eef_step = 0.01;  // 1 cm resolution
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

  RCLCPP_INFO(LOGGER, "Cartesian path (%.2f%% achieved)", fraction * 100.0);

  // Uncomment to execute
   move_group.execute(trajectory);
}
 */


 
//------------------------------------------------------------------------------
// Helper: build per‑joint limit maps (velocity & acceleration)
//------------------------------------------------------------------------------
static void build_joint_limit_maps(const rclcpp::Node::SharedPtr&                        node,
                                   const moveit::core::RobotModelConstPtr&               robot_model,
                                   const moveit::core::JointModelGroup*                  jmg,
                                   std::unordered_map<std::string, double>&              vel_limits,
                                   std::unordered_map<std::string, double>&              acc_limits,
                                   double                                                default_acc = 5.0)
{
  for (const std::string& name : jmg->getVariableNames())
  {
    // Velocity from URDF bounds ------------------------------------------------
    const moveit::core::VariableBounds& b = robot_model->getVariableBounds(name);
    vel_limits[name] = b.velocity_bounded_ ? b.max_velocity_ : 1.0; // fallback

    // Acceleration: parameter → else default ----------------------------------
    const std::string param_name = "robot_description_planning.joint_limits." + name + ".max_acceleration";
    double acc = node->get_parameter_or<double>(param_name, default_acc);
    acc_limits[name] = acc;
  }
}

//------------------------------------------------------------------------------
// Compute and execute one Cartesian segment with its own speed factor
//------------------------------------------------------------------------------
static bool execute_segment(moveit::planning_interface::MoveGroupInterface& move_group,
                           const geometry_msgs::msg::Pose&                   start,
                           const geometry_msgs::msg::Pose&                   goal,
                           double                                             vel_scale,
                           const std::unordered_map<std::string, double>&     vel_limits,
                           const std::unordered_map<std::string, double>&     acc_limits)
{
  constexpr char PLANNING_GROUP[] = "ur_manipulator";
  moveit_msgs::msg::RobotTrajectory traj_msg;
  std::vector<geometry_msgs::msg::Pose> waypoints { start, goal };
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(
          waypoints,            // std::vector<Pose>
          eef_step,             // resolución lineal
          traj_msg,             // trayectoria de salida
          /*avoid_collisions=*/false);   // opcional (por defecto true)
  if (fraction < 0.999)
  {
    RCLCPP_ERROR(LOGGER, "Cartesian segment incomplete (%.1f%%) – skipping", fraction*100.0);
    return false;
  }

  robot_trajectory::RobotTrajectory traj(move_group.getRobotModel(), PLANNING_GROUP);
  traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), traj_msg);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(traj, vel_limits, acc_limits, vel_scale, vel_scale))
  {
    RCLCPP_ERROR(LOGGER, "TOTG failed on segment");
    return false;
  }

  moveit_msgs::msg::RobotTrajectory exec_traj;
  traj.getRobotTrajectoryMsg(exec_traj);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = exec_traj;
  move_group.execute(plan);
  return true;
}

//------------------------------------------------------------------------------
// Main routine: build three segments with individual speeds
//------------------------------------------------------------------------------
void cartesian_path(const rclcpp::Node::SharedPtr& node)
{
  constexpr char PLANNING_GROUP[] = "ur_manipulator";
  constexpr char EEF_LINK[]       = "tool0";

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setEndEffectorLink(EEF_LINK);

  // Wait for state
  if (!move_group.getCurrentState(2.0))
  {
    RCLCPP_ERROR(LOGGER, "No joint state received – aborting");
    return;
  }

  // ---------------- Waypoint & speed list ------------------------------------
  struct Segment { geometry_msgs::msg::Pose target; double speed; };
  std::vector<Segment> segments;
  geometry_msgs::msg::Pose p = move_group.getCurrentPose().pose;

  // Down – slow 0.05
  p.position.z -= 0.2; segments.push_back({p, 0.01});
  // Right – fast 0.20
  p.position.y -= 0.2; segments.push_back({p, 0.20});
  // Up‑left – medium 0.15
  p.position.z += 0.2; p.position.y += 0.2; p.position.x -= 0.0; segments.push_back({p, 0.99});

  // Build limit maps once ------------------------------------------------------
  std::unordered_map<std::string,double> vel_limits, acc_limits;
  build_joint_limit_maps(node, move_group.getRobotModel(), move_group.getRobotModel()->getJointModelGroup(PLANNING_GROUP),
                         vel_limits, acc_limits);

  geometry_msgs::msg::Pose start = move_group.getCurrentPose().pose;
  const double max_speed = 1.00; // reference (fastest) speed for scaling

  for (const auto& seg : segments)
  {
    double scale = seg.speed / max_speed;            // normalize to 0‑1
    scale = std::clamp(scale, 0.05, 1.0);            // safety clamp
    RCLCPP_INFO(LOGGER, "Executing segment at %.0f%% of limits", scale*100.0);
    if (!execute_segment(move_group, start, seg.target, scale, vel_limits, acc_limits))
      return;
    start = seg.target; // next segment starts here
  }

  RCLCPP_INFO(LOGGER, "All segments executed");
}

int main(int argc, char** argv)
{
  // Initialize ROS and create the node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_opts;
  node_opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur10e_move_group_interface", node_opts);

  // Spin in a background thread to keep the node alive for planning and state updates
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // ✳️ Call one of the motion test blocks below
  //pose2pose(node);
   cartesian_path(node);
  // constrained_motion(node);
  // force_tracking_test(node);
  // etc.

  rclcpp::shutdown();
  return 0;
}