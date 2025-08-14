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
// TODO: Preffered Elbow Pose

#include <future>
#include <vector>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/joint_limits.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include "behav3d_cpp/motion_controller.hpp"

#define PMC_DEBUG(node, fmt, ...) RCLCPP_DEBUG((node)->get_logger(), "[PMC] " fmt, ##__VA_ARGS__)
#define PMC_INFO(node, fmt, ...) RCLCPP_INFO((node)->get_logger(), "[PMC] " fmt, ##__VA_ARGS__)
#define PMC_WARN(node, fmt, ...) RCLCPP_WARN((node)->get_logger(), "[PMC] " fmt, ##__VA_ARGS__)
#define PMC_ERROR(node, fmt, ...) RCLCPP_ERROR((node)->get_logger(), "[PMC] " fmt, ##__VA_ARGS__)

#define TO_STR(x) #x

namespace behav3d::motion_controller
{

  const std::string &PilzMotionController::getRootLink() const { return root_link_; }
  const std::string &PilzMotionController::getEefLink() const { return eef_link_; }

  std::string PilzMotionController::planningFrame() const
  {
    return move_group_.getPlanningFrame();
  }

  geometry_msgs::msg::PoseStamped PilzMotionController::getRelativePose(const std::string &from_link,
                                                                        const std::string &to_link) const
  {
    const auto &model = move_group_.getRobotModel();
    auto state = move_group_.getCurrentState();
    const std::string pf = planningFrame();

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = this->now();

    const bool from_ok = (model->getLinkModel(from_link) != nullptr);
    const bool to_ok = (model->getLinkModel(to_link) != nullptr);

    // Resolve target for `to_link`; fall back to EEF if unknown
    std::string target_to = to_ok ? to_link : eef_link_;
    if (!to_ok)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "getRelativePose: to_link '%s' not found; using eef_link '%s'",
                   to_link.c_str(), eef_link_.c_str());
    }

    const Eigen::Isometry3d &world_T_to = state->getGlobalLinkTransform(target_to);

    // If from_link is unknown, return in planning frame
    if (!from_ok)
    {
      RCLCPP_WARN(this->get_logger(),
                  "getRelativePose: from_link '%s' not found; returning pose in planning frame '%s'",
                  from_link.c_str(), pf.c_str());
      ps.header.frame_id = pf;
      ps.pose = tf2::toMsg(world_T_to);
      return ps;
    }

    const Eigen::Isometry3d &world_T_from = state->getGlobalLinkTransform(from_link);
    Eigen::Isometry3d from_T_to = world_T_from.inverse() * world_T_to;

    ps.header.frame_id = from_link;
    ps.pose = tf2::toMsg(from_T_to);
    return ps;
  }

  // Fully-parameterized constructor (NodeOptions)
  PilzMotionController::PilzMotionController(const rclcpp::NodeOptions &options)
      : Node("pilz_motion_controller_cpp", options),
        root_link_(this->declare_parameter<std::string>("root_link", "world")),
        eef_link_(this->declare_parameter<std::string>("eef_link", "femto__depth_optical_frame")),
        move_group_(std::shared_ptr<rclcpp::Node>(this, [](auto *) {}),
                    this->declare_parameter<std::string>("group", "ur_arm"))
  {
    const bool debug = this->declare_parameter<bool>("debug", false);
    if (debug)
    {
      this->get_logger().set_level(rclcpp::Logger::Level::Debug);
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "PilzMotionController (options) initialized: group=%s, root_link=%s, eef_link=%s, debug=%s",
                 this->get_parameter("group").as_string().c_str(),
                 root_link_.c_str(), eef_link_.c_str(),
                 debug ? "true" : "false");

    // Parameters with sane defaults, overridable from launch/CLI
    const double vel_scale = this->declare_parameter<double>("max_velocity_scale", 0.5);
    const double acc_scale = this->declare_parameter<double>("max_accel_scale", 0.5);
    const std::string pipeline = this->declare_parameter<std::string>("planning_pipeline", "pilz_industrial_motion_planner");

    move_group_.setPoseReferenceFrame(root_link_);
    move_group_.setEndEffectorLink(eef_link_);
    move_group_.setMaxVelocityScalingFactor(vel_scale);
    move_group_.setMaxAccelerationScalingFactor(acc_scale);
    move_group_.setPlanningPipelineId(pipeline);

    sequence_client_ = rclcpp_action::create_client<MoveGroupSequence>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/sequence_move_group");

    if (!sequence_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PilzMotionController] Waiting for /sequence_move_group action server…");
    }
  }

  // Simple accessors

  // Plan a PTP or LIN motion to a single target pose
  RobotTrajectoryPtr
  PilzMotionController::planTarget(const geometry_msgs::msg::PoseStamped &target,
                                   const std::string &motion_type,
                                   std::optional<double> vel_scale,
                                   std::optional<double> acc_scale)
  {
    move_group_.clearPoseTargets();
    move_group_.clearPathConstraints();
    move_group_.setJointValueTarget(std::vector<double>());

    // Validate motion_type
    if (motion_type != "PTP" && motion_type != "LIN")
    {
      RCLCPP_ERROR(this->get_logger(),
                   "planTarget: unsupported motion_type '%s'; must be 'PTP' or 'LIN'",
                   motion_type.c_str());
      return nullptr;
    }

    // Use provided scales if given; otherwise use controller-configured defaults
    const double vs = vel_scale.value_or(move_group_.getMaxVelocityScalingFactor());
    const double as = acc_scale.value_or(move_group_.getMaxAccelerationScalingFactor());
    move_group_.setMaxVelocityScalingFactor(vs);
    move_group_.setMaxAccelerationScalingFactor(as);

    RCLCPP_DEBUG(this->get_logger(),
                 "planTarget called: target=(%.3f,%.3f,%.3f), motion_type=%s, vel_scale=%.3f, acc_scale=%.3f",
                 target.pose.position.x, target.pose.position.y, target.pose.position.z,
                 motion_type.c_str(), vs, as);

    move_group_.setPlannerId(motion_type);
    move_group_.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = move_group_.plan(plan);

    RCLCPP_DEBUG(this->get_logger(),
                 "planTarget(%s): plan returned code=%d", motion_type.c_str(), code.val);

    auto traj = std::make_shared<RobotTrajectory>(
        move_group_.getRobotModel(), move_group_.getName());
    traj->setRobotTrajectoryMsg(*move_group_.getCurrentState(), plan.trajectory);
    if (code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "planTarget(%s): planning failed (code=%d)",
                   motion_type.c_str(), code.val);
      return nullptr;
    }
    return traj;
  }

  // Plan a joint-space PTP motion to given joint vector
  RobotTrajectoryPtr
  PilzMotionController::planJoints(const std::vector<double> &joint_positions,
                                   std::optional<double> vel_scale,
                                   std::optional<double> acc_scale)
  {
    move_group_.clearPoseTargets();
    move_group_.clearPathConstraints();

    const auto *jmg = move_group_.getRobotModel()->getJointModelGroup(move_group_.getName());
    if (joint_positions.size() != jmg->getVariableCount())
    {
      RCLCPP_ERROR(this->get_logger(),
                   "planJoints: incorrect number of joint angles given.");
      return nullptr;
    }

    move_group_.setPlannerId("PTP");
    const double vs = vel_scale.value_or(move_group_.getMaxVelocityScalingFactor());
    const double as = acc_scale.value_or(move_group_.getMaxAccelerationScalingFactor());
    move_group_.setMaxVelocityScalingFactor(vs);
    move_group_.setMaxAccelerationScalingFactor(as);

    move_group_.setJointValueTarget(joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = move_group_.plan(plan);

    RCLCPP_DEBUG(this->get_logger(),
                 "planJoints: plan returned code=%d", code.val);

    if (code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "planJoints: planning failed (code=%d)", code.val);
      return nullptr;
    }

    auto traj = std::make_shared<RobotTrajectory>(
        move_group_.getRobotModel(), move_group_.getName());
    traj->setRobotTrajectoryMsg(*move_group_.getCurrentState(), plan.trajectory);
    return traj;
  }

  // Plan a blended linear sequence through way-points (PILZ MotionSequence API)
  RobotTrajectoryPtr
  PilzMotionController::planSequence(const std::vector<geometry_msgs::msg::PoseStamped> &waypoints,
                                     double blend_radius,
                                     std::optional<double> vel_scale,
                                     std::optional<double> acc_scale,
                                     double pos_tolerance,
                                     double ori_tolerance)
  {
    move_group_.clearPoseTargets();
    move_group_.clearPathConstraints();
    move_group_.setJointValueTarget(std::vector<double>());

    const double vs = vel_scale.value_or(move_group_.getMaxVelocityScalingFactor());
    const double as = acc_scale.value_or(move_group_.getMaxAccelerationScalingFactor());

    RCLCPP_DEBUG(this->get_logger(),
                 "planSequence called: %zu waypoints, vel_scale=%.3f, acc_scale=%.3f, blend_radius=%.3f",
                 waypoints.size(), vs, as, blend_radius);
    if (waypoints.size() < 2)
    {
      RCLCPP_ERROR(this->get_logger(), "planSequence: need ≥2 way-points");
      return nullptr;
    }

    MoveGroupSequence::Goal goal;
    goal.request.items.reserve(waypoints.size());

    for (const auto &ps : waypoints)
    {
      moveit_msgs::msg::MotionPlanRequest req;
      req.pipeline_id = "pilz_industrial_motion_planner";
      req.planner_id = "LIN";
      req.group_name = move_group_.getName();
      req.allowed_planning_time = 10.0;
      req.max_velocity_scaling_factor = vs;
      req.max_acceleration_scaling_factor = as;

      // Build a fully‑specified pose constraint (position + orientation)
      moveit_msgs::msg::Constraints c =
          kinematic_constraints::constructGoalConstraints(eef_link_, ps, pos_tolerance, ori_tolerance);
      req.goal_constraints.push_back(c);

      moveit_msgs::msg::MotionSequenceItem item;
      item.blend_radius = blend_radius;
      item.req = req;
      goal.request.items.push_back(item);
    }
    goal.request.items.back().blend_radius = 0.0;

    RCLCPP_DEBUG(this->get_logger(),
                 "planSequence: dispatching sequence goal with %zu items", goal.request.items.size());

    // Configure planning options for sequence (blending and plan-only)
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = true;

    // Send the sequence goal asynchronously
    auto send_goal_future = sequence_client_->async_send_goal(goal);
    std::future_status status;
    // Wait for goal acceptance
    do
    {
      status = send_goal_future.wait_for(std::chrono::seconds(1));
      if (status == std::future_status::timeout)
      {
        RCLCPP_INFO(this->get_logger(), "[PMC] Waiting for sequence goal acceptance...");
      }
    } while (status != std::future_status::ready);

    auto goal_handle = send_goal_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "[PMC] planSequence: goal rejected");
      return nullptr;
    }

    // Wait for the result asynchronously
    auto result_future = sequence_client_->async_get_result(goal_handle);
    do
    {
      status = result_future.wait_for(std::chrono::seconds(1));
      if (status == std::future_status::timeout)
      {
        RCLCPP_INFO(this->get_logger(), "[PMC] Waiting for sequence result...");
      }
    } while (status != std::future_status::ready);

    auto wrapped_result = result_future.get();
    if (!wrapped_result.result)
    {
      RCLCPP_ERROR(this->get_logger(), "[PMC] planSequence: empty result pointer");
      return nullptr;
    }
    const auto &response = wrapped_result.result->response;

    RCLCPP_DEBUG(this->get_logger(),
                 "planSequence: received %zu trajectory segments",
                 response.planned_trajectories.size());

    if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "planSequence: planning failed (code=%d)",
                   response.error_code.val);
      return nullptr;
    }

    auto traj = std::make_shared<RobotTrajectory>(
        move_group_.getRobotModel(), move_group_.getName());
    // Build a single blended trajectory by chaining segments
    auto current_state = move_group_.getCurrentState();
    // Reference state for the first segment
    auto ref_state = std::make_shared<moveit::core::RobotState>(*current_state);
    traj->setRobotTrajectoryMsg(*ref_state, response.planned_trajectories.front());
    // Update reference state to end of the first segment
    ref_state = traj->getLastWayPointPtr();
    for (size_t i = 1; i < response.planned_trajectories.size(); ++i)
    {
      robot_trajectory::RobotTrajectory seg_traj(
          move_group_.getRobotModel(), move_group_.getName());
      seg_traj.setRobotTrajectoryMsg(*ref_state, response.planned_trajectories[i]);
      // Append without pause to maintain blending
      traj->append(seg_traj, /*dt=*/0.0);
      // Update reference state for the next segment
      ref_state = seg_traj.getLastWayPointPtr();
    }
    return traj;
  }

  // Execute a prepared trajectory, optionally applying TOTG timing
  bool PilzMotionController::executeTrajectory(const RobotTrajectoryPtr &traj,
                                               bool apply_totg)
  {
    RCLCPP_DEBUG(this->get_logger(),
                 "executeTrajectory called: traj=%p, apply_totg=%s",
                 static_cast<const void *>(traj.get()), apply_totg ? "true" : "false");
    if (!traj)
      return false;
    RobotTrajectory processed(*traj);
    if (apply_totg)
    {
      // Time-optimal parameterization using MoveIt joint limits message
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      // Gather per-joint limits into a JointLimits vector
      std::vector<moveit_msgs::msg::JointLimits> joint_limits;
      // Retrieve the joint names from the trajectory's group
      const auto *joint_model_group = processed.getGroup();
      const auto joint_names = joint_model_group->getVariableNames();
      const auto &robot_model = move_group_.getRobotModel();
      for (const auto &jn : joint_names)
      {
        moveit_msgs::msg::JointLimits limits_msg;
        limits_msg.joint_name = jn;
        const auto *jm = robot_model->getJointModel(jn);
        const auto &bounds = jm->getVariableBounds(jn);
        if (bounds.velocity_bounded_)
        {
          limits_msg.has_velocity_limits = true;
          limits_msg.max_velocity = bounds.max_velocity_;
        }
        if (bounds.acceleration_bounded_)
        {
          limits_msg.has_acceleration_limits = true;
          limits_msg.max_acceleration = bounds.max_acceleration_;
        }
        joint_limits.push_back(limits_msg);
      }
      // Use current scaling factors
      double max_vel_scale = move_group_.getMaxVelocityScalingFactor();
      double max_acc_scale = move_group_.getMaxAccelerationScalingFactor();
      bool success = totg.computeTimeStamps(processed, joint_limits, max_vel_scale, max_acc_scale);
      RCLCPP_DEBUG(this->get_logger(),
                   "executeTrajectory: time-optimal parameterization %s",
                   success ? "succeeded" : "failed");
      if (!success)
      {
        RCLCPP_WARN(this->get_logger(), "executeTrajectory: time-optimal parameterization failed");
      }
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    processed.getRobotTrajectoryMsg(plan.trajectory);
    RCLCPP_DEBUG(this->get_logger(), "executeTrajectory: executing plan");
    auto code = move_group_.execute(plan);
    if (code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "executeTrajectory: execution failed (code=%d)", code.val);
      return false;
    }
    return true;
  }

  // Pose of `link` (default: eef_link_) expressed in `root_frame` (default: planning frame, usually "world");
  // falls back to eef_link_ for unknown link and to the planning frame for unknown root_frame.
  geometry_msgs::msg::PoseStamped PilzMotionController::getCurrentPose(const std::string &link, const std::string &root_frame) const
  {
    const auto &model = move_group_.getRobotModel();

    // Resolve target link (default to configured EEF)
    std::string target_link = link.empty() ? eef_link_ : link;
    if (!model->getLinkModel(target_link))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "getCurrentPose: link '%s' not found; using eef_link '%s'",
                   target_link.c_str(), eef_link_.c_str());
      target_link = eef_link_;
    }

    const std::string pf = planningFrame();

    // If no root_frame requested or it equals planning frame, return pose in planning frame
    if (root_frame.empty() || root_frame == pf)
    {
      auto state = move_group_.getCurrentState();
      const Eigen::Isometry3d &world_T_link = state->getGlobalLinkTransform(target_link);
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = pf;
      ps.header.stamp = this->now();
      ps.pose = tf2::toMsg(world_T_link);
      RCLCPP_DEBUG(this->get_logger(),
                   "getCurrentPose[%s @ %s]: (%.3f,%.3f,%.3f)",
                   target_link.c_str(), pf.c_str(),
                   ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
      return ps;
    }

    // If requested root_frame is a known robot link, use helper
    if (model->getLinkModel(root_frame))
    {
      auto ps = getRelativePose(root_frame, target_link);
      RCLCPP_DEBUG(this->get_logger(),
                   "getCurrentPose[%s @ %s]: (%.3f,%.3f,%.3f)",
                   target_link.c_str(), root_frame.c_str(),
                   ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
      return ps;
    }

    // Fallback: unknown root_frame -> planning frame
    RCLCPP_WARN(this->get_logger(),
                "getCurrentPose: root_frame '%s' not found; returning pose in planning frame '%s'",
                root_frame.c_str(), pf.c_str());
    return getRelativePose(pf, target_link); // returns planning-frame pose
  }

  // Current joint state vector
  sensor_msgs::msg::JointState PilzMotionController::getCurrentJointState() const
  {
    sensor_msgs::msg::JointState js;
    auto current_state = move_group_.getCurrentState();
    const auto *jmg = current_state->getJointModelGroup(move_group_.getName());
    std::vector<double> positions;
    current_state->copyJointGroupPositions(jmg, positions);
    js.name = jmg->getVariableNames();
    js.position = positions;
    RCLCPP_DEBUG(this->get_logger(),
                 "getCurrentJointState: joints=%zu", js.name.size());
    return js;
  }

  // Cancel all active sequence goals
  void PilzMotionController::cancelAllGoals()
  {
    auto f = sequence_client_->async_cancel_all_goals();
    RCLCPP_DEBUG(this->get_logger(),
                 "cancelAllGoals: cancellation request sent");
  }

  // Compute IK solution for pose (blocking, timeout in seconds)
  moveit::core::RobotStatePtr
  PilzMotionController::computeIK(
      const geometry_msgs::msg::PoseStamped &pose,
      double timeout) const
  {
    PMC_DEBUG(this, "computeIK: pose=(%.3f,%.3f,%.3f)",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    const auto *jmg = move_group_.getRobotModel()->getJointModelGroup(move_group_.getName());
    auto state = std::make_shared<moveit::core::RobotState>(move_group_.getRobotModel());
    state->setToDefaultValues();

    // Try to solve IK for the desired pose
    bool found = state->setFromIK(jmg, pose.pose, eef_link_, timeout);
    if (!found)
    {
      PMC_DEBUG(this, "computeIK: no IK solution within %.3f s", timeout);
      return nullptr;
    }
    return state;
  }

  // Forward-kinematics for current state
  geometry_msgs::msg::PoseStamped
  PilzMotionController::computeFK(const moveit::core::RobotState &state) const
  {
    PMC_DEBUG(this, "computeFK called");

    geometry_msgs::msg::PoseStamped ps;
    const std::string pf = planningFrame();
    ps.header.frame_id = pf;
    ps.header.stamp = this->now();

    const Eigen::Isometry3d &world_T_eef = state.getGlobalLinkTransform(eef_link_);
    ps.pose = tf2::toMsg(world_T_eef);
    return ps;
  }

  // Quick reachability check via short IK
  bool PilzMotionController::isReachable(
      const geometry_msgs::msg::PoseStamped &pose) const
  {
    PMC_DEBUG(this, "isReachable: checking pose (%.3f,%.3f,%.3f)",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // Use a short IK timeout; reachable if we find a solution
    return static_cast<bool>(computeIK(pose, 0.05));
  }

} // namespace behav3d::motion_controller