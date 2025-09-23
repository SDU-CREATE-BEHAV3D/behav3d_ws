#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "behav3d_interfaces/srv/plan_with_move_it.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MotionBridge : public rclcpp::Node
{
public:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;

  MotionBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                 .automatically_declare_parameters_from_overrides(true))
  : rclcpp::Node("motion_bridge_node", options)
  {
    // Params (with sane defaults)
    exec_mode_ = this->declare_parameter<std::string>("exec_mode", "move_group"); // or "controller_action"
    controller_action_name_ = this->declare_parameter<std::string>(
        "controller_action_name",
        "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    viz_enabled_ = this->declare_parameter<bool>("viz_enabled", true);

    // Latched QoS for trajectories & markers
    rclcpp::QoS latched(1); latched.transient_local().reliable();

    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/move_group/display_planned_path", latched);
    last_plan_pub_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>(
        "/behav3d/last_plan", latched);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/behav3d/markers/eef_path", latched);

    // Service
    service_ = this->create_service<behav3d_interfaces::srv::PlanWithMoveIt>(
        "/behav3d/plan_with_moveit",
        std::bind(&MotionBridge::planWithMoveItCallback, this, _1, _2));

    // Action client (only used if exec_mode == controller_action)
    fjt_client_ = rclcpp_action::create_client<FJT>(this, controller_action_name_);

    RCLCPP_INFO(this->get_logger(),
      "Motion bridge ready: /behav3d/plan_with_moveit (exec_mode=%s, controller_action=%s)",
      exec_mode_.c_str(), controller_action_name_.c_str());
  }

private:
  void planWithMoveItCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanWithMoveIt::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanWithMoveIt::Response> res)
  {
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), req->group_name);

    if (!req->pipeline_id.empty()) {
      move_group.setPlannerId(req->pipeline_id);
    }

    const double vel = std::clamp<double>(req->velocity_scale, 0.0, 1.0);
    const double acc = std::clamp<double>(req->accel_scale, 0.0, 1.0);
    move_group.setMaxVelocityScalingFactor(vel);
    move_group.setMaxAccelerationScalingFactor(acc);

    // Target
    if (!req->named_target.empty()) {
      move_group.setNamedTarget(req->named_target);
    } else {
      auto goal = req->pose;
      if (goal.header.frame_id.empty()) goal.header.frame_id = move_group.getPlanningFrame();
      const std::string eef = req->eef_link.empty() ? "" : req->eef_link;
      move_group.setPoseTarget(goal, eef);
    }

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = move_group.plan(plan);
    if (!code) {
      RCLCPP_ERROR(this->get_logger(), "Planning failed (error code %d)", code.val);
      res->success = false;
      res->moveit_error_code = code.val;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Motion plan was computed successfully");

    // Time-param
    auto robot_model = move_group.getRobotModel();
    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);

    moveit::core::RobotState start_state(robot_model);
    moveit::core::robotStateMsgToRobotState(plan.start_state, start_state);
    rt.setRobotTrajectoryMsg(start_state, plan.trajectory);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    (void)totg.computeTimeStamps(rt, vel, acc);

    // Back to msg
    moveit_msgs::msg::RobotTrajectory traj_msg;
    rt.getRobotTrajectoryMsg(traj_msg);

    // Totals
    double total_time_sec = 0.0;
    if (!traj_msg.joint_trajectory.points.empty()) {
      const auto &last = traj_msg.joint_trajectory.points.back();
      total_time_sec = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
    }
    double path_length = 0.0;
    for (size_t i = 1; i < traj_msg.joint_trajectory.points.size(); ++i) {
      const auto &a = traj_msg.joint_trajectory.points[i-1].positions;
      const auto &b = traj_msg.joint_trajectory.points[i].positions;
      double accum = 0.0;
      for (size_t j = 0; j < std::min(a.size(), b.size()); ++j) {
        const double d = b[j] - a[j];
        accum += d * d;
      }
      path_length += std::sqrt(accum);
    }

    // Final pose
    geometry_msgs::msg::PoseStamped final_pose;
    const std::string tip = req->eef_link.empty() ? move_group.getEndEffectorLink() : req->eef_link;
    final_pose.header.frame_id = move_group.getPlanningFrame();
    if (rt.getWayPointCount() > 0) {
      const auto &state = rt.getLastWayPoint();
      const Eigen::Isometry3d T = state.getGlobalLinkTransform(tip);
      final_pose.pose = tf2::toMsg(T);
    }

    // Publish to RViz and our latched topic
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = robot_model->getName();
    disp.trajectory_start = plan.start_state;
    disp.trajectory.push_back(traj_msg);
    display_pub_->publish(disp);
    last_plan_pub_->publish(traj_msg);

    // Optional extra visualization (EEF LINE_STRIP)
    if (viz_enabled_) publishEEFPath(rt, tip, final_pose.header.frame_id);

    // Fill response
    res->success = true;
    res->moveit_error_code = 0;
    res->total_time_sec = total_time_sec;
    res->path_length = path_length;
    res->final_pose = final_pose;

    // Execute (optional)
    if (!req->preview_only) {
      if (exec_mode_ == "controller_action") {
        // Ensure action server exists
        if (!fjt_client_->wait_for_action_server(std::chrono::seconds(2))) {
          RCLCPP_WARN(this->get_logger(), "FJT action server %s not available", controller_action_name_.c_str());
        } else {
          auto goal = FJT::Goal();
          goal.trajectory = traj_msg.joint_trajectory;

          auto send_opts = typename rclcpp_action::Client<FJT>::SendGoalOptions();
          send_opts.result_callback = [this](const GoalHandleFJT::WrappedResult &res){
            if (res.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_WARN(this->get_logger(), "Controller action finished with code %d", (int)res.code);
            }
          };
          fjt_client_->async_send_goal(goal, send_opts);
        }
      } else {
        // MoveGroup execution path (what you already had)
        moveit::planning_interface::MoveGroupInterface::Plan timed_plan = plan;
        timed_plan.trajectory = traj_msg;
        auto exec_code = move_group.execute(timed_plan);
        if (!exec_code) {
          RCLCPP_WARN(this->get_logger(), "Execution failed (code=%d)", exec_code.val);
        }
      }
    }

    move_group.clearPoseTargets();
  }

  void publishEEFPath(const robot_trajectory::RobotTrajectory& rt,
                      const std::string& tip_link,
                      const std::string& frame_id)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = frame_id;
    m.ns = "eef_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.004; // line width
    m.color.a = 1.0;
    m.color.r = 0.1f; m.color.g = 0.8f; m.color.b = 0.2f;

    const size_t N = rt.getWayPointCount();
    m.points.reserve(N);
    for (size_t i = 0; i < N; ++i) {
      const auto &state = rt.getWayPoint(i);
      const auto T = state.getGlobalLinkTransform(tip_link);
      geometry_msgs::msg::Point p;
      p.x = T.translation().x();
      p.y = T.translation().y();
      p.z = T.translation().z();
      m.points.push_back(p);
    }
    marker_pub_->publish(m);
  }

private:
  std::string exec_mode_;
  std::string controller_action_name_;
  bool viz_enabled_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr last_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Service<behav3d_interfaces::srv::PlanWithMoveIt>::SharedPtr service_;
  rclcpp_action::Client<FJT>::SharedPtr fjt_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
