#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <unordered_set>

#include <boost/variant/get.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

// Constraints utils (.hpp to avoid deprecation) + Pilz sequence action
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

#include "behav3d_interfaces/srv/plan_with_move_it.hpp"
#include "behav3d_interfaces/srv/plan_cartesian_path.hpp"
#include "behav3d_interfaces/srv/plan_pilz_lin.hpp"
#include "behav3d_interfaces/srv/plan_pilz_ptp.hpp"
#include "behav3d_interfaces/srv/plan_pilz_sequence.hpp"
#include "behav3d_interfaces/srv/get_link_pose.hpp"
#include "behav3d_interfaces/srv/publish_targets.hpp"
#include "behav3d_interfaces/srv/delete_markers.hpp"
#include "behav3d_interfaces/srv/update_planning_scene_mesh.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

class MotionBridge : public rclcpp::Node
{
public:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FJT>;
  using MGS = moveit_msgs::action::MoveGroupSequence;
  using GoalHandleMGS = rclcpp_action::ClientGoalHandle<MGS>;

  MotionBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
                   .automatically_declare_parameters_from_overrides(true))
      : rclcpp::Node("motion_bridge_node", options)
  {
    // Params
    exec_mode_ = this->declare_parameter<std::string>("exec_mode", "move_group");
    controller_action_name_ = this->declare_parameter<std::string>(
        "controller_action_name",
        "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    viz_enabled_ = this->declare_parameter<bool>("viz_enabled", true);
    flange_link_ = this->declare_parameter<std::string>("flange_link", "ur20_tool0");
    allow_tcp_retime_ = this->declare_parameter<bool>("allow_tcp_retime", true);
    retime_fallback_on_failure_ = this->declare_parameter<bool>("retime_fallback_on_failure", true);
    max_tcp_speed_m_s_ = std::max(
        0.0, this->declare_parameter<double>("max_tcp_speed_m_s", 0.350));
    log_sequence_joint_stats_ = this->declare_parameter<bool>("log_sequence_joint_stats", true);

    // Latched QoS
    rclcpp::QoS latched(1);
    latched.transient_local().reliable();

    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/move_group/display_planned_path", latched);
    last_plan_pub_ = this->create_publisher<moveit_msgs::msg::RobotTrajectory>(
        "/behav3d/last_plan", latched);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/behav3d/markers/eef_path", latched);
    targets_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/behav3d/markers/targets", latched);

    // Services
    
    service_ = this->create_service<behav3d_interfaces::srv::PlanWithMoveIt>(
        "/behav3d/plan_with_moveit",
        std::bind(&MotionBridge::planWithMoveItCallback, this, _1, _2));

    cartesian_srv_ = this->create_service<behav3d_interfaces::srv::PlanCartesianPath>(
        "/behav3d/plan_cartesian_path",
        std::bind(&MotionBridge::planCartesianCallback, this, _1, _2));

    pilz_lin_srv_ = this->create_service<behav3d_interfaces::srv::PlanPilzLin>(
        "/behav3d/plan_pilz_lin",
        std::bind(&MotionBridge::planPilzLinCallback, this, _1, _2));

    pilz_ptp_srv_ = this->create_service<behav3d_interfaces::srv::PlanPilzPtp>(
        "/behav3d/plan_pilz_ptp",
        std::bind(&MotionBridge::planPilzPtpCallback, this, _1, _2));

    // Pilz sequence (LIN/LIN[/CIRC] with blending) via ACTION, using a dedicated helper node
    pilz_seq_srv_ = this->create_service<behav3d_interfaces::srv::PlanPilzSequence>(
        "/behav3d/plan_pilz_sequence",
        std::bind(&MotionBridge::planPilzSequenceCallback, this, _1, _2));

    // Action client for controller execution
    fjt_client_ = rclcpp_action::create_client<FJT>(this, controller_action_name_);

    // Dedicated node for the MoveGroupSequence action client to avoid double-adding this node to an executor
    seq_client_node_ = std::make_shared<rclcpp::Node>("motion_bridge_seq_client_node");
    mgs_client_ = rclcpp_action::create_client<MGS>(seq_client_node_, "sequence_move_group");

    // Get current pose of a link expressed in a base frame
    get_link_pose_srv_ = this->create_service<behav3d_interfaces::srv::GetLinkPose>(
        "/behav3d/get_link_pose",
        std::bind(&MotionBridge::getLinkPoseCallback, this, _1, _2));

    // Publish target markers (axes) and clear markers
    publish_targets_srv_ = this->create_service<behav3d_interfaces::srv::PublishTargets>(
        "/behav3d/publish_targets",
        std::bind(&MotionBridge::publishTargetsCallback, this, _1, _2));

    delete_markers_srv_ = this->create_service<behav3d_interfaces::srv::DeleteMarkers>(
        "/behav3d/delete_markers",
        std::bind(&MotionBridge::deleteMarkersCallback, this, _1, _2));

    update_planning_scene_mesh_srv_ =
        this->create_service<behav3d_interfaces::srv::UpdatePlanningSceneMesh>(
            "/behav3d/update_planning_scene_mesh",
            std::bind(&MotionBridge::updatePlanningSceneMeshCallback, this, _1, _2));

    // Joint state subscription
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::QoS(100),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          last_joint_state_ = *msg;
        });

    RCLCPP_INFO(this->get_logger(),
                "Motion bridge ready: /behav3d/plan_with_moveit, /behav3d/plan_cartesian_path, "
                "/behav3d/plan_pilz_lin, /behav3d/plan_pilz_ptp, /behav3d/plan_pilz_sequence "
                "/behav3d/get_link_pose, /behav3d/publish_targets, /behav3d/delete_markers, "
                "/behav3d/update_planning_scene_mesh "
                "(exec_mode=%s, controller_action=%s, flange_link=%s, allow_tcp_retime=%s, "
                "retime_fallback_on_failure=%s, max_tcp_speed=%.3fmm/s, log_sequence_joint_stats=%s)",
                exec_mode_.c_str(), controller_action_name_.c_str(), flange_link_.c_str(),
                allow_tcp_retime_ ? "true" : "false",
                retime_fallback_on_failure_ ? "true" : "false",
                max_tcp_speed_m_s_ * 1000.0,
                log_sequence_joint_stats_ ? "true" : "false");
  }

private:
  static std::string trimCopy(const std::string& value)
  {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
      return "";
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
  }

  static fs::path normalizePath(const fs::path& path)
  {
    return fs::absolute(path).lexically_normal();
  }

  static fs::path expandUserPath(const std::string& value)
  {
    if (value.empty() || value[0] != '~')
      return fs::path(value);

    if (const char* home = std::getenv("HOME"); home && *home)
    {
      std::string suffix = value.size() > 1 ? value.substr(1) : "";
      while (!suffix.empty() && suffix.front() == '/')
        suffix.erase(suffix.begin());
      return fs::path(home) / suffix;
    }
    return fs::path(value);
  }

  fs::path capturesRoot() const
  {
    if (const char* env_root = std::getenv("BEHAV3D_CAPTURES_ROOT"); env_root && *env_root)
      return normalizePath(fs::path(env_root));

    if (const char* home = std::getenv("HOME"); home && *home)
      return normalizePath(fs::path(home) / "behav3d_ws" / "captures");

    return normalizePath(fs::temp_directory_path() / "behav3d_captures");
  }

  std::optional<fs::path> getLatestSession(const fs::path& captures_root) const
  {
    if (!fs::exists(captures_root) || !fs::is_directory(captures_root))
      return std::nullopt;

    std::optional<fs::path> latest;
    fs::file_time_type latest_time;
    for (const auto& entry : fs::directory_iterator(captures_root))
    {
      if (!entry.is_directory())
        continue;

      if (!latest.has_value() || entry.last_write_time() > latest_time)
      {
        latest = entry.path();
        latest_time = entry.last_write_time();
      }
    }
    return latest ? std::optional<fs::path>(normalizePath(*latest)) : std::nullopt;
  }

  fs::path resolveSessionPath(const std::string& raw_path, bool use_latest) const
  {
    const fs::path captures_root = capturesRoot();
    const std::string value = trimCopy(raw_path);

    if ((use_latest && value.empty()) || value.empty())
    {
      const auto latest = getLatestSession(captures_root);
      return latest.value_or(captures_root);
    }

    if (value.rfind("@session", 0) == 0)
    {
      fs::path session_root;
      if (const char* active = std::getenv("BEHAV3D_ACTIVE_SESSION"); active && *active && fs::exists(active))
      {
        session_root = normalizePath(fs::path(active));
      }
      else
      {
        const auto latest = getLatestSession(captures_root);
        session_root = latest.value_or(captures_root);
      }

      std::string sub = value.substr(std::string("@session").size());
      while (!sub.empty() && sub.front() == '/')
        sub.erase(sub.begin());
      return sub.empty() ? session_root : normalizePath(session_root / sub);
    }

    const fs::path path_value = expandUserPath(value);
    if (path_value.is_absolute())
      return normalizePath(path_value);
    return normalizePath(captures_root / path_value);
  }

  std::optional<fs::path> resolveCandidatePath(const std::string& raw_path, const fs::path& session_dir) const
  {
    std::string value = trimCopy(raw_path);
    if (value.empty())
      return std::nullopt;

    if (value.rfind("file://", 0) == 0)
      value = value.substr(7);

    if (value.rfind("@session", 0) == 0)
      return resolveSessionPath(value, false);

    const fs::path path_value = expandUserPath(value);
    if (path_value.is_absolute())
      return normalizePath(path_value);
    return normalizePath(session_dir / path_value);
  }

  std::optional<fs::path> latestExistingMeshPath(const fs::path& session_dir) const
  {
    if (!fs::exists(session_dir) || !fs::is_directory(session_dir))
      return std::nullopt;

    static const std::unordered_set<std::string> allowed_names = {
      "tsdf_surface_mesh.stl",
      "tsdf_surface_mesh.ply",
      "tsdf_surface_mesh.obj",
    };

    std::optional<fs::path> latest;
    fs::file_time_type latest_time;
    for (const auto& entry : fs::recursive_directory_iterator(
             session_dir,
             fs::directory_options::skip_permission_denied))
    {
      if (!entry.is_regular_file())
        continue;
      if (allowed_names.find(entry.path().filename().string()) == allowed_names.end())
        continue;

      if (!latest.has_value() || entry.last_write_time() > latest_time)
      {
        latest = entry.path();
        latest_time = entry.last_write_time();
      }
    }
    return latest ? std::optional<fs::path>(normalizePath(*latest)) : std::nullopt;
  }

  fs::path defaultMeshPath(const fs::path& session_dir) const
  {
    if (const auto latest = latestExistingMeshPath(session_dir); latest.has_value())
      return *latest;

    const std::vector<fs::path> candidates = {
      session_dir / "tsdf_surface_mesh.stl",
      session_dir / "tsdf_surface_mesh.ply",
      session_dir / "tsdf_surface_mesh.obj",
    };

    for (const auto& candidate : candidates)
    {
      if (fs::exists(candidate) && fs::is_regular_file(candidate))
        return normalizePath(candidate);
    }
    return normalizePath(candidates.front());
  }

  static std::chrono::system_clock::time_point toSystemClock(fs::file_time_type file_time)
  {
    return std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        std::chrono::system_clock::now() + (file_time - fs::file_time_type::clock::now()));
  }

  bool waitForFile(
      const fs::path& path,
      double timeout_s,
      std::optional<std::chrono::system_clock::time_point> min_write_time) const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    while (true)
    {
      if (fs::exists(path) && fs::is_regular_file(path))
      {
        if (!min_write_time.has_value())
          return true;

        try
        {
          if (toSystemClock(fs::last_write_time(path)) >= *min_write_time)
            return true;
        }
        catch (const std::exception&)
        {
          return true;
        }
      }

      if (std::chrono::steady_clock::now() >= deadline)
        return false;

      std::this_thread::sleep_for(250ms);
    }
  }

  bool applyPlanningSceneMesh(
      const fs::path& mesh_path,
      const std::string& object_id,
      const std::string& frame_id,
      std::string& error,
      uint32_t& triangle_count) const
  {
    triangle_count = 0;
    const std::string resource = "file://" + mesh_path.string();
    std::unique_ptr<shapes::Mesh> shape_mesh(shapes::createMeshFromResource(resource));
    if (!shape_mesh)
    {
      error = "Failed to load mesh resource: " + resource;
      return false;
    }

    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(shape_mesh.get(), shape_msg))
    {
      error = "Failed to convert mesh to shape message: " + mesh_path.string();
      return false;
    }

    const auto* mesh_msg = boost::get<shape_msgs::msg::Mesh>(&shape_msg);
    if (mesh_msg == nullptr)
    {
      error = "Converted shape was not a triangle mesh: " + mesh_path.string();
      return false;
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_object.meshes.push_back(*mesh_msg);

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    collision_object.mesh_poses.push_back(mesh_pose);

    try
    {
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface("", false);
      if (!planning_scene_interface.applyCollisionObject(collision_object))
      {
        error = "PlanningSceneInterface.applyCollisionObject returned false";
        return false;
      }
    }
    catch (const std::exception& exc)
    {
      error = std::string("PlanningSceneInterface exception: ") + exc.what();
      return false;
    }

    triangle_count = static_cast<uint32_t>(mesh_msg->triangles.size());
    return true;
  }

  // --- Utilities ---
  moveit::core::RobotState makeStartStateFromJointState(const moveit::core::RobotModelConstPtr &model)
  {
    moveit::core::RobotState state(model);
    sensor_msgs::msg::JointState copy;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      copy = last_joint_state_;
    }

    const auto names = model->getVariableNames();
    std::unordered_set<std::string> known(names.begin(), names.end());

    for (size_t i = 0; i < copy.name.size(); ++i)
    {
      if (i < copy.position.size() && known.count(copy.name[i]))
        state.setVariablePosition(copy.name[i], copy.position[i]);
    }

    state.update();
    return state;
  }

  geometry_msgs::msg::PoseStamped safeFinalPose(const robot_trajectory::RobotTrajectory &rt,
                                                const std::string &eef,
                                                const std::string &frame)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame;
    if (rt.getWayPointCount() > 0)
      out.pose = tf2::toMsg(rt.getLastWayPoint().getGlobalLinkTransform(eef));
    else
      RCLCPP_WARN(this->get_logger(), "No waypoints in trajectory -> final_pose left default");
    return out;
  }

  double safeTotalTime(const moveit_msgs::msg::RobotTrajectory &traj)
  {
    if (traj.joint_trajectory.points.empty()) return 0.0;
    const auto &last = traj.joint_trajectory.points.back();
    return last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
  }

  static double durationToSec(const builtin_interfaces::msg::Duration& duration)
  {
    return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
  }

  static void setDurationMsg(double seconds, builtin_interfaces::msg::Duration& out)
  {
    const double safe_seconds = std::max(0.0, seconds);
    out.sec = static_cast<int32_t>(std::floor(safe_seconds));
    out.nanosec = static_cast<uint32_t>(
        std::round((safe_seconds - static_cast<double>(out.sec)) * 1e9));
    if (out.nanosec >= 1000000000u)
    {
      out.sec += 1;
      out.nanosec -= 1000000000u;
    }
  }

  static bool allFinite(const std::vector<double>& values)
  {
    return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
  }

  bool validateTrajectoryTimingAndValues(
      const moveit_msgs::msg::RobotTrajectory& traj_msg,
      double min_dt_s,
      std::string& error) const
  {
    error.clear();
    const auto& jt = traj_msg.joint_trajectory;
    const size_t dof = jt.joint_names.size();
    if (dof == 0)
    {
      error = "trajectory has no joint names";
      return false;
    }
    if (jt.points.empty())
    {
      error = "trajectory has no points";
      return false;
    }

    const double min_dt = std::max(1e-4, min_dt_s);
    double previous_time = -1.0;
    for (size_t i = 0; i < jt.points.size(); ++i)
    {
      const auto& point = jt.points[i];
      const double time_s = durationToSec(point.time_from_start);
      if (!std::isfinite(time_s) || time_s < 0.0)
      {
        error = "trajectory contains invalid timestamp at point " + std::to_string(i);
        return false;
      }
      if (i > 0)
      {
        const double dt = time_s - previous_time;
        if (dt <= 1e-9)
        {
          error = "trajectory timestamps are not strictly increasing at point " + std::to_string(i);
          return false;
        }
        if (dt + 1e-9 < min_dt)
        {
          std::ostringstream ss;
          ss << "trajectory dt too small at point " << i << ": " << dt << "s < " << min_dt << "s";
          error = ss.str();
          return false;
        }
      }
      previous_time = time_s;

      if (point.positions.size() != dof)
      {
        error = "trajectory position dimension mismatch at point " + std::to_string(i);
        return false;
      }
      if (!point.velocities.empty() && point.velocities.size() != dof)
      {
        error = "trajectory velocity dimension mismatch at point " + std::to_string(i);
        return false;
      }
      if (!point.accelerations.empty() && point.accelerations.size() != dof)
      {
        error = "trajectory acceleration dimension mismatch at point " + std::to_string(i);
        return false;
      }
      if (!allFinite(point.positions) || !allFinite(point.velocities) || !allFinite(point.accelerations))
      {
        error = "trajectory contains NaN/Inf at point " + std::to_string(i);
        return false;
      }
    }

    return true;
  }

  void logJointVelocityAccelerationStats(
      const moveit::core::RobotModelConstPtr& robot_model,
      const moveit_msgs::msg::RobotTrajectory& traj_msg,
      const std::string& label) const
  {
    if (!log_sequence_joint_stats_)
      return;
    if (!robot_model)
    {
      RCLCPP_WARN(this->get_logger(), "[%s] Joint stats skipped: robot model is null", label.c_str());
      return;
    }

    const auto& jt = traj_msg.joint_trajectory;
    const size_t dof = jt.joint_names.size();
    if (dof == 0 || jt.points.empty())
    {
      RCLCPP_WARN(this->get_logger(), "[%s] Joint stats skipped: empty trajectory", label.c_str());
      return;
    }

    const auto model_variable_names = robot_model->getVariableNames();
    const std::unordered_set<std::string> known_variables(
        model_variable_names.begin(), model_variable_names.end());

    for (size_t j = 0; j < dof; ++j)
    {
      const auto& joint_name = jt.joint_names[j];
      if (!known_variables.count(joint_name))
      {
        RCLCPP_WARN(this->get_logger(),
                    "[%s] Joint stats skipped for unknown joint=%s",
                    label.c_str(), joint_name.c_str());
        continue;
      }

      double max_abs_velocity = 0.0;
      double max_abs_accel = 0.0;
      size_t max_velocity_point = 0;
      size_t max_accel_point = 0;

      for (size_t i = 0; i < jt.points.size(); ++i)
      {
        const auto& point = jt.points[i];
        if (point.velocities.size() == dof)
        {
          const double abs_velocity = std::abs(point.velocities[j]);
          if (abs_velocity > max_abs_velocity)
          {
            max_abs_velocity = abs_velocity;
            max_velocity_point = i;
          }
        }

        if (point.accelerations.size() == dof)
        {
          const double abs_accel = std::abs(point.accelerations[j]);
          if (abs_accel > max_abs_accel)
          {
            max_abs_accel = abs_accel;
            max_accel_point = i;
          }
        }
      }

      const auto& bounds = robot_model->getVariableBounds(joint_name);
      const double velocity_limit = bounds.velocity_bounded_
          ? std::max(std::abs(bounds.min_velocity_), std::abs(bounds.max_velocity_))
          : 0.0;
      const double accel_limit = bounds.acceleration_bounded_
          ? std::max(std::abs(bounds.min_acceleration_), std::abs(bounds.max_acceleration_))
          : 0.0;
      const double velocity_ratio = velocity_limit > 1e-9 ? max_abs_velocity / velocity_limit : 0.0;
      const double accel_ratio = accel_limit > 1e-9 ? max_abs_accel / accel_limit : 0.0;

      RCLCPP_WARN(this->get_logger(),
                  "[%s] Joint stats: joint=%s "
                  "max_abs_velocity=%.6f limit=%.6f ratio=%.3f point=%zu "
                  "max_abs_accel=%.6f limit=%.6f ratio=%.3f point=%zu",
                  label.c_str(),
                  joint_name.c_str(),
                  max_abs_velocity, velocity_limit, velocity_ratio, max_velocity_point,
                  max_abs_accel, accel_limit, accel_ratio, max_accel_point);
    }
  }

  bool retimeForConstantTcpSpeed(
      robot_trajectory::RobotTrajectory& rt,
      moveit_msgs::msg::RobotTrajectory& traj_msg,
      const std::string& tip_link,
      double target_speed_m_s,
      double min_dt_s,
      double tcp_sample_spacing_m,
      std::string& error) const
  {
    error.clear();
    if (target_speed_m_s <= 1e-9)
      return false;

    const size_t count = rt.getWayPointCount();
    if (count < 2)
    {
      error = "trajectory has fewer than 2 waypoints";
      return false;
    }

    const double min_dt = std::max(1e-4, min_dt_s);
    std::vector<double> cumulative_distances(count, 0.0);
    for (size_t i = 1; i < count; ++i)
    {
      const Eigen::Vector3d p0 = rt.getWayPoint(i - 1).getGlobalLinkTransform(tip_link).translation();
      const Eigen::Vector3d p1 = rt.getWayPoint(i).getGlobalLinkTransform(tip_link).translation();
      const double ds = (p1 - p0).norm();
      cumulative_distances[i] = cumulative_distances[i - 1] + ds;
    }

    const double total_distance = cumulative_distances.back();
    if (total_distance <= 1e-9)
    {
      error = "trajectory TCP path length is zero";
      return false;
    }

    const double requested_spacing = std::max(0.0, tcp_sample_spacing_m);
    const double effective_spacing = std::max(
        requested_spacing > 1e-9 ? requested_spacing : target_speed_m_s * min_dt,
        target_speed_m_s * min_dt);

    std::vector<double> sample_distances;
    sample_distances.reserve(static_cast<size_t>(std::ceil(total_distance / effective_spacing)) + 2);
    sample_distances.push_back(0.0);
    for (double s = effective_spacing; s < total_distance - 1e-9; s += effective_spacing)
      sample_distances.push_back(s);
    if (sample_distances.back() < total_distance)
      sample_distances.push_back(total_distance);

    robot_trajectory::RobotTrajectory sampled_rt(rt.getRobotModel(), rt.getGroup());
    sampled_rt.addSuffixWayPoint(rt.getFirstWayPoint(), 0.0);

    size_t segment_index = 1;
    for (size_t sample_i = 1; sample_i < sample_distances.size(); ++sample_i)
    {
      const double sample_s = sample_distances[sample_i];
      while (segment_index + 1 < count && cumulative_distances[segment_index] < sample_s)
        ++segment_index;

      const double prev_s = cumulative_distances[segment_index - 1];
      const double next_s = cumulative_distances[segment_index];
      const double segment_length = next_s - prev_s;

      moveit::core::RobotState sampled_state(rt.getWayPoint(segment_index - 1));
      if (segment_length > 1e-9 && sample_s < total_distance - 1e-9)
      {
        const double alpha = std::clamp((sample_s - prev_s) / segment_length, 0.0, 1.0);
        rt.getWayPoint(segment_index - 1)
            .interpolate(rt.getWayPoint(segment_index), alpha, sampled_state, rt.getGroup());
        sampled_state.update();
      }
      else
      {
        sampled_state = rt.getWayPoint(segment_index);
        sampled_state.update();
      }

      const double ds = sample_distances[sample_i] - sample_distances[sample_i - 1];
      const double dt = std::max(min_dt, ds / target_speed_m_s);
      sampled_rt.addSuffixWayPoint(sampled_state, dt);
    }

    std::vector<double> times(sampled_rt.getWayPointCount(), 0.0);
    for (size_t i = 1; i < sampled_rt.getWayPointCount(); ++i)
      times[i] = times[i - 1] + sampled_rt.getWayPointDurationFromPrevious(i);

    sampled_rt.getRobotTrajectoryMsg(traj_msg);
    auto& points = traj_msg.joint_trajectory.points;
    if (points.size() != sampled_rt.getWayPointCount())
    {
      error = "retimed trajectory point count mismatch";
      return false;
    }

    for (size_t i = 0; i < points.size(); ++i)
      setDurationMsg(times[i], points[i].time_from_start);

    const size_t dof = points.front().positions.size();
    if (dof == 0)
    {
      error = "trajectory points do not contain positions";
      return false;
    }

    for (auto& point : points)
    {
      point.velocities.assign(dof, 0.0);
      point.accelerations.assign(dof, 0.0);
    }

    for (size_t i = 1; i + 1 < points.size(); ++i)
    {
      const double dt = times[i + 1] - times[i - 1];
      if (dt <= 1e-9)
        continue;
      for (size_t j = 0; j < dof; ++j)
        points[i].velocities[j] = (points[i + 1].positions[j] - points[i - 1].positions[j]) / dt;
    }

    if (points.size() >= 2)
    {
      const double dt0 = times[1] - times[0];
      if (dt0 > 1e-9)
      {
        for (size_t j = 0; j < dof; ++j)
          points[0].accelerations[j] = (points[1].velocities[j] - points[0].velocities[j]) / dt0;
      }

      for (size_t i = 1; i < points.size(); ++i)
      {
        const double dt = times[i] - times[i - 1];
        if (dt <= 1e-9)
          continue;
        for (size_t j = 0; j < dof; ++j)
          points[i].accelerations[j] = (points[i].velocities[j] - points[i - 1].velocities[j]) / dt;
      }
    }

    // Keep endpoints at rest for controller compatibility.
    std::fill(points.front().velocities.begin(), points.front().velocities.end(), 0.0);
    std::fill(points.back().velocities.begin(), points.back().velocities.end(), 0.0);
    std::fill(points.front().accelerations.begin(), points.front().accelerations.end(), 0.0);
    std::fill(points.back().accelerations.begin(), points.back().accelerations.end(), 0.0);
    RCLCPP_INFO(this->get_logger(),
                "[Pilz SEQ] TCP resample retime: original_points=%zu sampled_points=%zu "
                "path=%.4fm requested_spacing=%.3fmm effective_spacing=%.3fmm target=%.3fmm/s",
                count,
                sampled_rt.getWayPointCount(),
                total_distance,
                requested_spacing * 1000.0,
                effective_spacing * 1000.0,
                target_speed_m_s * 1000.0);
    return true;
  }

  struct TcpSpeedStats
  {
    double path_length_m = 0.0;
    double duration_s = 0.0;
    double min_speed_m_s = 0.0;
    double mean_speed_m_s = 0.0;
    double max_speed_m_s = 0.0;
    uint32_t sample_count = 0;
    uint32_t low_sample_count = 0;
  };

  TcpSpeedStats computeTcpSpeedStats(
      const robot_trajectory::RobotTrajectory& rt,
      const std::string& tip_link,
      double low_speed_threshold_m_s) const
  {
    TcpSpeedStats stats;
    if (rt.getWayPointCount() < 2)
      return stats;

    double min_speed = std::numeric_limits<double>::infinity();
    double max_speed = 0.0;
    for (size_t i = 1; i < rt.getWayPointCount(); ++i)
    {
      const double dt = rt.getWayPointDurationFromPrevious(i);
      if (dt <= 1e-9)
        continue;

      const Eigen::Vector3d p0 = rt.getWayPoint(i - 1).getGlobalLinkTransform(tip_link).translation();
      const Eigen::Vector3d p1 = rt.getWayPoint(i).getGlobalLinkTransform(tip_link).translation();
      const double ds = (p1 - p0).norm();
      const double speed = ds / dt;

      stats.path_length_m += ds;
      stats.duration_s += dt;
      min_speed = std::min(min_speed, speed);
      max_speed = std::max(max_speed, speed);
      stats.sample_count += 1;
      if (low_speed_threshold_m_s > 0.0 && speed < low_speed_threshold_m_s)
        stats.low_sample_count += 1;
    }

    if (stats.sample_count > 0)
    {
      stats.min_speed_m_s = std::isfinite(min_speed) ? min_speed : 0.0;
      stats.max_speed_m_s = max_speed;
      stats.mean_speed_m_s = stats.duration_s > 1e-9 ? stats.path_length_m / stats.duration_s : 0.0;
    }
    return stats;
  }

  void publishEEFPath(const robot_trajectory::RobotTrajectory &rt,
                      const std::string &tip_link,
                      const std::string &frame_id)
  {
    visualization_msgs::msg::Marker m;
    m.header.stamp = this->now();
    m.header.frame_id = frame_id;
    m.ns = "eef_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.004;
    m.color.a = 1.0;
    m.color.r = 0.1f;
    m.color.g = 0.8f;
    m.color.b = 0.2f;

    m.points.clear();
    m.points.reserve(rt.getWayPointCount());
    for (size_t i = 0; i < rt.getWayPointCount(); ++i)
    {
      const auto &s = rt.getWayPoint(i);
      const auto T = s.getGlobalLinkTransform(tip_link);
      geometry_msgs::msg::Point p;
      p.x = T.translation().x();
      p.y = T.translation().y();
      p.z = T.translation().z();
      m.points.push_back(p);
    }
    marker_pub_->publish(m);
  }

  
  // Get current pose of a link expressed in a base frame
  void getLinkPoseCallback(
    const std::shared_ptr<behav3d_interfaces::srv::GetLinkPose::Request> req,
    std::shared_ptr<behav3d_interfaces::srv::GetLinkPose::Response> res)
  {
    // Create a MoveGroupInterface (pick your default group or make it a param)
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), "ur_arm");
    auto model = mgi.getRobotModel();

    // Build current RobotState from /joint_states
    moveit::core::RobotState state = makeStartStateFromJointState(model);

    // Determine frames
    const std::string base = req->base_frame.empty() ? mgi.getPlanningFrame() : req->base_frame;
    const std::string tip  = req->link;

    // Validate tip link exists
    if (!model->hasLinkModel(tip)) {
      res->success = false;
      res->message = "Unknown link: " + tip;
      return;
    }

    // Get transforms in RobotState's global frame
    Eigen::Isometry3d T_base = Eigen::Isometry3d::Identity();
    if (model->hasLinkModel(base)) {
      T_base = state.getGlobalLinkTransform(base);
    }
    const Eigen::Isometry3d T_tip = state.getGlobalLinkTransform(tip);

    // Pose of tip expressed in base
    const Eigen::Isometry3d T_base_tip = T_base.inverse() * T_tip;

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = base;
    out.pose = tf2::toMsg(T_base_tip);

    res->success = true;
    res->message = "OK";
    res->pose = out;
  }

  void publishTargetsCallback(
    const std::shared_ptr<behav3d_interfaces::srv::PublishTargets::Request> req,
    std::shared_ptr<behav3d_interfaces::srv::PublishTargets::Response> res);

  void deleteMarkersCallback(
    const std::shared_ptr<behav3d_interfaces::srv::DeleteMarkers::Request> req,
    std::shared_ptr<behav3d_interfaces::srv::DeleteMarkers::Response> res);

  void updatePlanningSceneMeshCallback(
    const std::shared_ptr<behav3d_interfaces::srv::UpdatePlanningSceneMesh::Request> req,
    std::shared_ptr<behav3d_interfaces::srv::UpdatePlanningSceneMesh::Response> res)
  {
    const auto request_started = std::chrono::system_clock::now();
    const fs::path session_dir = resolveSessionPath(req->session_path, bool(req->use_latest));
    res->session_dir = session_dir.string();

    if (!fs::exists(session_dir) || !fs::is_directory(session_dir))
    {
      res->success = false;
      res->message = "Session directory not found: " + session_dir.string();
      res->resolved_mesh_path = "";
      res->applied_object_id = "";
      res->applied_frame_id = "";
      res->triangle_count = 0;
      return;
    }

    fs::path mesh_path = resolveCandidatePath(req->mesh_path, session_dir).value_or(defaultMeshPath(session_dir));
    res->resolved_mesh_path = mesh_path.string();

    const double wait_timeout_s = std::max(0.0, static_cast<double>(req->wait_timeout_s));
    const bool has_explicit_path = !trimCopy(req->mesh_path).empty();
    std::optional<std::chrono::system_clock::time_point> min_write_time;
    if (!has_explicit_path && wait_timeout_s > 0.0)
      min_write_time = request_started - 100ms;

    if (!waitForFile(mesh_path, wait_timeout_s, min_write_time))
    {
      res->success = false;
      res->message =
          "Mesh path not available after " + std::to_string(wait_timeout_s) + "s: " + mesh_path.string();
      res->applied_object_id = "";
      res->applied_frame_id = "";
      res->triangle_count = 0;
      return;
    }

    std::string frame_id = trimCopy(req->frame_id);
    try
    {
      if (frame_id.empty())
      {
        moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), "ur_arm");
        frame_id = mgi.getPlanningFrame();
      }
    }
    catch (const std::exception& exc)
    {
      res->success = false;
      res->message = std::string("Failed to resolve planning frame: ") + exc.what();
      res->applied_object_id = "";
      res->applied_frame_id = "";
      res->triangle_count = 0;
      return;
    }

    std::string object_id = trimCopy(req->object_id);
    if (object_id.empty())
      object_id = "behav3d_reconstructed_mesh";

    uint32_t triangle_count = 0;
    std::string error;
    if (!applyPlanningSceneMesh(mesh_path, object_id, frame_id, error, triangle_count))
    {
      res->success = false;
      res->message = error;
      res->applied_object_id = "";
      res->applied_frame_id = "";
      res->triangle_count = 0;
      return;
    }

    res->success = true;
    res->message = "Applied planning-scene mesh: " + mesh_path.string();
    res->applied_object_id = object_id;
    res->applied_frame_id = frame_id;
    res->triangle_count = triangle_count;
  }


  // --- PlanWithMoveIt callback ---
  void planWithMoveItCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanWithMoveIt::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanWithMoveIt::Response> res)
  {
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), req->group_name);

    if (!req->pipeline_id.empty())
      move_group.setPlanningPipelineId(req->pipeline_id);
    move_group.setPlanningTime(5.0);

    const double vel = std::clamp<double>(req->velocity_scale, 0.0, 1.0);
    const double acc = std::clamp<double>(req->accel_scale, 0.0, 1.0);
    move_group.setMaxVelocityScalingFactor(vel);
    move_group.setMaxAccelerationScalingFactor(acc);

    auto robot_model = move_group.getRobotModel();
    auto start_state = makeStartStateFromJointState(robot_model);
    move_group.setStartState(start_state);
    move_group.clearPathConstraints();
    move_group.setEndEffectorLink(flange_link_);

    if (!req->named_target.empty())
    {
      move_group.setNamedTarget(req->named_target);
    }
    else
    {
      auto goal = req->pose;
      if (goal.header.frame_id.empty())
        goal.header.frame_id = move_group.getPlanningFrame();
      move_group.setPoseReferenceFrame(goal.header.frame_id);
      move_group.setPoseTarget(goal, flange_link_);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = move_group.plan(plan);
    if (!code)
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed (error code %d)", code.val);
      res->success = false;
      res->moveit_error_code = code.val;
      return;
    }

    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
    rt.setRobotTrajectoryMsg(start_state, plan.trajectory);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    (void)totg.computeTimeStamps(rt, vel, acc);

    moveit_msgs::msg::RobotTrajectory traj_msg;
    rt.getRobotTrajectoryMsg(traj_msg);

    geometry_msgs::msg::PoseStamped final_pose =
        safeFinalPose(rt, move_group.getEndEffectorLink(), move_group.getPlanningFrame());

    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = robot_model->getName();
    disp.trajectory_start = plan.start_state;
    disp.trajectory.push_back(traj_msg);
    display_pub_->publish(disp);
    last_plan_pub_->publish(traj_msg);
    if (viz_enabled_)
      publishEEFPath(rt, move_group.getEndEffectorLink(), final_pose.header.frame_id);

    res->success = true;
    res->moveit_error_code = 0;
    res->final_pose = final_pose;

    if (!req->preview_only)
    {
      if (exec_mode_ == "controller_action")
      {
        if (fjt_client_->wait_for_action_server(2s))
        {
          auto goal = FJT::Goal();
          goal.trajectory = traj_msg.joint_trajectory;
          fjt_client_->async_send_goal(goal);
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan = plan;
        exec_plan.trajectory = traj_msg;
        auto exec_code = move_group.execute(exec_plan);
        if (!exec_code)
          RCLCPP_WARN(this->get_logger(), "Execution failed (code=%d)", exec_code.val);
      }
    }

    move_group.clearPoseTargets();
  }

  // --- Cartesian path callback ---
  void planCartesianCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanCartesianPath::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanCartesianPath::Response> res)
  {
    if (req->waypoints.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "CartesianPath: no waypoints provided");
      res->success = false;
      res->fraction = 0.0;
      return;
    }

    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), req->group_name);

    if (!req->pipeline_id.empty())
      mgi.setPlanningPipelineId(req->pipeline_id);
    mgi.setPlanningTime(5.0);

    const double vel = std::clamp<double>(req->velocity_scale, 0.0, 1.0);
    const double acc = std::clamp<double>(req->accel_scale, 0.0, 1.0);
    mgi.setMaxVelocityScalingFactor(vel);
    mgi.setMaxAccelerationScalingFactor(acc);

    auto robot_model = mgi.getRobotModel();
    auto start_state = makeStartStateFromJointState(robot_model);
    mgi.setStartState(start_state);
    mgi.clearPathConstraints();

    const std::string eef = req->eef_link.empty() ? flange_link_ : req->eef_link;
    mgi.setEndEffectorLink(eef);

    const std::string frame = req->frame_id.empty() ? mgi.getPlanningFrame() : req->frame_id;
    mgi.setPoseReferenceFrame(frame);

    moveit_msgs::msg::RobotTrajectory traj_msg;
    double max_step = (req->max_step > 0.0) ? req->max_step : 0.005;
    bool avoid_coll = req->avoid_collisions;

    // Jazzy signature: (waypoints, eef_step, traj_msg, avoid_collisions)
    double fraction = mgi.computeCartesianPath(req->waypoints, max_step, traj_msg, avoid_coll);

    res->fraction = fraction;
    if (fraction <= 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "CartesianPath failed (fraction=%.3f)", fraction);
      res->success = false;
      return;
    }

    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
    rt.setRobotTrajectoryMsg(start_state, traj_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    (void)totg.computeTimeStamps(rt, vel, acc);
    rt.getRobotTrajectoryMsg(traj_msg);

    geometry_msgs::msg::PoseStamped final_pose =
        safeFinalPose(rt, mgi.getEndEffectorLink(), mgi.getPlanningFrame());

    moveit_msgs::msg::DisplayTrajectory disp;
    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(start_state, start_msg);
    disp.trajectory_start = start_msg;
    disp.trajectory.push_back(traj_msg);
    display_pub_->publish(disp);
    last_plan_pub_->publish(traj_msg);
    if (viz_enabled_)
      publishEEFPath(rt, eef, final_pose.header.frame_id);

    res->success = true;
    res->trajectory = traj_msg;
    res->final_pose = final_pose;

    if (!req->preview_only)
    {
      if (exec_mode_ == "controller_action")
      {
        if (fjt_client_->wait_for_action_server(2s))
        {
          auto goal = FJT::Goal();
          goal.trajectory = traj_msg.joint_trajectory;
          fjt_client_->async_send_goal(goal);
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
        exec_plan.trajectory = traj_msg;
        auto exec_code = mgi.execute(exec_plan);
        if (!exec_code)
          RCLCPP_WARN(this->get_logger(), "Execution failed (code=%d)", exec_code.val);
      }
    }
  }

  // --- Pilz LIN ---
  void planPilzLinCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanPilzLin::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanPilzLin::Response> res)
  {
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), req->group_name);

    const std::string pipeline = req->pipeline_id.empty() ? "pilz_industrial_motion_planner" : req->pipeline_id;
    mgi.setPlanningPipelineId(pipeline);
    const std::string planner = req->planner_id.empty() ? "LIN" : req->planner_id;
    mgi.setPlannerId(planner);

    if (!req->eef_link.empty()) mgi.setEndEffectorLink(req->eef_link);
    mgi.setPlanningTime(5.0);
    mgi.setMaxVelocityScalingFactor(std::clamp<double>(req->velocity_scale, 0.0, 1.0));
    mgi.setMaxAccelerationScalingFactor(std::clamp<double>(req->accel_scale, 0.0, 1.0));

    auto robot_model = mgi.getRobotModel();
    auto start_state = makeStartStateFromJointState(robot_model);
    mgi.setStartState(start_state);
    mgi.clearPathConstraints();

    auto goal = req->target;
    if (goal.header.frame_id.empty()) goal.header.frame_id = mgi.getPlanningFrame();
    mgi.setPoseReferenceFrame(goal.header.frame_id);
    if (!req->eef_link.empty())
      mgi.setPoseTarget(goal, req->eef_link);
    else
      mgi.setPoseTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = mgi.plan(plan);
    if (!code)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz LIN] Planning failed (code=%d)", code.val);
      res->success = false; res->moveit_error_code = code.val;
      return;
    }

    moveit_msgs::msg::RobotTrajectory traj_copy = plan.trajectory;
    res->trajectory = traj_copy;

    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
    rt.setRobotTrajectoryMsg(start_state, traj_copy);
    res->final_pose = safeFinalPose(rt, mgi.getEndEffectorLink(), mgi.getPlanningFrame());
    res->total_time_sec = safeTotalTime(traj_copy);

    res->success = true; res->moveit_error_code = 0;

    if (!req->preview_only)
    {
      if (exec_mode_ == "controller_action")
      {
        if (fjt_client_->wait_for_action_server(2s))
        {
          auto goal_action = FJT::Goal();
          goal_action.trajectory = traj_copy.joint_trajectory;
          fjt_client_->async_send_goal(goal_action);
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
        exec_plan.trajectory = traj_copy;
        auto exec_code = mgi.execute(exec_plan);
        if (!exec_code)
          RCLCPP_WARN(this->get_logger(), "[Pilz LIN] Execution failed (code=%d)", exec_code.val);
      }
    }
  }

  // --- Pilz PTP ---
  void planPilzPtpCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanPilzPtp::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanPilzPtp::Response> res)
  {
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), req->group_name);

    const std::string pipeline = req->pipeline_id.empty() ? "pilz_industrial_motion_planner" : req->pipeline_id;
    mgi.setPlanningPipelineId(pipeline);
    const std::string planner = req->planner_id.empty() ? "PTP" : req->planner_id;
    mgi.setPlannerId(planner);

    if (!req->eef_link.empty()) mgi.setEndEffectorLink(req->eef_link);
    mgi.setPlanningTime(5.0);
    mgi.setMaxVelocityScalingFactor(std::clamp<double>(req->velocity_scale, 0.0, 1.0));
    mgi.setMaxAccelerationScalingFactor(std::clamp<double>(req->accel_scale, 0.0, 1.0));

    auto robot_model = mgi.getRobotModel();
    auto start_state = makeStartStateFromJointState(robot_model);
    mgi.setStartState(start_state);
    mgi.clearPathConstraints();

    if (!req->named_target.empty()) {
      mgi.setNamedTarget(req->named_target);
    } else {
      auto goal = req->target;
      if (goal.header.frame_id.empty()) goal.header.frame_id = mgi.getPlanningFrame();
      mgi.setPoseReferenceFrame(goal.header.frame_id);
      if (!req->eef_link.empty())
        mgi.setPoseTarget(goal, req->eef_link);
      else
        mgi.setPoseTarget(goal);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = mgi.plan(plan);
    if (!code)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz PTP] Planning failed (code=%d)", code.val);
      res->success = false; res->moveit_error_code = code.val;
      return;
    }

    moveit_msgs::msg::RobotTrajectory traj_copy = plan.trajectory;
    res->trajectory = traj_copy;

    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
    rt.setRobotTrajectoryMsg(start_state, traj_copy);
    res->final_pose = safeFinalPose(rt, mgi.getEndEffectorLink(), mgi.getPlanningFrame());
    res->total_time_sec = safeTotalTime(traj_copy);

    res->success = true; res->moveit_error_code = 0;

    if (!req->preview_only)
    {
      if (exec_mode_ == "controller_action")
      {
        if (fjt_client_->wait_for_action_server(2s))
        {
          auto goal_action = FJT::Goal();
          goal_action.trajectory = traj_copy.joint_trajectory;
          fjt_client_->async_send_goal(goal_action);
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
        exec_plan.trajectory = traj_copy;
        auto exec_code = mgi.execute(exec_plan);
        if (!exec_code)
          RCLCPP_WARN(this->get_logger(), "[Pilz PTP] Execution failed (code=%d)", exec_code.val);
      }
    }
  }

  // --- Pilz LIN/LIN[/CIRC] SEQUENCE with blending (Jazzy: ACTION result.response.*) ---
  void planPilzSequenceCallback(
      const std::shared_ptr<behav3d_interfaces::srv::PlanPilzSequence::Request> req,
      std::shared_ptr<behav3d_interfaces::srv::PlanPilzSequence::Response> res)
  {
    if (req->targets.size() < 2)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Need at least 2 targets");
      res->success = false;
      res->message = "Need at least 2 targets";
      return;
    }

    // Use MoveGroupInterface only to query planning frame/model
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), req->group_name);
    const std::string planning_frame = mgi.getPlanningFrame();
    const std::string target_frame = trimCopy(req->frame_id).empty() ? planning_frame : trimCopy(req->frame_id);
    const std::string eef = req->eef_link.empty() ? flange_link_ : req->eef_link;

    // Build current start state message
    auto robot_model = mgi.getRobotModel();
    auto start_state_core = makeStartStateFromJointState(robot_model);
    moveit_msgs::msg::RobotState start_state_msg;
    moveit::core::robotStateToRobotStateMsg(start_state_core, start_state_msg);

    // Fill the MotionSequenceRequest (Jazzy: only 'items' lives here)
    moveit_msgs::msg::MotionSequenceRequest seq;
    seq.items.clear();
    seq.items.reserve(req->targets.size());

    for (size_t i = 0; i < req->targets.size(); ++i)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = target_frame;
      ps.pose = req->targets[i];

      moveit_msgs::msg::MotionPlanRequest mpr;
      mpr.group_name = req->group_name;
      mpr.pipeline_id = "pilz_industrial_motion_planner";
      mpr.planner_id = "LIN";
      mpr.max_velocity_scaling_factor = std::clamp<double>(req->velocity_scale, 0.0, 1.0);
      mpr.max_acceleration_scaling_factor = std::clamp<double>(req->accel_scale, 0.0, 1.0);
      mpr.allowed_planning_time = 10.0;

      if (i == 0)
        mpr.start_state = start_state_msg;

      // Pose goal for EEF
      auto gc = kinematic_constraints::constructGoalConstraints(eef, ps, 1e-3, 1e-3);
      mpr.goal_constraints.clear();
      mpr.goal_constraints.push_back(gc);

      moveit_msgs::msg::MotionSequenceItem item;
      item.req = mpr;
      item.blend_radius =
          (i + 1 == req->targets.size()) ? 0.0 :
          ((i < req->blend_radii.size()) ? std::max(0.0, req->blend_radii[i]) : 0.0);

      seq.items.push_back(item);
    }

    std::string blend_summary;
    for (size_t i = 0; i < seq.items.size(); ++i)
    {
      if (!blend_summary.empty())
        blend_summary += ", ";
      blend_summary += std::to_string(seq.items[i].blend_radius);
    }
    RCLCPP_INFO(this->get_logger(),
                "[Pilz SEQ] Request: targets=%zu frame=%s eef=%s v=%.3f a=%.3f blend_radii=[%s]",
                req->targets.size(), target_frame.c_str(), eef.c_str(),
                std::clamp<double>(req->velocity_scale, 0.0, 1.0),
                std::clamp<double>(req->accel_scale, 0.0, 1.0),
                blend_summary.c_str());

    // Use the dedicated action client/node (avoid executor double-add)
    if (!mgs_client_->wait_for_action_server(3s))
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] sequence_move_group action not available");
      res->success = false;
      res->message = "sequence_move_group action not available";
      return;
    }

    MGS::Goal goal;
    goal.request = seq;
    goal.planning_options.planning_scene_diff.is_diff = true;
    goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;

    auto send_future = mgs_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(seq_client_node_->get_node_base_interface(), send_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Action send failed");
      res->success = false;
      res->message = "Action send failed";
      return;
    }

    auto gh = send_future.get();
    if (!gh)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Goal rejected");
      res->success = false;
      res->message = "Goal rejected";
      return;
    }

    auto result_future = mgs_client_->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(seq_client_node_->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Failed to get result");
      res->success = false;
      res->message = "Failed to get result";
      return;
    }

    auto wrapped_result = result_future.get();
    const auto &resp = wrapped_result.result->response;

    if (resp.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Planning failed (code=%d)", resp.error_code.val);
      res->success = false;
      res->message = "Planning failed (code=" + std::to_string(resp.error_code.val) + ")";
      return;
    }
    if (resp.planned_trajectories.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] No trajectories returned");
      res->success = false;
      res->message = "No trajectories returned";
      return;
    }

    moveit_msgs::msg::RobotTrajectory planned;
    if (resp.planned_trajectories.size() == 1)
    {
      planned = resp.planned_trajectories.front();
    }
    else
    {
      robot_trajectory::RobotTrajectory combined(robot_model, req->group_name);
      combined.setRobotTrajectoryMsg(start_state_core, resp.planned_trajectories.front());

      moveit::core::RobotState ref_state = combined.getLastWayPoint();
      for (size_t i = 1; i < resp.planned_trajectories.size(); ++i)
      {
        robot_trajectory::RobotTrajectory segment(robot_model, req->group_name);
        segment.setRobotTrajectoryMsg(ref_state, resp.planned_trajectories[i]);
        combined.append(segment, 0.0);
        ref_state = combined.getLastWayPoint();
      }
      combined.getRobotTrajectoryMsg(planned);
    }

    robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
    rt.setRobotTrajectoryMsg(start_state_core, planned);
    bool tcp_speed_retimed = false;
    bool tcp_speed_retime_fallback = false;
    std::string status_message = "Pilz sequence planned";
    double tcp_speed_threshold_m_s = std::max(0.0, static_cast<double>(req->tcp_speed_threshold_m_s));
    if (static_cast<double>(req->target_tcp_speed_m_s) > 1e-9)
    {
      const double requested_tcp_speed = static_cast<double>(req->target_tcp_speed_m_s);
      double target_tcp_speed = requested_tcp_speed;
      if (max_tcp_speed_m_s_ > 0.0 && target_tcp_speed > max_tcp_speed_m_s_)
      {
        target_tcp_speed = max_tcp_speed_m_s_;
        if (tcp_speed_threshold_m_s > 0.0)
          tcp_speed_threshold_m_s *= target_tcp_speed / requested_tcp_speed;
        RCLCPP_WARN(this->get_logger(),
                    "[Pilz SEQ] Requested target_tcp_speed=%.3fmm/s exceeds max_tcp_speed=%.3fmm/s; "
                    "clamping retime target to %.3fmm/s",
                    requested_tcp_speed * 1000.0,
                    max_tcp_speed_m_s_ * 1000.0,
                    target_tcp_speed * 1000.0);
      }
      const double retime_min_dt = std::max(1e-4, static_cast<double>(req->retime_min_dt_s));
      std::string retime_error;
      bool retime_allowed = true;

      if (!allow_tcp_retime_)
      {
        retime_allowed = false;
        retime_error = "allow_tcp_retime=false";
      }

      if (retime_allowed)
      {
        moveit_msgs::msg::RobotTrajectory retimed_plan = planned;
        robot_trajectory::RobotTrajectory retimed_rt(robot_model, req->group_name);
        retimed_rt.setRobotTrajectoryMsg(start_state_core, retimed_plan);

        tcp_speed_retimed = retimeForConstantTcpSpeed(
            retimed_rt,
            retimed_plan,
            eef,
            target_tcp_speed,
            retime_min_dt,
            static_cast<double>(req->tcp_sample_spacing_m),
            retime_error);

        if (tcp_speed_retimed)
        {
          std::string validation_error;
          tcp_speed_retimed = validateTrajectoryTimingAndValues(
              retimed_plan, retime_min_dt, validation_error);
          if (!tcp_speed_retimed)
            retime_error = validation_error;
        }

        if (tcp_speed_retimed)
        {
          planned = retimed_plan;
          rt.setRobotTrajectoryMsg(start_state_core, planned);
          status_message = "Pilz sequence retimed for target TCP speed";
          if (target_tcp_speed < requested_tcp_speed)
            status_message += " (clamped by max_tcp_speed_m_s)";
          RCLCPP_INFO(this->get_logger(),
                      "[Pilz SEQ] Retimed trajectory for target_tcp_speed=%.3fmm/s min_dt=%.4fs",
                      target_tcp_speed * 1000.0,
                      retime_min_dt);
        }
      }

      if (!tcp_speed_retimed)
      {
        if (retime_fallback_on_failure_)
        {
          tcp_speed_retime_fallback = true;
          rt.setRobotTrajectoryMsg(start_state_core, planned);
          status_message = "Constant TCP-speed retime skipped/failed; using original Pilz plan: " + retime_error;
          RCLCPP_WARN(this->get_logger(),
                      "[Pilz SEQ] %s",
                      status_message.c_str());
        }
        else
        {
          status_message = "Constant TCP-speed retime failed and fallback is disabled: " + retime_error;
          RCLCPP_ERROR(this->get_logger(),
                       "[Pilz SEQ] %s",
                       status_message.c_str());
          res->success = false;
          res->message = status_message;
          return;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "[Pilz SEQ] Planned %zu sequence segment(s), combined trajectory points=%zu",
                resp.planned_trajectories.size(), planned.joint_trajectory.points.size());

    std::string final_validation_error;
    if (!validateTrajectoryTimingAndValues(planned, 1e-4, final_validation_error))
    {
      status_message = "Invalid final trajectory: " + final_validation_error;
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] %s", status_message.c_str());
      res->success = false;
      res->message = status_message;
      return;
    }

    const auto tcp_stats = computeTcpSpeedStats(rt, eef, tcp_speed_threshold_m_s);
    logJointVelocityAccelerationStats(robot_model, planned, "Pilz SEQ final trajectory");

    last_plan_pub_->publish(planned);

    RCLCPP_INFO(this->get_logger(),
                "[Pilz SEQ] TCP speed stats: path=%.4fm duration=%.3fs "
                "min=%.3fmm/s mean=%.3fmm/s max=%.3fmm/s low_samples=%u/%u threshold=%.3fmm/s",
                tcp_stats.path_length_m, tcp_stats.duration_s,
                tcp_stats.min_speed_m_s * 1000.0,
                tcp_stats.mean_speed_m_s * 1000.0,
                tcp_stats.max_speed_m_s * 1000.0,
                tcp_stats.low_sample_count, tcp_stats.sample_count,
                tcp_speed_threshold_m_s * 1000.0);

    if (viz_enabled_)
    {
      publishEEFPath(rt, eef, planning_frame);
    }

    res->trajectory = planned.joint_trajectory;
    res->success = true;
    res->message = status_message;
    res->tcp_speed_retimed = tcp_speed_retimed;
    res->tcp_speed_retime_fallback = tcp_speed_retime_fallback;
    res->tcp_path_length_m = tcp_stats.path_length_m;
    res->tcp_duration_s = tcp_stats.duration_s;
    res->tcp_speed_min_m_s = tcp_stats.min_speed_m_s;
    res->tcp_speed_mean_m_s = tcp_stats.mean_speed_m_s;
    res->tcp_speed_max_m_s = tcp_stats.max_speed_m_s;
    res->tcp_speed_sample_count = tcp_stats.sample_count;
    res->tcp_speed_low_sample_count = tcp_stats.low_sample_count;

    if (!req->preview_only)
    {
      if (exec_mode_ == "controller_action")
      {
        if (fjt_client_->wait_for_action_server(2s))
        {
          FJT::Goal exec_goal;
          exec_goal.trajectory = res->trajectory;
          fjt_client_->async_send_goal(exec_goal);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "[Pilz SEQ] Controller action server not available; skipping exec");
        }
      }
      else
      {
        moveit::planning_interface::MoveGroupInterface mgi_exec(shared_from_this(), req->group_name);
        moveit::planning_interface::MoveGroupInterface::Plan exec_plan;
        exec_plan.trajectory = planned;
        auto exec_code = mgi_exec.execute(exec_plan);
        if (!exec_code)
          RCLCPP_WARN(this->get_logger(), "[Pilz SEQ] Execution failed (code=%d)", exec_code.val);
      }
    }
  }

private:
  std::string exec_mode_;
  std::string controller_action_name_;
  bool viz_enabled_;
  std::string flange_link_;
  bool allow_tcp_retime_;
  bool retime_fallback_on_failure_;
  double max_tcp_speed_m_s_;
  bool log_sequence_joint_stats_;

  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr last_plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr targets_pub_;

  rclcpp::Service<behav3d_interfaces::srv::PlanWithMoveIt>::SharedPtr service_;
  rclcpp::Service<behav3d_interfaces::srv::PlanCartesianPath>::SharedPtr cartesian_srv_;
  rclcpp::Service<behav3d_interfaces::srv::PlanPilzLin>::SharedPtr pilz_lin_srv_;
  rclcpp::Service<behav3d_interfaces::srv::PlanPilzPtp>::SharedPtr pilz_ptp_srv_;
  rclcpp::Service<behav3d_interfaces::srv::PlanPilzSequence>::SharedPtr pilz_seq_srv_;
  rclcpp::Service<behav3d_interfaces::srv::GetLinkPose>::SharedPtr get_link_pose_srv_;
  rclcpp::Service<behav3d_interfaces::srv::PublishTargets>::SharedPtr publish_targets_srv_;
  rclcpp::Service<behav3d_interfaces::srv::DeleteMarkers>::SharedPtr delete_markers_srv_;
  rclcpp::Service<behav3d_interfaces::srv::UpdatePlanningSceneMesh>::SharedPtr
      update_planning_scene_mesh_srv_;

  rclcpp_action::Client<FJT>::SharedPtr fjt_client_;

  // Separate node and client for the MoveGroupSequence action
  rclcpp::Node::SharedPtr seq_client_node_;
  rclcpp_action::Client<MGS>::SharedPtr mgs_client_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState last_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

void MotionBridge::publishTargetsCallback(
  const std::shared_ptr<behav3d_interfaces::srv::PublishTargets::Request> req,
  std::shared_ptr<behav3d_interfaces::srv::PublishTargets::Response> res)
{
  const auto &targets = req->targets;
  if (req->clear_before) {
    visualization_msgs::msg::MarkerArray clear_arr;
    visualization_msgs::msg::Marker clear_m;
    clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_arr.markers.push_back(clear_m);
    targets_pub_->publish(clear_arr);
  }

  if (targets.empty()) {
    res->success = true;
    res->message = "No targets provided.";
    return;
  }

  const std::string frame_id = targets.front().header.frame_id.empty() ? "world" : targets.front().header.frame_id;
  const float axis_len = (req->axis_length > 0.0f) ? req->axis_length : 0.05f;
  const float axis_rad = (req->axis_radius > 0.0f) ? req->axis_radius : 0.003f;

  visualization_msgs::msg::Marker mx;
  visualization_msgs::msg::Marker my;
  visualization_msgs::msg::Marker mz;

  auto init_marker = [&](visualization_msgs::msg::Marker &m, const std::string &ns, float r, float g, float b)
  {
    m.header.frame_id = frame_id;
    m.header.stamp = this->now();
    m.ns = ns;
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = axis_rad;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.points.clear();
    m.points.reserve(targets.size() * 2);
  };

  init_marker(mx, "targets_x", 1.0f, 0.1f, 0.1f);
  init_marker(my, "targets_y", 0.1f, 1.0f, 0.1f);
  init_marker(mz, "targets_z", 0.1f, 0.1f, 1.0f);

  for (const auto &ps : targets)
  {
    const auto &p = ps.pose.position;
    const auto &q = ps.pose.orientation;

    Eigen::Quaterniond qe(q.w, q.x, q.y, q.z);
    if (qe.norm() < 1e-9) {
      qe = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    } else {
      qe.normalize();
    }
    Eigen::Matrix3d R = qe.toRotationMatrix();
    Eigen::Vector3d ex = R.col(0);
    Eigen::Vector3d ey = R.col(1);
    Eigen::Vector3d ez = R.col(2);

    geometry_msgs::msg::Point start;
    start.x = p.x; start.y = p.y; start.z = p.z;

    geometry_msgs::msg::Point endx = start;
    endx.x += axis_len * ex.x();
    endx.y += axis_len * ex.y();
    endx.z += axis_len * ex.z();

    geometry_msgs::msg::Point endy = start;
    endy.x += axis_len * ey.x();
    endy.y += axis_len * ey.y();
    endy.z += axis_len * ey.z();

    geometry_msgs::msg::Point endz = start;
    endz.x += axis_len * ez.x();
    endz.y += axis_len * ez.y();
    endz.z += axis_len * ez.z();

    mx.points.push_back(start);
    mx.points.push_back(endx);
    my.points.push_back(start);
    my.points.push_back(endy);
    mz.points.push_back(start);
    mz.points.push_back(endz);
  }

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(mx);
  arr.markers.push_back(my);
  arr.markers.push_back(mz);
  targets_pub_->publish(arr);

  res->success = true;
  res->message = "Published target axes.";
}

void MotionBridge::deleteMarkersCallback(
  const std::shared_ptr<behav3d_interfaces::srv::DeleteMarkers::Request> req,
  std::shared_ptr<behav3d_interfaces::srv::DeleteMarkers::Response> res)
{
  (void)req;
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker clear_m;
  clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear_m);
  targets_pub_->publish(arr);
  res->success = true;
  res->message = "Deleted all target markers.";
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
