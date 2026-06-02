#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <optional>
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
                "(exec_mode=%s, controller_action=%s, flange_link=%s)",
                exec_mode_.c_str(), controller_action_name_.c_str(), flange_link_.c_str());
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
    if (req->targets.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] No targets provided");
      res->success = false;
      return;
    }

    // Use MoveGroupInterface only to query planning frame/model
    moveit::planning_interface::MoveGroupInterface mgi(shared_from_this(), req->group_name);
    const std::string planning_frame = mgi.getPlanningFrame();
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
      ps.header.frame_id = planning_frame;  // stamp in planning frame
      ps.pose = req->targets[i];

      moveit_msgs::msg::MotionPlanRequest mpr;
      mpr.group_name = req->group_name;
      mpr.pipeline_id = "pilz_industrial_motion_planner";
      mpr.planner_id = "LIN";
      mpr.max_velocity_scaling_factor = std::clamp<double>(req->velocity_scale, 0.0, 1.0);
      mpr.max_acceleration_scaling_factor = std::clamp<double>(req->accel_scale, 0.0, 1.0);

      if (i == 0)
        mpr.start_state = start_state_msg;

      // Pose goal for EEF
      auto gc = kinematic_constraints::constructGoalConstraints(eef, ps, 1e-3, 1e-3);
      mpr.goal_constraints.clear();
      mpr.goal_constraints.push_back(gc);

      moveit_msgs::msg::MotionSequenceItem item;
      item.req = mpr;
      item.blend_radius = (i < req->blend_radii.size()) ? std::max(0.0, req->blend_radii[i]) : 0.0;

      seq.items.push_back(item);
    }

    // Use the dedicated action client/node (avoid executor double-add)
    if (!mgs_client_->wait_for_action_server(3s))
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] sequence_move_group action not available");
      res->success = false;
      return;
    }

    MGS::Goal goal;
    goal.request = seq;
    goal.planning_options.plan_only = true;
    goal.planning_options.look_around = false;
    goal.planning_options.replan = false;

    auto send_future = mgs_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(seq_client_node_->get_node_base_interface(), send_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Action send failed");
      res->success = false;
      return;
    }

    auto gh = send_future.get();
    if (!gh)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Goal rejected");
      res->success = false;
      return;
    }

    auto result_future = mgs_client_->async_get_result(gh);
    if (rclcpp::spin_until_future_complete(seq_client_node_->get_node_base_interface(), result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Failed to get result");
      res->success = false;
      return;
    }

    auto wrapped_result = result_future.get();
    const auto &resp = wrapped_result.result->response;

    if (resp.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] Planning failed (code=%d)", resp.error_code.val);
      res->success = false;
      return;
    }
    if (resp.planned_trajectories.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "[Pilz SEQ] No trajectories returned");
      res->success = false;
      return;
    }

    const auto &planned = resp.planned_trajectories.back();
    last_plan_pub_->publish(planned);

    if (viz_enabled_)
    {
      robot_trajectory::RobotTrajectory rt(robot_model, req->group_name);
      rt.setRobotTrajectoryMsg(start_state_core, planned);
      publishEEFPath(rt, eef, planning_frame);
    }

    // Your srv carries only JointTrajectory + success
    res->trajectory = planned.joint_trajectory;
    res->success = true;

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
        exec_plan.trajectory = planned;  // RobotTrajectory
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
