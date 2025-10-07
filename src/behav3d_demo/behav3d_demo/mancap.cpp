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

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <nlohmann/json.hpp>  
namespace fs = std::filesystem;

#include "behav3d_cpp/camera_manager.hpp"
#include "behav3d_cpp/motion_controller.hpp"
#include "behav3d_cpp/motion_visualizer.hpp"
#include "behav3d_cpp/target_builder.hpp"
#include "behav3d_cpp/trajectory_builder.hpp"
#include "behav3d_cpp/util.hpp"
#include <behav3d_interfaces/srv/compute_hand_eye.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <thread>
#include <atomic>

using behav3d::motion_controller::PilzMotionController;
using behav3d::motion_visualizer::MotionVisualizer;
using behav3d::camera_manager::CameraManager;

using behav3d::target_builder::flipTargetAxes;
using behav3d::target_builder::worldXY;
using behav3d::target_builder::worldXZ;
using behav3d::trajectory_builder::fibonacciSphericalCap;
using behav3d::trajectory_builder::sweepZigzag;
using behav3d::util::deg2rad;
using behav3d::trajectory_builder::addJitter;

using std::placeholders::_1;


class SessionLogger {
public:
  explicit SessionLogger(rclcpp::Node* node)
  : node_(node) {}

  // Call once at the beginning
  bool start(const std::string& base_frame,
             const std::string& tool0_frame,
             const std::string& output_root = "~/behav3d_ws/captures",
             const std::string& tag = "mancap")
  {
    base_frame_  = base_frame;
    tool0_frame_ = tool0_frame;

    // Expand ~
    std::string root = output_root;
    if (!root.empty() && root[0] == '~') {
      const char* home = std::getenv("HOME");
      if (home) root = std::string(home) + root.substr(1);
    }

    // Timestamp string (local time)
    const auto now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y%m%d_%H%M%S");

    // Create session dir
    session_dir_ = fs::path(root) / ("session-" + ts.str() + "_" + tag);
    std::error_code ec;
    fs::create_directories(session_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create session dir: %s (%s)",
                   session_dir_.string().c_str(), ec.message().c_str());
      return false;
    }

    // Open manifest
    manifest_path_ = session_dir_ / "manifest.json";
    ofs_.open(manifest_path_, std::ios::out | std::ios::trunc);
    if (!ofs_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open manifest: %s",
                   manifest_path_.string().c_str());
      return false;
    }

    // Build header WITHOUT "captures"
    nlohmann::json header;
    header["version"]          = 1;
    header["session_dir"]      = session_dir_.string();
    header["started_at_local"] = ts.str();
    header["frames"]           = { {"base", base_frame_}, {"tool0", tool0_frame_} };

    // Dump header prettily, then remove final '}' and append "captures":[
    std::string pre = header.dump(2);
    auto pos = pre.rfind('}');
    if (pos == std::string::npos) {
      RCLCPP_ERROR(node_->get_logger(), "Internal error building JSON header.");
      return false;
    }
    ofs_ << pre.substr(0, pos) << ",\n"
         << "  \"captures\": [\n";
    first_ = true;
    return true;
  }

  // Append one capture entry (color_path optional)
  void append(std::size_t index,
              const geometry_msgs::msg::PoseStamped& tool0_in_base,
              const rclcpp::Time& stamp_ns,
              const std::string& color_path = std::string())
  {
    if (!ofs_) return;

    nlohmann::json j;
    j["index"]    = index;
    j["stamp_ns"] = stamp_ns.nanoseconds();

    // Pose: base -> tool0
    j["pose_tool0"] = {
      {"frame_id", tool0_in_base.header.frame_id},
      {"child_frame_id", tool0_frame_},
      {"position", {
        tool0_in_base.pose.position.x,
        tool0_in_base.pose.position.y,
        tool0_in_base.pose.position.z
      }},
      {"orientation_xyzw", {
        tool0_in_base.pose.orientation.x,
        tool0_in_base.pose.orientation.y,
        tool0_in_base.pose.orientation.z,
        tool0_in_base.pose.orientation.w
      }}
    };

    if (!color_path.empty())
      j["image_color"] = color_path;

    // Stream with comma if needed
    if (!first_) ofs_ << ",\n";
    ofs_ << "    " << j.dump(2);
    first_ = false;
  }

  // Call once at the end
  void finish()
  {
    if (!ofs_) return;
    ofs_ << "\n  ]\n}\n"; // close "captures": [ ... ] and the root object
    ofs_.close();
    RCLCPP_INFO(node_->get_logger(), "Wrote manifest: %s", manifest_path_.string().c_str());
  }

  fs::path session_dir() const { return session_dir_; }

private:
  rclcpp::Node* node_;
  std::string base_frame_, tool0_frame_;
  fs::path session_dir_, manifest_path_;
  std::ofstream ofs_;
  bool first_ = true;
};

// ---------------------------------------------------------------------------
//                                Spawn handeye srv
// ---------------------------------------------------------------------------
// Start the installed Python solver (behav3d_py/handeye_solver) as a background process.
// Returns true if the spawn was attempted (even if the solver later exits on its own).
static std::atomic<bool> g_solver_running{false};

static bool spawn_handeye_solver() {
  try {
    const std::string prefix = ament_index_cpp::get_package_prefix("behav3d_py");
    const std::string exe    = prefix + "/lib/behav3d_py/handeye_solver";

    // Prevent multiple concurrent spawns
    bool expected = false;
    if (!g_solver_running.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(rclcpp::get_logger("mancap"), "handeye_solver already running; skipping spawn.");
      return true;
    }

    RCLCPP_INFO(rclcpp::get_logger("mancap"), "Spawning solver: %s", exe.c_str());
    std::thread([exe]{
      // Run solver; inherits this process' stdout/stderr (no redirection)
      std::system(exe.c_str());
      g_solver_running.store(false);
    }).detach();

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("mancap"), "Failed to locate behav3d_py: %s", e.what());
    return false;
  }
}

// ---------------------------------------------------------------------------
//                                Demo Node
// ---------------------------------------------------------------------------
class PilzDemo : public rclcpp::Node
{
public:
  explicit PilzDemo(const std::shared_ptr<PilzMotionController> &ctrl,
                     const std::shared_ptr<MotionVisualizer> &viz,
                    const std::shared_ptr<behav3d::camera_manager::CameraManager> &cam)
      : Node("pilz_demo_cpp"), ctrl_(ctrl), viz_(viz), cam_(cam)
  {

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&PilzDemo::callback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "PilzDemo ready. Commands: 'home', 'grid_sweep', 'fibonacci_cap', 'get_pose'");
  }

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<CameraManager> cam_;
  std::shared_ptr<MotionVisualizer> viz_;
  SessionLogger logger_{this}; 

  // ------------------------------------------------------------------------
  //  Command dispatcher
  // ------------------------------------------------------------------------
  void callback(const std_msgs::msg::String &msg)
  {
    const std::string cmd = msg.data;
    if (cmd == "home")
      home();
    else if (cmd == "grid_sweep")
      grid_sweep();
    else if (cmd == "fibonacci_cap")
      fibonacci_cap();
    else if (cmd == "get_pose")
      get_pose();
    else if (cmd == "quit")
      rclcpp::shutdown();
    else
      RCLCPP_WARN(this->get_logger(), "Unknown command '%s'", cmd.c_str());
  }

  // ------------------------------------------------------------------------
  //  Command implementations
  // ------------------------------------------------------------------------
  void home()
  {
    // Joint‑space “home” configuration (given in degrees)
    const std::vector<double> home_joints_deg = {-90.0, -120.0, 120.0, -90.0, 90.0, -150.0};

    // Convert degrees to radians for MoveIt
    std::vector<double> home_joints_rad;
    home_joints_rad.reserve(home_joints_deg.size());
    std::transform(home_joints_deg.begin(), home_joints_deg.end(),
                   std::back_inserter(home_joints_rad),
                   [](double deg)
                   { return deg * M_PI / 180.0; });

    // Plan a PTP joint motion and execute it
    auto traj = ctrl_->planJoints(home_joints_rad);
    ctrl_->executeTrajectory(traj);
  }

  // exact TF at given stamp (preferred)
  geometry_msgs::msg::PoseStamped get_pose(const rclcpp::Time& stamp)
  {
    const std::string base  = "ur20_base_link";
    const std::string tool0 = "ur20_tool0";

    const auto timeout = std::chrono::milliseconds(300);
    // small settle (optional)
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    if (!tf_buffer_->canTransform(base, tool0, stamp, timeout)) {
      RCLCPP_WARN(this->get_logger(), "TF not available: %s -> %s @ %ld ns",
                  base.c_str(), tool0.c_str(), stamp.nanoseconds());
      return geometry_msgs::msg::PoseStamped{};
    }

    auto tf = tf_buffer_->lookupTransform(base, tool0, stamp, timeout);

    geometry_msgs::msg::PoseStamped pose_tool0;
    pose_tool0.header = tf.header;          // frame_id = base, stamp = camera/asked stamp
    pose_tool0.header.frame_id = base;
    pose_tool0.pose.position.x = tf.transform.translation.x;
    pose_tool0.pose.position.y = tf.transform.translation.y;
    pose_tool0.pose.position.z = tf.transform.translation.z;
    pose_tool0.pose.orientation = tf.transform.rotation;
    auto ns = rclcpp::Time(pose_tool0.header.stamp).nanoseconds();

    RCLCPP_INFO(this->get_logger(),
                "[%s] Pose of '%s' in '%s' @ %ld\n"
                "  pos: [%.6f, %.6f, %.6f]\n"
                "  ori (xyzw): [%.6f, %.6f, %.6f, %.6f]",
                base.c_str(), tool0.c_str(), base.c_str(), ns,
                pose_tool0.pose.position.x, pose_tool0.pose.position.y, pose_tool0.pose.position.z,
                pose_tool0.pose.orientation.x, pose_tool0.pose.orientation.y,
                pose_tool0.pose.orientation.z, pose_tool0.pose.orientation.w);

    return pose_tool0;
  }

  // convenience: latest TF (keeps your existing call sites working)
  geometry_msgs::msg::PoseStamped get_pose()
  {
    return get_pose(rclcpp::Time(0)); // latest
  }


  void fibonacci_cap(double radius = 0.5,
                     double center_x = 0.0, double center_y = 0.75, double center_z = -0.075,
                     double cap_deg = 22.5, int n_points = 16)
  {
    home();
    const std::string base  = "ur20_base_link";
    const std::string tool0 = "ur20_tool0";
    if (!logger_.start(base, tool0, /*output_root*/"~/behav3d_ws/captures", /*tag*/"mancap"))
      return;
    cam_->initSession(logger_.session_dir(), "mancap");
    {
      sensor_msgs::msg::CameraInfo ci_color, ci_depth, ci_ir;
      const double timeout_sec = 2.0; 
      const bool write_yaml = true;
      if (!cam_->getCalibration(ci_color, ci_depth, ci_ir, timeout_sec, write_yaml)) {
        RCLCPP_WARN(this->get_logger(), "Did not get Camera info");
      } else {
        RCLCPP_INFO(this->get_logger(), "Intrinsics saved: %s/calib/{color,depth,ir}_intrinsics.yaml",
                    logger_.session_dir().string().c_str());
      }
    }
    const double cap_rad = deg2rad(cap_deg);
    const auto center = worldXY(center_x, center_y, center_z, ctrl_->getRootLink());

    viz_->publishTargetPose(center);
    auto targets = fibonacciSphericalCap(center, radius, cap_rad, n_points);
    targets = addJitter(targets, 0.01, 2.5);

    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "fibonacci_cap: no targets generated!");
      return;
    }

    viz_->publishTargetPose(targets);
    viz_->prompt("Press 'next' in RViz to start the cap scan");

    // --- Move PTP to the first point ---
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"));

    // --- t0 ---
    viz_->prompt("Press 'next' to LOG pose @ t0");

    // 1) capture
    behav3d::camera_manager::CameraManager::FilePaths files0{};
    rclcpp::Time stamp0 = this->now();
    std::string key0 = "t0";
    bool cap_ok0 = cam_->capture(key0, files0, &stamp0);

    // 2) TF at same stamp
    auto pose0 = get_pose(stamp0);

    // 3) log
    logger_.append(0, pose0, stamp0, cap_ok0 ? files0.color : "");

    // --- remaining targets ---
    for (size_t i = 1; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to move to target " + std::to_string(i));
      auto traj = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(traj);

      viz_->prompt("Press 'next' to LOG pose");

      behav3d::camera_manager::CameraManager::FilePaths files{};
      rclcpp::Time stamp = this->now();
      std::string key = "t" + std::to_string(i);
      bool cap_ok = cam_->capture(key, files, &stamp);

      auto pose_i = get_pose(stamp);               // <<< same timestamp
      logger_.append(i, pose_i, stamp, cap_ok ? files.color : "");
    }

    viz_->deleteAllMarkers();
    logger_.finish();

    home();
    
    // 1) Spawn the solver 
    if (!spawn_handeye_solver()) {
      RCLCPP_ERROR(this->get_logger(), "Could not spawn handeye_solver.");
      return;
    }
    // 2) Create the service client
    using behav3d_interfaces::srv::ComputeHandEye;
    auto client = this->create_client<ComputeHandEye>("compute_handeye");
    // 3) Wait briefly for the service to appear (up to ~5 seconds total)
    for (int i = 0; i < 50; ++i) {
      if (client->wait_for_service(std::chrono::milliseconds(100))) break;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    if (!client->service_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "Service '~/compute_handeye' not available.");
      return;
    }
    // 4) Build and send the request
    auto req = std::make_shared<ComputeHandEye::Request>();
    req->session_dir = logger_.session_dir().string();  // use the just-created session
    req->one_shot    = true;                            // solver will shut down after replying
    auto future = client->async_send_request(req);
    // 5) Optionally block until the result is ready (set a sensible timeout)
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "ComputeHandEye request timed out.");
      return;
    }
    const auto resp = future.get();
    if (!resp->ok) {
      RCLCPP_ERROR(this->get_logger(), "Hand-eye failed: %s", resp->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Hand-eye OK: t=[%.6f %.6f %.6f], q=[%.6f %.6f %.6f %.6f]\nJSON: %s",
      resp->t_m[0], resp->t_m[1], resp->t_m[2],
      resp->q_xyzw[0], resp->q_xyzw[1], resp->q_xyzw[2], resp->q_xyzw[3],
      resp->out_json.c_str());

  }

  void grid_sweep(double width = 1.0, double height = 0.5,
                  double center_x = 0.0, double center_y = 0.75, double center_z = 0.0,
                  double z_off = 0.75,
                  int nx = 10, int ny = 5,
                  bool row_major = false)
  {
    // 1. Return to a known joint configuration
    home();

    // 2. Build center pose and generate a zig‑zag raster pattern that
    const auto center = worldXY(center_x, center_y, center_z,
                                ctrl_->getRootLink());

    viz_->publishTargetPose(center);

    // Enforce a minimum of two waypoints per axis, per sweepZigzag’s contract.
    nx = std::max(2, nx);
    ny = std::max(2, ny);

    // z_off is fixed to 0 here because sweepZigzag now flips the targets internally.
    auto targets = sweepZigzag(center, width, height, z_off,
                               nx, ny, row_major);

    if (targets.empty())
    {
      RCLCPP_WARN(this->get_logger(), "grid_sweep/sweepZigzag: no targets generated!");
      return;
    }

    // 3. Visualize all targets
    viz_->publishTargetPose(targets);

    // 4. Prompt the user before starting motion
    viz_->prompt("Press 'next' in the RVizVisualToolsGui window to start grid scan");

    // 5. Move to the first target with a PTP, then traverse the rest with LIN
    ctrl_->executeTrajectory(ctrl_->planTarget(targets.front(), "PTP"));

    for (size_t i = 1; i < targets.size(); ++i)
    {
      viz_->prompt("Press 'next' to continue to target " + std::to_string(i));
      auto traj = ctrl_->planTarget(targets[i], "LIN");
      ctrl_->executeTrajectory(traj);
    }

    // 6. Clean up markers and return home
    viz_->deleteAllMarkers();
    home();
  }
  std::shared_ptr<PilzMotionController> ctrl_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

// ---------------------------------------------------------------------------
//                                   main()
// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions controller_opts;
  controller_opts.use_intra_process_comms(true);
  auto controller = std::make_shared<PilzMotionController>(controller_opts);

  rclcpp::NodeOptions visualizer_opts;
  visualizer_opts.use_intra_process_comms(true);
  auto visualizer = std::make_shared<MotionVisualizer>(visualizer_opts);
  
  rclcpp::NodeOptions camera_opts;
  camera_opts.use_intra_process_comms(true);
  auto camera = std::make_shared<behav3d::camera_manager::CameraManager>(camera_opts);

  auto demo = std::make_shared<PilzDemo>(controller, visualizer, camera);

  controller->set_parameters({
    rclcpp::Parameter("max_velocity_scale", 0.1),
    rclcpp::Parameter("max_accel_scale", 0.1),
  });

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(controller);
  exec.add_node(visualizer);
  exec.add_node(camera);
  exec.add_node(demo);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}