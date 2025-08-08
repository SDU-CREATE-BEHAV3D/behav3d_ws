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
// Date: 2025-08
// =============================================================================

#include "behav3d_cpp/camera_manager.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <ctime>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace fs = std::filesystem;
using std::placeholders::_1;
using std::placeholders::_2;

namespace behav3d::camera
{

    // --- Disk I/O helpers
    std::vector<int> CameraManager::defaultPngParams()
    {
        return std::vector<int>{cv::IMWRITE_PNG_COMPRESSION, 3};
    }

    bool CameraManager::saveMatPng(const cv::Mat &img, const std::string &path, const std::vector<int> &png_params)
    {
        if (img.empty() || path.empty())
            return false;
        try
        {
            return cv::imwrite(path, img, png_params);
        }
        catch (const std::exception &e)
        {
            return false;
        }
    }

    bool CameraManager::saveSnapshotToDisk(const Snapshot &snap, FilePaths &out_paths)
    {
        const std::string ts = timeStringFromStamp(snap.stamp);
        const auto png = defaultPngParams();

        if (snap.has_ir && !snap.ir_raw.empty())
        {
            out_paths.ir = (fs::path(dir_ir_) / (ts + ".png")).string();
            saveMatPng(snap.ir_raw, out_paths.ir, png);
        }
        if (snap.has_color && !snap.color_raw.empty())
        {
            out_paths.color = (fs::path(dir_color_) / (ts + ".png")).string();
            saveMatPng(snap.color_raw, out_paths.color, png);
        }
        if (snap.has_depth && !snap.depth_raw.empty())
        {
            out_paths.depth = (fs::path(dir_depth_) / (ts + ".png")).string();
            saveMatPng(snap.depth_raw, out_paths.depth, png);
        }
        if (snap.has_d2c && !snap.d2c_depth.empty())
        {
            out_paths.d2c = (fs::path(dir_d2c_) / (ts + ".png")).string();
            saveMatPng(snap.d2c_depth, out_paths.d2c, png);
        }
        if (snap.has_c2d && !snap.c2d_color.empty())
        {
            out_paths.c2d = (fs::path(dir_c2d_) / (ts + ".png")).string();
            saveMatPng(snap.c2d_color, out_paths.c2d, png);
        }
        return true;
    }

    void CameraManager::tryDumpCameraInfos(const Snapshot &snap)
    {
        if (calib_dumped_)
            return;
        const bool have_all = (snap.color_info.k.size() == 9 &&
                               snap.depth_info.k.size() == 9 &&
                               snap.ir_info.k.size() == 9);
        if (!have_all)
            return;
        try
        {
            writeCameraInfoYaml(snap.color_info, (fs::path(dir_calib_) / "color_camera_info.yaml").string());
            writeCameraInfoYaml(snap.depth_info, (fs::path(dir_calib_) / "depth_camera_info.yaml").string());
            writeCameraInfoYaml(snap.ir_info, (fs::path(dir_calib_) / "ir_camera_info.yaml").string());
            calib_dumped_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Failed to write camera info yaml: %s", e.what());
        }
    }

    void CameraManager::appendManifest(const Snapshot &snap, const FilePaths &paths)
    {
        if (!write_json_manifest_)
            return;
        try
        {
            std::ofstream out(manifest_path_, std::ios::app);
            out << "{\"stamp_ns\":" << snap.stamp.nanoseconds()
                << ",\"ir\":" << (paths.ir.empty() ? "null" : "\"" + paths.ir + "\"")
                << ",\"color\":" << (paths.color.empty() ? "null" : "\"" + paths.color + "\"")
                << ",\"depth\":" << (paths.depth.empty() ? "null" : "\"" + paths.depth + "\"")
                << ",\"d2c\":" << (paths.d2c.empty() ? "null" : "\"" + paths.d2c + "\"")
                << ",\"c2d\":" << (paths.c2d.empty() ? "null" : "\"" + paths.c2d + "\"")
                << "}\n";
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "Failed to append manifest: %s", e.what());
        }
    }

    CameraManager::CameraManager(const rclcpp::NodeOptions &options)
        : rclcpp::Node("camera_manager", options)
    {
        setupParams();
        ensureSessionLayout();
        setupSubs();
        setupServices();

        // Start writer thread
        writer_ = std::thread(&CameraManager::writerThreadFn, this);

        if (passive_ir_on_start_)
        {
            // Call the camera driver to disable laser (passive IR)
            std::thread([this]()
                        {
      if (!cli_set_laser_) return;
      if (!cli_set_laser_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_WARN(get_logger(), "Laser service not available at startup: %s",
                    set_laser_service_name_.c_str());
        return;
      }
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

      req->data = false;
      auto fut = cli_set_laser_->async_send_request(req);
      (void)fut; })
                .detach();
        }
    }

    CameraManager::~CameraManager()
    {
        running_.store(false);
        cv_.notify_all();
        if (writer_.joinable())
            writer_.join();
    }

    void CameraManager::setupParams()
    {
        ns_ = this->declare_parameter<std::string>("camera_ns", "/camera");
        output_dir_ = this->declare_parameter<std::string>("output_dir", "~/behav3d_captures");
        want_c2d_ = this->declare_parameter<bool>("want_c2d", false);
        max_age_ms_ = this->declare_parameter<double>("max_snapshot_age_ms", 150.0);
        max_queue_ = static_cast<size_t>(this->declare_parameter<int>("max_queue", 8));
        write_json_manifest_ = this->declare_parameter<bool>("write_json_manifest", true);
        passive_ir_on_start_ = this->declare_parameter<bool>("passive_ir_on_start", false);

        color_topic_ = this->declare_parameter<std::string>("color_topic", ns_ + "/color/image_raw");
        depth_topic_ = this->declare_parameter<std::string>("depth_topic", ns_ + "/depth/image_raw");
        ir_topic_ = this->declare_parameter<std::string>("ir_topic", ns_ + "/ir/image_raw");
        color_info_topic_ = this->declare_parameter<std::string>("color_info_topic", ns_ + "/color/camera_info");
        depth_info_topic_ = this->declare_parameter<std::string>("depth_info_topic", ns_ + "/depth/camera_info");
        ir_info_topic_ = this->declare_parameter<std::string>("ir_info_topic", ns_ + "/ir/camera_info");

        d2c_depth_topic_ = this->declare_parameter<std::string>("d2c_depth_topic", ns_ + "/aligned_depth_to_color/image_raw");
        c2d_color_topic_ = this->declare_parameter<std::string>("c2d_color_topic", ns_ + "/aligned_color_to_depth/image_raw");

        set_laser_service_name_ = this->declare_parameter<std::string>("laser_enable_service", ns_ + "/set_laser_enable");

        RCLCPP_INFO(get_logger(), "CameraManager configured with ns='%s' output='%s'",
                    ns_.c_str(), output_dir_.c_str());
    }

    bool CameraManager::ensureSessionLayout()
    {
        auto out_root = expandUser(output_dir_);
        std::error_code ec;
        fs::create_directories(out_root, ec);
        if (ec)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create output_dir '%s': %s", out_root.c_str(), ec.message().c_str());
            return false;
        }

        // session dir
        auto now = this->now();
        session_dir_ = (fs::path(out_root) / (std::string("session-") + timeStringDateTime(now))).string();
        fs::create_directories(session_dir_, ec);
        RCLCPP_INFO(get_logger(), "Created session directory: %s", session_dir_.c_str());
        if (ec)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create session_dir '%s': %s", session_dir_.c_str(), ec.message().c_str());
            return false;
        }

        dir_color_ = (fs::path(session_dir_) / "color_raw").string();
        dir_depth_ = (fs::path(session_dir_) / "depth_raw").string();
        dir_ir_ = (fs::path(session_dir_) / "ir_raw").string();
        dir_d2c_ = (fs::path(session_dir_) / "depth_to_color").string();
        dir_c2d_ = (fs::path(session_dir_) / "color_to_depth").string();
        dir_calib_ = (fs::path(session_dir_) / "calib").string();
        manifest_path_ = (fs::path(session_dir_) / "captures.jsonl").string();

        fs::create_directories(dir_color_, ec);
        fs::create_directories(dir_depth_, ec);
        fs::create_directories(dir_ir_, ec);
        fs::create_directories(dir_d2c_, ec);
        fs::create_directories(dir_c2d_, ec);
        fs::create_directories(dir_calib_, ec);
        return true;
    }

    void CameraManager::setupSubs()
    {
        auto qos = rclcpp::SensorDataQoS();

        sub_color_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_topic_, qos, std::bind(&CameraManager::onColor, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to color: %s", color_topic_.c_str());
        sub_depth_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos, std::bind(&CameraManager::onDepth, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to depth: %s", depth_topic_.c_str());
        sub_ir_ = this->create_subscription<sensor_msgs::msg::Image>(
            ir_topic_, qos, std::bind(&CameraManager::onIr, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to IR: %s", ir_topic_.c_str());

        sub_color_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            color_info_topic_, qos, std::bind(&CameraManager::onColorInfo, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to color_info: %s", color_info_topic_.c_str());
        sub_depth_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            depth_info_topic_, qos, std::bind(&CameraManager::onDepthInfo, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to depth_info: %s", depth_info_topic_.c_str());
        sub_ir_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            ir_info_topic_, qos, std::bind(&CameraManager::onIrInfo, this, _1));
        RCLCPP_INFO(get_logger(), "Subscribed to ir_info: %s", ir_info_topic_.c_str());

        // Optional aligned topics (subscribe even if empty; callbacks just won't fire if not published)
        if (!d2c_depth_topic_.empty())
        {
            sub_d2c_ = this->create_subscription<sensor_msgs::msg::Image>(
                d2c_depth_topic_, qos, std::bind(&CameraManager::onDepthAlignedToColor, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribed to depth->color (aligned depth): %s", d2c_depth_topic_.c_str());
        }
        if (!c2d_color_topic_.empty())
        {
            sub_c2d_ = this->create_subscription<sensor_msgs::msg::Image>(
                c2d_color_topic_, qos, std::bind(&CameraManager::onColorAlignedToDepth, this, _1));
            RCLCPP_INFO(get_logger(), "Subscribed to color->depth (aligned color): %s", c2d_color_topic_.c_str());
        }
    }

    void CameraManager::setupServices()
    {
        srv_capture_ = this->create_service<std_srvs::srv::Trigger>(
            "capture",
            std::bind(&CameraManager::handleCapture, this, _1, _2));

        srv_passive_ir_ = this->create_service<std_srvs::srv::SetBool>(
            "set_passive_ir",
            std::bind(&CameraManager::handleSetPassiveIr, this, _1, _2));

        cli_set_laser_ = this->create_client<std_srvs::srv::SetBool>(set_laser_service_name_);
    }

    void CameraManager::onColor(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_color_ = msg;
    }

    void CameraManager::onDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_depth_ = msg;
    }

    void CameraManager::onIr(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ir_ = msg;
    }

    void CameraManager::onColorInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_color_info_ = msg;
    }

    void CameraManager::onDepthInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_depth_info_ = msg;
    }

    void CameraManager::onIrInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_ir_info_ = msg;
    }

    void CameraManager::onDepthAlignedToColor(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_d2c_depth_ = msg;
    }

    void CameraManager::onColorAlignedToDepth(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        last_c2d_color_ = msg;
    }

    bool CameraManager::captureAsync()
    {
        Snapshot snap;
        if (!makeSnapshot(snap))
        {
            return false;
        }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (queue_.size() >= max_queue_)
            {
                queue_.pop_front();
                RCLCPP_WARN(get_logger(), "Capture queue full — dropping oldest snapshot");
            }
            queue_.push_back(std::move(snap));
            RCLCPP_INFO(get_logger(), "Snapshot enqueued at %s", timeStringFromStamp(queue_.back().stamp).c_str());
        }
        cv_.notify_one();
        return true;
    }

    void CameraManager::waitForIdle()
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this]()
                 { return queue_.empty(); });
    }

    void CameraManager::handleCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        bool ok = captureAsync();
        res->success = ok;
        res->message = ok ? "Snapshot enqueued" : "Not ready: missing IR frame";
        if (ok)
        {
            RCLCPP_INFO(get_logger(), "Capture enqueued");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Capture not ready: missing IR frame");
        }
    }

    void CameraManager::handleSetPassiveIr(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                           std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        // Our API: data=true means passive (laser OFF). The camera's SetBool is laser_enable.
        if (!cli_set_laser_)
        {
            res->success = false;
            res->message = "laser service client not initialized";
            return;
        }
        if (!cli_set_laser_->wait_for_service(std::chrono::seconds(2)))
        {
            res->success = false;
            res->message = "laser service not available";
            return;
        }
        auto req_cam = std::make_shared<std_srvs::srv::SetBool::Request>();
        req_cam->data = !req->data; // passive -> disable laser
        try
        {
            auto fut = cli_set_laser_->async_send_request(req_cam);
            auto resp = fut.get();
            res->success = resp->success;
            res->message = resp->message;
        }
        catch (const std::exception &e)
        {
            res->success = false;
            res->message = std::string("service call failed: ") + e.what();
        }
    }

    bool CameraManager::makeSnapshot(Snapshot &out)
    {
        sensor_msgs::msg::Image::ConstSharedPtr color;
        sensor_msgs::msg::Image::ConstSharedPtr depth;
        sensor_msgs::msg::Image::ConstSharedPtr ir;
        sensor_msgs::msg::Image::ConstSharedPtr d2c;
        sensor_msgs::msg::Image::ConstSharedPtr c2d;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr ci, di, ii;

        {
            std::lock_guard<std::mutex> lk(mtx_);
            ir = last_ir_;
            if (!ir)
                return false; // IR required for hand-eye
            color = last_color_;
            depth = last_depth_;
            d2c = last_d2c_depth_;
            c2d = last_c2d_color_;
            ci = last_color_info_;
            di = last_depth_info_;
            ii = last_ir_info_;
        }

        try
        {
            out.stamp = ir->header.stamp;

            // IR first (required)
            out.ir_raw = toMono(*ir);
            out.has_ir = !out.ir_raw.empty();
            if (out.ir_raw.empty())
                out.has_ir = false;

            if (color)
            {
                out.color_raw = toColorBgr(*color);
                out.has_color = !out.color_raw.empty();
                if (out.color_raw.empty())
                    out.has_color = false;
            }
            if (depth)
            {
                out.depth_raw = depthToUint16(*depth);
                out.has_depth = !out.depth_raw.empty();
                if (out.depth_raw.empty())
                    out.has_depth = false;
            }
            if (d2c)
            {
                out.d2c_depth = depthToUint16(*d2c);
                out.has_d2c = !out.d2c_depth.empty();
                if (out.d2c_depth.empty())
                    out.has_d2c = false;
            }
            if (c2d)
            {
                out.c2d_color = toColorBgr(*c2d);
                out.has_c2d = !out.c2d_color.empty();
                if (out.c2d_color.empty())
                    out.has_c2d = false;
            }

            if (ci)
                out.color_info = *ci;
            if (di)
                out.depth_info = *di;
            if (ii)
                out.ir_info = *ii;

            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Snapshot conversion failed: %s", e.what());
            return false;
        }
    }

    // ============== Writer thread ==============
    void CameraManager::writerThreadFn()
    {
        while (running_.load())
        {
            Snapshot snap;
            {
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait(lk, [this]()
                         { return !queue_.empty() || !running_.load(); });
                if (!running_.load() && queue_.empty())
                    break;
                snap = std::move(queue_.front());
                queue_.pop_front();
            }
            RCLCPP_INFO(get_logger(), "Writing snapshot %s", timeStringFromStamp(snap.stamp).c_str());
            FilePaths paths{};
            try
            {
                saveSnapshotToDisk(snap, paths);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Failed to write images: %s", e.what());
            }
            RCLCPP_DEBUG(get_logger(), "Saved snapshot images to disk");

            tryDumpCameraInfos(snap);
            appendManifest(snap, paths);
            RCLCPP_DEBUG(get_logger(), "Appended snapshot to manifest");

            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (queue_.empty())
                {
                    RCLCPP_INFO(get_logger(), "Writer idle");
                    cv_.notify_all();
                }
            }
        }
    }

    // ============== Conversions ==============
    cv::Mat CameraManager::toColorBgr(const sensor_msgs::msg::Image &msg)
    {
        try {
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (const cv_bridge::Exception &) {
            // Best-effort fallback: if it's grayscale, expand to BGR
            try {
                auto any = cv_bridge::toCvCopy(msg)->image;
                if (!any.empty() && any.channels() == 1) {
                    cv::Mat bgr;
                    cv::cvtColor(any, bgr, cv::COLOR_GRAY2BGR);
                    return bgr;
                }
            } catch (...) {
                // ignore and return empty
            }
            return cv::Mat();
        }
    }

    cv::Mat CameraManager::toMono(const sensor_msgs::msg::Image &msg)
    {
        // Expect MONO16 or MONO8 from driver; anything else is skipped
        if (msg.encoding == sensor_msgs::image_encodings::MONO16)
        {
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16)->image;
        }
        if (msg.encoding == sensor_msgs::image_encodings::MONO8)
        {
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
        }
        return cv::Mat();
    }

    cv::Mat CameraManager::depthToUint16(const sensor_msgs::msg::Image &msg)
    {
        namespace enc = sensor_msgs::image_encodings;
        // Expect 16-bit depth in mm from driver
        if (msg.encoding == enc::TYPE_16UC1)
        {
            return cv_bridge::toCvCopy(msg, enc::TYPE_16UC1)->image;
        }
        // Some drivers might expose depth as MONO16; accept it as-is
        if (msg.encoding == enc::MONO16)
        {
            return cv_bridge::toCvCopy(msg, enc::MONO16)->image;
        }
        return cv::Mat();
    }

    // ============== Misc helpers ==============
    std::string CameraManager::expandUser(const std::string &path)
    {
        if (!path.empty() && path[0] == '~')
        {
            const char *home = std::getenv("HOME");
            if (home)
            {
                return std::string(home) + path.substr(1);
            }
        }
        return path;
    }

    std::string CameraManager::timeStringFromStamp(const rclcpp::Time &t)
    {
        int64_t nsec_total = t.nanoseconds();
        int64_t sec = nsec_total / 1000000000LL;
        int32_t nsec = static_cast<int32_t>(nsec_total - sec * 1000000000LL);
        std::time_t tt = static_cast<std::time_t>(sec);
        std::tm bt{};
#ifdef _WIN32
        localtime_s(&bt, &tt);
#else
        localtime_r(&tt, &bt);
#endif
        std::ostringstream oss;
        oss << std::put_time(&bt, "%Y%m%d-%H%M%S")
            << '-' << std::setw(9) << std::setfill('0') << nsec;
        return oss.str();
    }

    bool CameraManager::writeCameraInfoYaml(const sensor_msgs::msg::CameraInfo &info,
                                            const std::string &path)
    {
        std::ofstream f(path);
        if (!f.is_open())
            return false;
        f << "image_width: " << info.width << '\n';
        f << "image_height: " << info.height << '\n';
        f << "camera_name: '" << info.header.frame_id << "'\n";
        f << "camera_matrix:\n  rows: 3\n  cols: 3\n  data: [";
        for (size_t i = 0; i < info.k.size(); ++i)
        {
            f << std::setprecision(10) << info.k[i];
            if (i + 1 < info.k.size())
                f << ", ";
        }
        f << "]\n";
        f << "distortion_model: '" << info.distortion_model << "'\n";
        f << "distortion_coefficients:\n  rows: 1\n  cols: " << info.d.size() << "\n  data: [";
        for (size_t i = 0; i < info.d.size(); ++i)
        {
            f << std::setprecision(10) << info.d[i];
            if (i + 1 < info.d.size())
                f << ", ";
        }
        f << "]\n";
        f << "rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [";
        for (size_t i = 0; i < info.r.size(); ++i)
        {
            f << std::setprecision(10) << info.r[i];
            if (i + 1 < info.r.size())
                f << ", ";
        }
        f << "]\n";
        f << "projection_matrix:\n  rows: 3\n  cols: 4\n  data: [";
        for (size_t i = 0; i < info.p.size(); ++i)
        {
            f << std::setprecision(10) << info.p[i];
            if (i + 1 < info.p.size())
                f << ", ";
        }
        f << "]\n";
        return true;
    }

} // namespace behav3d::camera