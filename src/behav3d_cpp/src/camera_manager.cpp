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
#include "behav3d_cpp/util.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <ctime>
#include <cstdlib>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace fs = std::filesystem;
using std::placeholders::_1;
using std::placeholders::_2;

namespace behav3d::camera_manager
{

    bool CameraManager::writeSnapshot(const Snapshot &snap, FilePaths &out_paths, const std::string &stem_override)
    {
        const std::vector<int> png = {cv::IMWRITE_PNG_COMPRESSION, 0}; // no compression
        std::string stem;
        if (!stem_override.empty())
        {
            stem = stem_override;
        }
        else
        {
            const uint64_t idx = snap_seq_.fetch_add(1, std::memory_order_relaxed);
            stem = behav3d::util::indexString(idx, 3);
        }

        if (snap.has_ir && !snap.ir_raw.empty())
        {
            out_paths.ir = (dir_ir_ / (std::string("ir_") + stem + ".png")).string();
            cv::imwrite(out_paths.ir, snap.ir_raw, png);
        }
        if (snap.has_color && !snap.color_raw.empty())
        {
            out_paths.color = (dir_color_ / (std::string("color_") + stem + ".png")).string();
            cv::imwrite(out_paths.color, snap.color_raw, png);
        }
        if (snap.has_depth && !snap.depth_raw.empty())
        {
            out_paths.depth = (dir_depth_ / (std::string("depth_") + stem + ".png")).string();
            cv::imwrite(out_paths.depth, snap.depth_raw, png);
        }
        if (snap.has_d2c && !snap.d2c_depth.empty())
        {
            out_paths.d2c = (dir_d2c_ / (std::string("d2c_") + stem + ".png")).string();
            cv::imwrite(out_paths.d2c, snap.d2c_depth, png);
        }
        if (snap.has_c2d && !snap.c2d_color.empty())
        {
            out_paths.c2d = (dir_c2d_ / (std::string("c2d_") + stem + ".png")).string();
            cv::imwrite(out_paths.c2d, snap.c2d_color, png);
        }
        return true;
    }

    CameraManager::CameraManager(const rclcpp::NodeOptions &options)
        : rclcpp::Node("camera_manager", options)
    {
        initParams();
        initSubscriptions();
        initServices();

        // Start writer thread
        writer_ = std::thread(&CameraManager::writerThread, this);
    }

    CameraManager::~CameraManager()
    {
        running_.store(false);
        cv_.notify_all();
        if (writer_.joinable())
            writer_.join();
    }

    bool CameraManager::captureAsync()
    {
        Snapshot snap;
        if (!buildSnapshot(snap))
            return false;
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

    bool CameraManager::capture(const std::string &stem_override, FilePaths &out_paths, rclcpp::Time *stamp_out)
    {
        Snapshot snap;
        if (!buildSnapshot(snap))
            return false;
        RCLCPP_INFO(get_logger(), "Writing snapshot %s", timeStringFromStamp(snap.stamp).c_str());
        try
        {
            writeSnapshot(snap, out_paths, stem_override);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to write images: %s", e.what());
            return false;
        }
        if (stamp_out)
            *stamp_out = snap.stamp;
        return true;
    }

    void CameraManager::waitForIdle()
    {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this]()
                 { return queue_.empty(); });
    }

    void CameraManager::initParams()
    {
        ns_ = this->declare_parameter<std::string>("camera_ns", "/camera");
        output_dir_ = this->declare_parameter<std::string>("output_dir", "~/behav3d_captures");
        max_queue_ = static_cast<size_t>(this->declare_parameter<int>("max_queue", 8));

        color_topic_ = this->declare_parameter<std::string>("color_topic", ns_ + "/color/image_raw");
        depth_topic_ = this->declare_parameter<std::string>("depth_topic", ns_ + "/depth/image_raw");
        ir_topic_ = this->declare_parameter<std::string>("ir_topic", ns_ + "/ir/image_raw");
        color_info_topic_ = this->declare_parameter<std::string>("color_info_topic", ns_ + "/color/camera_info");
        depth_info_topic_ = this->declare_parameter<std::string>("depth_info_topic", ns_ + "/depth/camera_info");
        ir_info_topic_ = this->declare_parameter<std::string>("ir_info_topic", ns_ + "/ir/camera_info");

        d2c_depth_topic_ = this->declare_parameter<std::string>("d2c_depth_topic", ns_ + "/aligned_depth_to_color/image_raw");
        c2d_color_topic_ = this->declare_parameter<std::string>("c2d_color_topic", ns_ + "/aligned_color_to_depth/image_raw");

        RCLCPP_INFO(get_logger(), "CameraManager configured with ns='%s' output='%s'",
                    ns_.c_str(), output_dir_.c_str());
    }

    void CameraManager::initSubscriptions()
    {
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(20);

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

    void CameraManager::initServices()
    {
        srv_capture_ = this->create_service<std_srvs::srv::Trigger>(
            "capture",
            std::bind(&CameraManager::onCapture, this, _1, _2));
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

    void CameraManager::onCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
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

    // --- Calibration retrieval methods ---
    bool CameraManager::getCalibration(double timeout_sec, bool write_yaml)
    {
        sensor_msgs::msg::CameraInfo c, d, i;
        return getCalibration(c, d, i, timeout_sec, write_yaml);
    }

    bool CameraManager::getCalibration(sensor_msgs::msg::CameraInfo &color,
                                       sensor_msgs::msg::CameraInfo &depth,
                                       sensor_msgs::msg::CameraInfo &ir,
                                       double timeout_sec,
                                       bool write_yaml)
    {
        // Wait until all three CameraInfo messages are available, up to timeout
        const auto deadline = this->now() + rclcpp::Duration::from_seconds(timeout_sec);
        sensor_msgs::msg::CameraInfo::ConstSharedPtr csp, dsp, isp;
        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lk(mtx_);
                csp = last_color_info_;
                dsp = last_depth_info_;
                isp = last_ir_info_;
            }
            if (csp && dsp && isp)
                break;
            if (this->now() >= deadline)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!(csp && dsp && isp))
            return false;

        color = *csp;
        depth = *dsp;
        ir = *isp;

        if (write_yaml)
        {
            // Only write calibration YAMLs when an active session directory is set
            if (session_dir_.empty())
            {
                RCLCPP_WARN(get_logger(), "getCalibration(write_yaml=true): no active session_dir — skipping YAML write");
            }
            else
            {
                std::error_code ec;
                fs::create_directories(dir_calib_, ec);
                (void)ec;
                try
                {
                    const std::string f_color = (dir_calib_ / "color_intrinsics.yaml").string();
                    const std::string f_depth = (dir_calib_ / "depth_intrinsics.yaml").string();
                    const std::string f_ir = (dir_calib_ / "ir_intrinsics.yaml").string();

                    bool ok_color = writeCalibrationYaml(color, f_color);
                    bool ok_depth = writeCalibrationYaml(depth, f_depth);
                    bool ok_ir = writeCalibrationYaml(ir, f_ir);

                    if (ok_color && ok_depth && ok_ir)
                    {
                        RCLCPP_INFO(get_logger(), "Wrote camera info YAMLs to %s", dir_calib_.string().c_str());
                    }
                    else
                    {
                        if (!ok_color)
                            RCLCPP_ERROR(get_logger(), "Failed to write %s", f_color.c_str());
                        if (!ok_depth)
                            RCLCPP_ERROR(get_logger(), "Failed to write %s", f_depth.c_str());
                        if (!ok_ir)
                            RCLCPP_ERROR(get_logger(), "Failed to write %s", f_ir.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(get_logger(), "Exception while writing camera info YAMLs: %s", e.what());
                }
            }
        }
        return true;
    }

    bool CameraManager::buildSnapshot(Snapshot &out)
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
            color = last_color_;
            depth = last_depth_;
            d2c = last_d2c_depth_;
            c2d = last_c2d_color_;
            ci = last_color_info_;
            di = last_depth_info_;
            ii = last_ir_info_;
        }

        out.stamp = this->now();

        try
        {
            if (depth)
            {
                out.stamp = depth->header.stamp;
            }
            else if (color)
            {
                out.stamp = color->header.stamp;
            }
            else if (ir)
            {
                out.stamp = ir->header.stamp;
            }

            if (depth)
            {
                out.depth_raw = toUint16(*depth);
                out.has_depth = !out.depth_raw.empty();
                if (out.depth_raw.empty())
                {
                    out.has_depth = false;
                    RCLCPP_WARN(get_logger(), "buildSnapshot: DEPTH conversion EMPTY (encoding='%s')",
                                depth->encoding.c_str());
                }
                else
                {
                    RCLCPP_DEBUG(get_logger(), "buildSnapshot: DEPTH Mat %dx%d type=%d",
                                 out.depth_raw.rows, out.depth_raw.cols, out.depth_raw.type());
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "buildSnapshot: no DEPTH frame available at capture time");
            }

            if (ir)
            {
                out.ir_raw = toGray(*ir);
                out.has_ir = !out.ir_raw.empty();
                if (out.ir_raw.empty())
                {
                    out.has_ir = false;
                    RCLCPP_WARN(get_logger(), "buildSnapshot: IR conversion EMPTY (encoding='%s')",
                                ir->encoding.c_str());
                }
                else
                {
                    RCLCPP_DEBUG(get_logger(), "buildSnapshot: IR Mat %dx%d type=%d",
                                 out.ir_raw.rows, out.ir_raw.cols, out.ir_raw.type());
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "buildSnapshot: no IR frame available at capture time");
            }

            if (color)
            {
                out.color_raw = toBgr(*color);

                out.has_color = !out.color_raw.empty();
                if (out.color_raw.empty())
                {
                    out.has_color = false;
                    RCLCPP_WARN(get_logger(), "buildSnapshot: COLOR conversion EMPTY (encoding='%s')",
                                color->encoding.c_str());
                }
                else
                {
                    RCLCPP_DEBUG(get_logger(), "buildSnapshot: COLOR Mat %dx%d type=%d",
                                 out.color_raw.rows, out.color_raw.cols, out.color_raw.type());
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "buildSnapshot: no COLOR frame available at capture time");
            }

            if (d2c)
            {
                out.d2c_depth = toUint16(*d2c);
                out.has_d2c = !out.d2c_depth.empty();
                if (out.d2c_depth.empty())
                    out.has_d2c = false;
            }
            if (c2d)
            {
                out.c2d_color = toBgr(*c2d);
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

    // ============== Conversions ==============
    cv::Mat CameraManager::toBgr(const sensor_msgs::msg::Image &msg)
    {
        try
        {
            return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        }
        catch (const cv_bridge::Exception &)
        {
            // Best-effort fallback: if it's grayscale, expand to BGR
            try
            {
                auto any = cv_bridge::toCvCopy(msg)->image;
                if (!any.empty() && any.channels() == 1)
                {
                    cv::Mat bgr;
                    cv::cvtColor(any, bgr, cv::COLOR_GRAY2BGR);
                    return bgr;
                }
            }
            catch (...)
            {
                // ignore and return empty
            }
            return cv::Mat();
        }
    }

    cv::Mat CameraManager::toGray(const sensor_msgs::msg::Image &msg)
    {
        namespace enc = sensor_msgs::image_encodings;
        // Accept common 1-channel grayscale encodings
        if (msg.encoding == enc::MONO16)
        {
            return cv_bridge::toCvCopy(msg, enc::MONO16)->image;
        }
        if (msg.encoding == enc::MONO8)
        {
            return cv_bridge::toCvCopy(msg, enc::MONO8)->image;
        }
        if (msg.encoding == enc::TYPE_16UC1)
        {
            return cv_bridge::toCvCopy(msg, enc::TYPE_16UC1)->image; // treat as 16U mono
        }
        if (msg.encoding == enc::TYPE_8UC1)
        {
            return cv_bridge::toCvCopy(msg, enc::TYPE_8UC1)->image; // treat as 8U mono
        }
        return cv::Mat();
    }

    cv::Mat CameraManager::toUint16(const sensor_msgs::msg::Image &msg)
    {
        namespace enc = sensor_msgs::image_encodings;
        // 16-bit depth in millimeters
        if (msg.encoding == enc::TYPE_16UC1)
        {
            return cv_bridge::toCvCopy(msg, enc::TYPE_16UC1)->image;
        }
        // Some drivers expose depth as MONO16; accept it as-is
        if (msg.encoding == enc::MONO16)
        {
            return cv_bridge::toCvCopy(msg, enc::MONO16)->image;
        }
        // Depth in meters (float32) -> convert to uint16 millimeters
        if (msg.encoding == enc::TYPE_32FC1)
        {
            cv::Mat f = cv_bridge::toCvCopy(msg, enc::TYPE_32FC1)->image;
            if (f.empty())
                return cv::Mat();
            cv::patchNaNs(f, 0.0f);
            cv::Mat mm_f32 = f * 1000.0f; // meters -> millimeters
            cv::threshold(mm_f32, mm_f32, 65535.0, 65535.0, cv::THRESH_TRUNC);
            mm_f32.setTo(0.0f, mm_f32 < 0.0f); // clamp negatives
            cv::Mat mm_u16;
            mm_f32.convertTo(mm_u16, CV_16UC1);
            return mm_u16;
        }
        return cv::Mat();
    }

    // ============== Misc helpers ==============
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

    std::string CameraManager::indexString(uint64_t idx, int width)
    {
        std::ostringstream oss;
        oss << std::setw(width) << std::setfill('0') << idx;
        return oss.str();
    }

    bool CameraManager::writeCalibrationYaml(const sensor_msgs::msg::CameraInfo &info,
                                             const std::string &path)
    {
        YAML::Node node;
        node["image_width"] = info.width;
        node["image_height"] = info.height;
        node["camera_name"] = info.header.frame_id;

        auto makeMat = [](int rows, int cols, const auto &vec)
        {
            YAML::Node m;
            // OpenCV matrix header fields
            m["rows"] = rows;
            m["cols"] = cols;
            m["dt"] = "d"; // double
            YAML::Node data(YAML::NodeType::Sequence);
            for (const auto &v : vec)
                data.push_back(v);
            m["data"] = data;
            m.SetTag("opencv-matrix");
            return m;
        };

        node["camera_matrix"] = makeMat(3, 3, info.k);
        node["camera_matrix"];
        node["distortion_model"] = info.distortion_model;

        node["distortion_coefficients"] = makeMat(1, static_cast<int>(info.d.size()), info.d);
        node["distortion_coefficients"];

        node["rectification_matrix"] = makeMat(3, 3, info.r);
        node["rectification_matrix"];
        node["projection_matrix"] = makeMat(3, 4, info.p);
        node["projection_matrix"];

        return behav3d::util::writeYaml(path, node);
    }

    void CameraManager::writerThread()
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
                writeSnapshot(snap, paths);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(get_logger(), "Failed to write images: %s", e.what());
            }

            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (queue_.empty())
                    cv_.notify_all();
            }
        }
    }
    bool CameraManager::initSession(const std::string &session_dir, const std::string &tag)
    {
        (void)tag; // reserved for logging/metadata
        std::error_code ec;
        session_dir_ = fs::path(session_dir);
        fs::create_directories(session_dir_, ec);
        if (ec)
        {
            RCLCPP_ERROR(get_logger(), "Failed to create session_dir '%s': %s", session_dir_.string().c_str(), ec.message().c_str());
            return false;
        }
        dir_color_ = session_dir_ / "color_raw";
        dir_depth_ = session_dir_ / "depth_raw";
        dir_ir_ = session_dir_ / "ir_raw";
        dir_d2c_ = session_dir_ / "depth_to_color";
        dir_c2d_ = session_dir_ / "color_to_depth";
        dir_calib_ = session_dir_ / "calib";
        fs::create_directories(dir_color_, ec);
        fs::create_directories(dir_depth_, ec);
        fs::create_directories(dir_ir_, ec);
        fs::create_directories(dir_d2c_, ec);
        fs::create_directories(dir_c2d_, ec);
        fs::create_directories(dir_calib_, ec);

        snap_seq_.store(0, std::memory_order_relaxed);

        RCLCPP_INFO(get_logger(), "CameraManager bound to external session: %s", session_dir_.string().c_str());
        return true;
    }

} // namespace behav3d::camera_manager