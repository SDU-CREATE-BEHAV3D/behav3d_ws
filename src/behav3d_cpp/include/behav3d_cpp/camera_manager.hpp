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

#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <opencv2/core.hpp>

namespace behav3d::camera_manager
{

    class CameraManager : public rclcpp::Node
    {
    public:
        explicit CameraManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~CameraManager() override;

        // Paths written for a single snapshot (used by SessionManager manifest)
        struct FilePaths
        {
            std::string ir, color, depth, d2c, c2d;
        };

        // Bind to an externally-created session directory and disable internal JSONL writes.
        bool beginSession(const std::string &session_dir, const std::string &tag);

        // Synchronous capture: converts current frames and writes to disk before returning.
        // If stem_override (e.g., "t007") is non-empty, it is used as the filename stem.
        // If out_paths/stamp_out are provided, they are filled with the results.
        bool capture(const std::string &stem_override = "", FilePaths &out_paths, rclcpp::Time *stamp_out = nullptr);

        // Non-blocking: enqueue latest frames and return immediately
        bool captureAsync();

        // Block until the writer queue is empty
        void waitForIdle();

        // Retrieve calibration; optionally write YAMLs into the current session's calib/ dir.
        // Returns true if all three camera infos were obtained within timeout.
        bool getCalibration(double timeout_sec = 2.0, bool write_yaml = true);
        bool getCalibration(sensor_msgs::msg::CameraInfo &color,
                            sensor_msgs::msg::CameraInfo &depth,
                            sensor_msgs::msg::CameraInfo &ir,
                            double timeout_sec = 2.0,
                            bool write_yaml = true);

    private:
        // ==== ROS wiring
        void initParams();
        void initSubscriptions();
        void initServices();

        void onColor(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onIr(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onColorInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onDepthInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onIrInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onDepthAlignedToColor(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onColorAlignedToDepth(const sensor_msgs::msg::Image::ConstSharedPtr &);

        void onCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response>);

        // ==== Snapshot queue & writer
        struct Snapshot
        {
            rclcpp::Time stamp;
            cv::Mat color_raw; // BGR8
            cv::Mat depth_raw; // expected 16UC1 (millimeters) or MONO16 from driver
            cv::Mat ir_raw;    // MONO16 or MONO8 from driver
            cv::Mat d2c_depth; // 16UC1 (optional)
            cv::Mat c2d_color; // BGR8 (optional)

            sensor_msgs::msg::CameraInfo color_info;
            sensor_msgs::msg::CameraInfo depth_info;
            sensor_msgs::msg::CameraInfo ir_info;

            bool has_color = false;
            bool has_depth = false;
            bool has_ir = false;
            bool has_d2c = false;
            bool has_c2d = false;
        };

        // ---- Disk I/O helpers
        // If stem_override is non-empty, it will be used as the filename stem (e.g., "t007");
        // otherwise an auto-incrementing index like "003" is used.
        bool writeSnapshot(const Snapshot &snap, FilePaths &out_paths, const std::string &stem_override = "");

        bool buildSnapshot(Snapshot &out);

        void writerThread();

        // ==== Helpers
        static cv::Mat toBgr(const sensor_msgs::msg::Image &msg);
        static cv::Mat toGray(const sensor_msgs::msg::Image &msg);
        static cv::Mat toUint16(const sensor_msgs::msg::Image &msg);
        static std::string timeStringFromStamp(const rclcpp::Time &t);
        static std::string indexString(uint64_t idx, int width = 3);
        static bool writeCalibrationYaml(const sensor_msgs::msg::CameraInfo &info,
                                         const std::string &path);

        // ==== Params
        std::string ns_;
        std::string output_dir_;
        size_t max_queue_ = 8;

        // Topic params (derived from ns_, but overridable)
        std::string color_topic_;
        std::string depth_topic_;
        std::string ir_topic_;
        std::string color_info_topic_;
        std::string depth_info_topic_;
        std::string ir_info_topic_;
        std::string d2c_depth_topic_;
        std::string c2d_color_topic_;

        // ==== Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_ir_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_d2c_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_c2d_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_color_info_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_info_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_ir_info_;

        // ==== Services / clients
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_capture_;

        // ==== Latest messages
        std::mutex mtx_;
        sensor_msgs::msg::Image::ConstSharedPtr last_color_;
        sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
        sensor_msgs::msg::Image::ConstSharedPtr last_ir_;
        sensor_msgs::msg::Image::ConstSharedPtr last_d2c_depth_;
        sensor_msgs::msg::Image::ConstSharedPtr last_c2d_color_;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr last_color_info_;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr last_depth_info_;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr last_ir_info_;

        // ==== Writer state
        std::atomic<bool> running_{true};
        std::thread writer_;
        std::condition_variable cv_;
        std::deque<Snapshot> queue_;

        // ==== Paths
        std::filesystem::path session_dir_;
        std::filesystem::path dir_color_;
        std::filesystem::path dir_depth_;
        std::filesystem::path dir_ir_;
        std::filesystem::path dir_d2c_;
        std::filesystem::path dir_c2d_;
        std::filesystem::path dir_calib_;
        // (manifest_path_ removed)
        std::atomic<uint64_t> snap_seq_{0};
    };

} // namespace behav3d::camera_manager