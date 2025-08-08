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
#include <std_srvs/srv/set_bool.hpp>

#include <opencv2/core.hpp>

namespace behav3d::camera
{

    class CameraManager : public rclcpp::Node
    {
    public:
        explicit CameraManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~CameraManager() override;

        // Synchronous capture: converts current frames and writes to disk before returning
        bool capture();

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
        void setupParams();
        void setupSubs();
        void setupServices();

        void onColor(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onIr(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onColorInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onDepthInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onIrInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void onDepthAlignedToColor(const sensor_msgs::msg::Image::ConstSharedPtr &);
        void onColorAlignedToDepth(const sensor_msgs::msg::Image::ConstSharedPtr &);

        void handleCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response>);
        void handleSetPassiveIr(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
                                std::shared_ptr<std_srvs::srv::SetBool::Response>);

        // ==== Snapshot queue & writer
        struct Snapshot
        {
            rclcpp::Time stamp;
            cv::Mat color_raw; // BGR8
            cv::Mat depth_raw; // expected 16UC1 (millimeters) or MONO16 from driver
            cv::Mat ir_raw;    // 16UC1
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

        // Paths written for a single snapshot (used for manifest)
        struct FilePaths
        {
            std::string ir, color, depth, d2c, c2d;
        };

        // ---- Disk I/O helpers
        static std::vector<int> defaultPngParams();
        bool saveSnapshotToDisk(const Snapshot &snap, FilePaths &out_paths);
        static bool saveMatPng(const cv::Mat &img, const std::string &path, const std::vector<int> &png_params);
        void tryDumpCameraInfos(const Snapshot &snap);
        void appendManifest(const Snapshot &snap, const FilePaths &paths);

        bool makeSnapshot(Snapshot &out);

        void writerThreadFn();

        // ==== Helpers
        static cv::Mat toColorBgr(const sensor_msgs::msg::Image &msg);
        static cv::Mat toMono(const sensor_msgs::msg::Image &msg);
        static cv::Mat depthToUint16(const sensor_msgs::msg::Image &msg);
        static std::string expandUser(const std::string &path);
        static std::string timeStringFromStamp(const rclcpp::Time &t);
        static std::string timeStringDateTime(const rclcpp::Time &t);
        static std::string indexString(uint64_t idx, int width = 3);

        bool ensureSessionLayout();
        static bool writeCameraInfoYaml(const sensor_msgs::msg::CameraInfo &info,
                                        const std::string &path);

        // ==== Params
        std::string ns_;
        std::string output_dir_;
        bool want_c2d_ = false;
        double max_age_ms_ = 150.0;
        size_t max_queue_ = 8;
        bool write_json_manifest_ = true;
        bool passive_ir_on_start_ = false;

        // Topic params (derived from ns_, but overridable)
        std::string color_topic_;
        std::string depth_topic_;
        std::string ir_topic_;
        std::string color_info_topic_;
        std::string depth_info_topic_;
        std::string ir_info_topic_;
        std::string d2c_depth_topic_;
        std::string c2d_color_topic_;
        std::string set_laser_service_name_;

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
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_passive_ir_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr cli_set_laser_;

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
        std::string session_dir_;
        std::string dir_color_;
        std::string dir_depth_;
        std::string dir_ir_;
        std::string dir_d2c_;
        std::string dir_c2d_;
        std::string dir_calib_;
        std::string manifest_path_;
        bool calib_dumped_ = false;
        std::atomic<uint64_t> snap_seq_{0};
    };

} // namespace behav3d::camera