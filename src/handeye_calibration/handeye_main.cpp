#include "behav3d_cpp/handeye_calibration.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<behav3d::handeye::HandeyeCalibration>();
    const bool ok = node->run();
    rclcpp::shutdown();
    return ok ? 0 : 1;
}
