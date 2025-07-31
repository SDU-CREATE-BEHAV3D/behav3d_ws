#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("demo_node");
  RCLCPP_INFO(node->get_logger(), "Nodo demo funcionando!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
