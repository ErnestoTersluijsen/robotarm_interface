#include "RosInterface.hpp"

#include <rclcpp/rclcpp.hpp>

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RosInterface>());
	rclcpp::shutdown();
	return 0;
}