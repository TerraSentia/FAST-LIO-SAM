#include "fast_lio_sam/fast_lio_sam_2.h"

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a shared pointer for the node
    auto node = std::make_shared<FastLioSam>();

    // Create a MultiThreadedExecutor for multithreaded spinning
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin the node until shutdown
    executor.spin();

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}
