#include "rclcpp/rclcpp.hpp"
#include "lo_dev/Frontend/frontend.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions options;
    options.arguments({"frontend_node"});
    
    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    std::shared_ptr<lo_dev::Frontend> frontend = std::make_shared<lo_dev::Frontend>(options);

    frontend->initializeInterface();

    // std::cout << __FUNCTION__ << __LINE__ << std::endl;

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(frontend);
    executor.spin();
    // rclcpp::spin(featureExtraction->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
