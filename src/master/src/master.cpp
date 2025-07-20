#include "master/master.hpp"

Master::Master(const rclcpp::NodeOptions &options)
    : Node("master", options)
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        throw std::runtime_error("Error");
    }

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Master::callback_routine, this));

    pub_ui_test = this->create_publisher<std_msgs::msg::String>("ui_test", 10);

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_routine()
{
    process_transmitter();
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Master)