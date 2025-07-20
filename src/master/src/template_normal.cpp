#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

class TemplateNormal : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;

    TemplateNormal()
        : Node("template_normal")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TemplateNormal::callback_routine, this));

        logger.info("TemplateNormal node initialized");
    }

    ~TemplateNormal()
    {
    }

    void callback_routine()
    {
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_termplate_normal = std::make_shared<TemplateNormal>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_termplate_normal);
    executor.spin();

    return 0;
}