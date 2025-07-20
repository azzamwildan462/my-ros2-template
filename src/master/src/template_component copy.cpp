#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/global_definitions.hpp"

class TemplateComponent : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;

    HelpLogger logger;

    explicit TemplateComponent(const rclcpp::NodeOptions &options)
        : Node("template_component", options)
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            throw std::runtime_error("Error");
        }

        tim_routine = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&TemplateComponent::callback_routine, this));

        logger.info("TemplateComponent node initialized");
    }

    ~TemplateComponent()
    {
    }

    void callback_routine()
    {
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(TemplateComponent)