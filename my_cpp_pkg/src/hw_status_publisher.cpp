#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisher : public rclcpp::Node 
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher")
    {
        // creating a publisher
        publisher_ = create_publisher<robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                        std::bind(&HardwareStatusPublisher::publishHardwareStatus, this));
    
        RCLCPP_INFO(this->get_logger(), "Hardware status publisher has started!");
    }

private:
    void publishHardwareStatus()
        {
            auto msg = robot_interfaces::msg::HardwareStatus();
            msg.temperature = 55;
            msg.are_motors_ready = false;
            msg.debug_message = "Related Info";
            publisher_->publish(msg);
        }
        
        // declaring the publisher
        rclcpp::Publisher<robot_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
        // declaring timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}