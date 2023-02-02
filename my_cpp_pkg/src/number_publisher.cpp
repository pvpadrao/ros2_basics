#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberPublisherNode : public rclcpp::Node 
{
public:
    NumberPublisherNode() : Node("number_publisher"), number_(10)
    {
        // declaring a parameter
        this->declare_parameter("test_param", 2);

        // creating a publisher
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
                                        std::bind(&NumberPublisherNode::callback_number, this));
    
        RCLCPP_INFO(this->get_logger(), "Number Publisher has started!");
    }

private:
void callback_number()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_ ;
        publisher_->publish(msg);
    }
    int number_;
    // declaring the publisher
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    // declaring timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}