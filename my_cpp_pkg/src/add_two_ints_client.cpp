#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node 
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {   // 1 & 4 are the input for the service. The two ints we want to sum.
        thread1_ = std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 1, 4));
        // creating new threads inside the thread pool to call service multiple times
        // threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 61, 4)));
        // threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 31, 4)));
        // threads_.push_back(std::thread(std::bind(&AddTwoIntsClientNode::callAddTwoIntsService, this, 21, 4)));

    }

    void callAddTwoIntsService(int a, int b)
    {
        
        // creating client node
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        // wait for the server to be ready
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(),"Waiting for the server to be up...");
        }
        // creating the request
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;
        // send the request asynchronously from the client to the server
        auto future_obj = client->async_send_request(request);

        try
        {   // this will pause the current thread at this point. spin(node) would not get processed 
            // The idea is to create another thread and keep it running
            auto response = future_obj.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)a, (int)b, (int)response->sum);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        
    }


private:
    std::thread thread1_;
    // to create a thread pool, uncomment the following line and delete the previous one:
    // std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}