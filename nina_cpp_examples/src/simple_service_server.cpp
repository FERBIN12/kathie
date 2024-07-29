#include <rclcpp/rclcpp.hpp>
#include <nina_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<nina_msgs::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Service server is ready to receive requests.");
    }

private:
    rclcpp::Service<nina_msgs::srv::AddTwoInts>::SharedPtr service_;   

void serviceCallback(std::shared_ptr<nina_msgs::srv::AddTwoInts::Request> req,
                     std::shared_ptr<nina_msgs::srv::AddTwoInts::Response> res)
{
    RCLCPP_INFO(get_logger(), "Received request: %ld + %ld", req->a, req->b);
    res->sum = req->a + req->b;
    RCLCPP_INFO(get_logger(), "Sending response: %ld", res->sum);

}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleServiceServer>());
  rclcpp::shutdown();
  return 0;
}