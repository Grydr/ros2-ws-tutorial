#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;
using example_interfaces::srv::AddTwoInts;

class SimpleService : public rclcpp::Node {
public:
    SimpleService() : Node("simple_service") {
        m_serv = this->create_service<AddTwoInts>("add_two_ints", 
            std::bind(&SimpleService::serverCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Waiting for request...");
    }

private:
    void serverCallback(const std::shared_ptr<AddTwoInts::Request> request, 
                        const std::shared_ptr<AddTwoInts::Response> response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a: '%ld', b: '%ld'", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Response result: '%ld'", response->sum);
    }

private:
    rclcpp::Service<AddTwoInts>::SharedPtr m_serv;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleService>());
    rclcpp::shutdown();
    return 0;
}