#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/srv/add_three_ints.hpp"

using namespace std::placeholders;
using simple_interfaces::srv::AddThreeInts;

class serviceCustomSrv : public rclcpp::Node {
public:
    serviceCustomSrv() : Node("service_custom_srv") {
        m_serv = this->create_service<AddThreeInts>("add_three_ints", 
            std::bind(&serviceCustomSrv::serverCallback, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Waiting for request...");
    }

private:
    void serverCallback(const std::shared_ptr<AddThreeInts::Request> request, 
                        const std::shared_ptr<AddThreeInts::Response> response) {
        response->sum = request->a + request->b + request->c;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a: '%ld', b: '%ld', c: '%ld", request->a, 
                    request->b, request->c);
        RCLCPP_INFO(this->get_logger(), "Sent result: '%ld'", response->sum);
    }

private:
    rclcpp::Service<AddThreeInts>::SharedPtr m_serv;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serviceCustomSrv>());
    rclcpp::shutdown();
    return 0;
}