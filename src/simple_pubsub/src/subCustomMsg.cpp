#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/msg/num.hpp"

using Num = simple_interfaces::msg::Num;
using namespace std::placeholders;

class SubCustomMsg : public rclcpp::Node {
public:
    SubCustomMsg() : Node("sub_custom_msg") {
        m_sub = this->create_subscription<Num>("pub_custom_msg", 10, 
                                                std::bind(&SubCustomMsg::subCallback, this, _1));
    }

private:
    void subCallback(const Num::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Num: %ld", msg->num);
    }

private:
    rclcpp::Subscription<Num>::SharedPtr m_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubCustomMsg>());
    rclcpp::shutdown();
    return 0;
}