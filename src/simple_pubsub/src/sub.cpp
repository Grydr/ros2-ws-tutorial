#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;
using std_msgs::msg::String;

class SimpleSubscriber : public rclcpp::Node {
public:
    SimpleSubscriber() : Node("simple_subscriber") {
        m_sub = this->create_subscription<String>("simple_pub", 10, 
                                                std::bind(&SimpleSubscriber::callback, this, _1));
    }

private:
    void callback(const String &msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg.data.c_str());
    }

private:
    rclcpp::Subscription<String>::SharedPtr m_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}