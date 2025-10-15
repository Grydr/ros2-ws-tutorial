#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "simple_pubsub/fib.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using std_msgs::msg::String;

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher() : Node("simple_publisher"), m_count(1) {
        // create publisher sharedptr obj
        m_pub = this->create_publisher<String>("simple_pub", 10);
        m_timer = this->create_wall_timer(500ms, std::bind(&SimplePublisher::callback, this));
    }

private:
    void callback() {
        std::vector<int> fibList = createFibonacci(m_count);

        std::string list{};
        for (const auto &num : fibList) {
            list += std::to_string(num) + " ";
        }

        String msg;
        msg.data = list;

        RCLCPP_INFO(this->get_logger(), "Published: '%s'", list.c_str());
        m_pub->publish(msg);
        ++m_count;
    }

private:
    rclcpp::Publisher<String>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    int m_count;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}