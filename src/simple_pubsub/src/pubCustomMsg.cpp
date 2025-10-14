#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/msg/num.hpp"

using namespace std::chrono_literals;
using Num = simple_interfaces::msg::Num;

class PubCustomMsg : public rclcpp::Node {
public:
    PubCustomMsg() : Node("pub_custom_msg"), m_count(0) {
        m_pub = this->create_publisher<Num>("pub_custom_msg", 10);
        m_timer = this->create_wall_timer(500ms, std::bind(&PubCustomMsg::timerCallback, this));
    }

private:
    void timerCallback() {
        Num msg;
        msg.num = m_count++;
        RCLCPP_INFO(this->get_logger(), "Published Num: %ld", msg.num);
        m_pub->publish(msg);
    }

private:
    rclcpp::Publisher<Num>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    int m_count;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubCustomMsg>());
    rclcpp::shutdown();
    return 0;
}