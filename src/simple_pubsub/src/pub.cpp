#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "simple_pubsub/fib.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher() : Node("simple_publisher") {
        // create publisher sharedptr obj
        m_pub = this->create_publisher<String>("simple_pub", 10);

        // declare parameter with default value 1
        this->declare_parameter<int>("fib_count", 1);
        
        // parameter validator
        m_paramOnSetHandle = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                return paramSetCallback(params);
            }
        );
        m_count = this->get_parameter("fib_count").as_int();

        // parameter listener to sync m_count with valid parameter
        m_paramEventHandle = std::make_shared<rclcpp::ParameterEventHandler>(this);
        m_paramEventCbHandle = m_paramEventHandle->add_parameter_callback("fib_count", 
            [this](const rclcpp::Parameter &param) {
                paramEventCallback(param);
            }
        );

        m_timer = this->create_wall_timer(500ms, [this]() { callback(); });
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

    rcl_interfaces::msg::SetParametersResult paramSetCallback(
        const std::vector<rclcpp::Parameter> &params) {

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : params) {
            if (param.get_name() == "fib_count" && param.as_int() < 1) {
                result.successful = false;
                result.reason = "'fib_count' value mus be >= 1";
            }
        }
        return result;
    }

    void paramEventCallback(const rclcpp::Parameter &param) {
        m_count = param.as_int();
        RCLCPP_INFO(this->get_logger(), "m_count changed to '%ld'", param.as_int());
    }

private:
    rclcpp::Publisher<String>::SharedPtr m_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_paramOnSetHandle;
    std::shared_ptr<rclcpp::ParameterEventHandler> m_paramEventHandle;
    rclcpp::ParameterCallbackHandle::SharedPtr m_paramEventCbHandle;
    int m_count;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}