#include <functional>
#include <memory>
#include <thread>

#include "simple_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class SimpleActionClient : public rclcpp::Node {
public:
    using Fibonacci = simple_interfaces::action::Fibonacci;
    using GoalHandleFib = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit SimpleActionClient() : Node("simple_action_client"), m_goalDone(false) {
        m_actionClient = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci"
        );

        m_timer = this->create_wall_timer(500ms, 
            std::bind(&SimpleActionClient::sendGoal, this));
    }

    bool isGoalDone() {
        return m_goalDone;
    }

    void sendGoal() {
        m_timer->cancel();

        if (!m_actionClient) {
            RCLCPP_ERROR(this->get_logger(), "Action Client not initialized");
            return;
        }

        if (!m_actionClient->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            m_goalDone = true;
            return;
        }

        Fibonacci::Goal goalMsg{};
        goalMsg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        rclcpp_action::Client<Fibonacci>::SendGoalOptions sendGoalOptions{};
        sendGoalOptions.goal_response_callback = 
            std::bind(&SimpleActionClient::goalResponseCallback, this, _1);
        sendGoalOptions.feedback_callback = 
            std::bind(&SimpleActionClient::feedbackCallback, this, _1, _2);
        sendGoalOptions.result_callback = 
            std::bind(&SimpleActionClient::resultCallback, this, _1);

        auto goalHandleFuture = m_actionClient->async_send_goal(goalMsg, sendGoalOptions);
    }

private:
    void goalResponseCallback(const std::shared_ptr<GoalHandleFib> &handle) {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(const std::shared_ptr<GoalHandleFib> &handle, 
        const std::shared_ptr<const Fibonacci::Feedback> &fb) {
        (void)handle;
        RCLCPP_INFO(this->get_logger(), 
            "Received next number in sequence %d", 
            fb->partial_sequence.back());
    }

    void resultCallback(const GoalHandleFib::WrappedResult &result) {
        using ResultCode = rclcpp_action::ResultCode;
        m_goalDone = true;
        switch(result.code) {
            case ResultCode::SUCCEEDED:
                break;
            case ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was cancelled");
                return;
            case ResultCode::UNKNOWN:
                RCLCPP_ERROR(this->get_logger(), "Goal state unkown");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Result Received");
        for (const auto &num : result.result->sequence) {
            RCLCPP_INFO(this->get_logger(), "%d", num);
        }
    }

private:
    std::shared_ptr<rclcpp_action::Client<Fibonacci>> m_actionClient;
    std::shared_ptr<rclcpp::TimerBase> m_timer;
    bool m_goalDone;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto actionClient = std::make_shared<SimpleActionClient>();
    while (!actionClient->isGoalDone()) {
        rclcpp::spin_some(actionClient);
    }

    rclcpp::shutdown();
    return 0;
}