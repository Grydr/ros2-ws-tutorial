#include <functional>
#include <memory>
#include <thread>

#include "simple_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::placeholders;

class SimpleActionServer : public rclcpp::Node {
public:
    using Fibonacci = simple_interfaces::action::Fibonacci;
    using GoalHandleFib = rclcpp_action::ServerGoalHandle<Fibonacci>;

    explicit SimpleActionServer() : Node("simple_action_server") {
        m_actionServer = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&SimpleActionServer::goalHandle, this, _1, _2),
            std::bind(&SimpleActionServer::cancelHandle, this, _1),
            std::bind(&SimpleActionServer::acceptHandle, this, _1)
        );
    }

    rclcpp_action::GoalResponse goalHandle(const rclcpp_action::GoalUUID &uuid, 
                                            const std::shared_ptr<const Fibonacci::Goal> &goal) {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received goal request with order: %d", goal->order);
        if (goal->order > 100) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancelHandle(const std::shared_ptr<GoalHandleFib> &handle) {
        (void)handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void acceptHandle(const std::shared_ptr<GoalHandleFib> &handle) {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&SimpleActionServer::execute, this, _1), handle}.detach();
    }

private:
    void execute(const std::shared_ptr<GoalHandleFib> &handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        rclcpp::Rate loopRate{1};
        const auto goal = handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto &partialSequence = feedback->partial_sequence;
        partialSequence.push_back(0);
        partialSequence.push_back(1);
        auto result = std::make_shared<Fibonacci::Result>();

        for (int i = 1; (i < goal->order) && rclcpp::ok(); i++) {
            // Check if there is a cancel request
            if (handle->is_canceling()) {
                result->sequence = partialSequence;
                handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal cancelled");
                return;
            }

            // update partialsequence
            partialSequence.push_back(partialSequence.at(i) + partialSequence.at(i-1));

            // publish feedback
            handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish Feedback");

            loopRate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->sequence = partialSequence;
            handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
    }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr m_actionServer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleActionServer>());
    rclcpp::shutdown();
    return 0;
}