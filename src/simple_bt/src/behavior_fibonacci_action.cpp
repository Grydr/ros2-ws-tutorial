#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "simple_interfaces/action/fibonacci.hpp"

// let's define these, for brevity
using Fibonacci = simple_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

using namespace BT;

class FibonacciAction : public RosActionNode<Fibonacci> {
public:
    FibonacciAction(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<Fibonacci>(name, conf, params) {}

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosActionNode::providedBasicPorts()
    static PortsList providedPorts() { return providedBasicPorts({ InputPort<unsigned>("order") }); }

    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(RosActionNode::Goal& goal) override {
        // get "order" from the Input port
        getInput("order", goal.order);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    NodeStatus onResultReceived(const WrappedResult& wr) override {
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : wr.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(logger(), ss.str().c_str());
        return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override {
        RCLCPP_ERROR(logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    // we also support a callback for the feedback, as in
    // the original tutorial.
    // Usually, this callback should return RUNNING, but you
    // might decide, based on the value of the feedback, to abort
    // the action, and consider the TreeNode completed.
    // In that case, return SUCCESS or FAILURE.
    // The Cancel request will be send automatically to the server.
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(logger(), ss.str().c_str());
        return NodeStatus::RUNNING;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // in main()
    BehaviorTreeFactory factory;

    auto node = std::make_shared<rclcpp::Node>("fibonacci_action_client");
    // provide the ROS node and the name of the action service
    RosNodeParams params;
    params.nh = node;
    params.default_port_value = "fibonacci";

    // Register the custom BT node
    factory.registerNodeType<FibonacciAction>("Fibonacci", params);

    const std::string xml_text = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <Fibonacci order="5"/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";

    // Create the tree from the XML
    auto tree = factory.createTreeFromText(xml_text);

    RCLCPP_INFO(node->get_logger(), "Starting Behavior Tree...");

    // Tick the tree until the Sequence finishes (returns SUCCESS or FAILURE)
    tree.tickWhileRunning();

    RCLCPP_INFO(node->get_logger(), "Behavior Tree Finished. Shutting down.");

    rclcpp::shutdown();
    return 0;
}