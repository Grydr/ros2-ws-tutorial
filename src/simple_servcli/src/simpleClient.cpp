#include <chrono>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using example_interfaces::srv::AddTwoInts;

class SimpleClient : public rclcpp::Node {
public:
    SimpleClient() : Node("simple_client") {
        m_client = this->create_client<AddTwoInts>("add_two_ints");
    }

    rclcpp::Client<AddTwoInts>::SharedFuture asyncRequest(int64_t a, int64_t b) {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto result = m_client->async_send_request(request);
        return result;
    }

    void pruneRequest() {
        std::vector<int64_t> prunedRequest{};
        std::size_t nPruned = m_client->prune_requests_older_than(
            std::chrono::system_clock::now() - 3s, &prunedRequest);
        if (nPruned) {
            RCLCPP_INFO(
                this->get_logger(),
                "The server hasn't replied for more than 3s, %zu requests were discarded, "
                "the discarded requests numbers are:",
                nPruned);    
            for (const auto &req : prunedRequest) {
                RCLCPP_INFO(this->get_logger(), "\t%ld", req);
            }
        }
    }

    bool waitService() {
        while (!m_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(this->get_logger(),
                        "service not available, waiting again...");
        }
        return true;
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr m_client;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "usage: add_two_ints_client X Y");
        return 1;
    }

    auto simpleClient = std::make_shared<SimpleClient>();
    int64_t a = std::atoll(argv[1]), b = std::atoll(argv[2]);
    
    auto logger = simpleClient->get_logger();
    if (!simpleClient->waitService()) {
        return 1;
    }
    auto future = simpleClient->asyncRequest(a, b);
    
    auto returnCode = rclcpp::spin_until_future_complete(simpleClient, future, 3s);
    if (returnCode == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(logger, "Get Response from service");
        RCLCPP_INFO(logger, "Sum: %ld", future.get()->sum);
    } else if (returnCode == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(logger, "Response timed out");
        simpleClient->pruneRequest();
    } else {
        RCLCPP_ERROR(logger, "Response Interrupted");
        simpleClient->pruneRequest();
    }
    
    rclcpp::shutdown();
    return 0;
}
