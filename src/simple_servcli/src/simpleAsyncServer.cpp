/*
    STILL WIP PROBABLY SHOULD NOT USE
*/

#include <chrono>
#include <functional>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using example_interfaces::srv::AddTwoInts;

class SimpleAsyncClient : public rclcpp::Node {
public:
    SimpleAsyncClient() : Node("simple_async_service") {
        m_client = this->create_client<AddTwoInts>("add_two_ints");
        m_pruneTimer = this->create_wall_timer(5s, std::bind(&SimpleAsyncClient::pruneRequest, this));
    }

    void asyncRequest(int64_t a, int64_t b) {
        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        m_client->async_send_request(request, 
                                    std::bind(&SimpleAsyncClient::handleResponse,
                                    this, _1));
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
    using ServiceResponseFuture =
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;
    void handleResponse(ServiceResponseFuture future) {
        auto responsePair = future.get();
        RCLCPP_INFO(
            this->get_logger(),
            "Response sum: %ld",
            responsePair.second->sum);  
        rclcpp::shutdown();
    }

    void pruneRequest() {
        std::vector<int64_t> prunedRequest{};
        std::size_t nPruned = m_client->prune_requests_older_than(
            std::chrono::system_clock::now() - 5s, &prunedRequest);
        if (nPruned) {
            RCLCPP_INFO(
                this->get_logger(),
                "The server hasn't replied for more than 5s, %zu requests were discarded, "
                "the discarded requests numbers are:",
                nPruned);    
            for (const auto &req : prunedRequest) {
                RCLCPP_INFO(this->get_logger(), "\t%ld", req);
            }
        }
    }

private:
    rclcpp::Client<AddTwoInts>::SharedPtr m_client;
    rclcpp::TimerBase::SharedPtr m_pruneTimer;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "usage: add_two_ints_client X Y");
        return 1;
    }

    auto simpleAsyncClient = std::make_shared<SimpleAsyncClient>();
    int64_t a = std::atoll(argv[1]), 
            b = std::atoll(argv[2]);
            
    if (!simpleAsyncClient->waitService()) {
        return 1;
    }
    simpleAsyncClient->asyncRequest(a, b);

    rclcpp::spin(simpleAsyncClient);
    
    rclcpp::shutdown();
    return 0;
}
