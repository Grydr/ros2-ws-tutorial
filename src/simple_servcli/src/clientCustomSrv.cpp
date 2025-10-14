#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "simple_interfaces/srv/add_three_ints.hpp"

using namespace std::chrono_literals;
using simple_interfaces::srv::AddThreeInts;

class ClientCustomSrv : public rclcpp::Node {
public:
    ClientCustomSrv() : Node("client_custom_srv") {
        m_client = this->create_client<AddThreeInts>("add_three_ints");
    }

    rclcpp::Client<AddThreeInts>::SharedFuture asyncRequest(int64_t a, int64_t b, int64_t c) {
        auto request = std::make_shared<AddThreeInts::Request>();
        request->a = a;
        request->b = b;
        request->c = c;

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
    rclcpp::Client<AddThreeInts>::SharedPtr m_client;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "usage: add_two_ints_client A B C");
        return 1;
    }

    auto clientNode = std::make_shared<ClientCustomSrv>();
    int64_t a = std::atoll(argv[1]), b = std::atoll(argv[2]), c = std::atoll(argv[3]);
    
    auto logger = clientNode->get_logger();
    if (!clientNode->waitService()) {
        return 1;
    }
    auto future = clientNode->asyncRequest(a, b, c);
    
    auto returnCode = rclcpp::spin_until_future_complete(clientNode, future, 3s);
    if (returnCode == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(logger, "Get Response from service");
        RCLCPP_INFO(logger, "Sum: %ld", future.get()->sum);
    } else if (returnCode == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(logger, "Response timed out");
        clientNode->pruneRequest();
    } else {
        RCLCPP_ERROR(logger, "Response Interrupted");
        clientNode->pruneRequest();
    }
    
    rclcpp::shutdown();
    return 0;
}
