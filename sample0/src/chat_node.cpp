#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "my_interface/msg/chat_msg.hpp"

using namespace std::chrono_literals;

class ChatNode : public rclcpp::Node {
public:
    ChatNode() : Node("ChatNode") {
        std::cout << "put your name > ";
        std::cin >> name_;

        /* Publisherの作成*/
        pub_ = this->create_publisher<my_interface::msg::ChatMsg>("chat_topic", 1);
        
        /* Subscriberの作成 */
        auto recv_msg = [this](my_interface::msg::ChatMsg::SharedPtr msg) -> void {
            std::cout << "[" << msg->name << "] " << msg->data << std::endl; 
        };
        sub_ = this->create_subscription<my_interface::msg::ChatMsg>("chat_topic", 1, recv_msg);
    }

    /* メッセージを送るようのスレッドで動くメソッド */
    void run() {
        while (rclcpp::ok()) {
            std::string input;
            std::getline(std::cin, input);

            if (!input.empty()) {
                auto msg = std::make_unique<my_interface::msg::ChatMsg>();
                msg->name = name_;
                msg->data = input;
                pub_->publish(std::move(msg));
            }
        }
    }

private:
    std::string name_;
    my_interface::msg::ChatMsg msg_;
    rclcpp::Publisher<my_interface::msg::ChatMsg>::SharedPtr pub_;
    rclcpp::Subscription<my_interface::msg::ChatMsg>::SharedPtr sub_;
};


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChatNode>();
    std::thread([&]() {
        node->run();        
    }).detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}