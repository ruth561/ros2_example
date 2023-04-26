#include <iostream>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interface/action/range.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using my_interface::action::Range;

class Sample1Server : public rclcpp::Node {
public:
    Sample1Server() : Node("server") {

        /* Goalのコールバック */
        auto goal_cb = [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Range::Goal> msg
        ) -> rclcpp_action::GoalResponse {
            std::cout << "invoked goal_cb" << std::endl;
            (void) uuid; // 変数uuidを使用しないことを明記する記法らしい
            std::cout << msg->begin << " ~ " << msg->end << std::endl;

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        /* Cancelのコールバック*/
        auto cancel_cb = [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<Range>> goal_handle
        ) -> rclcpp_action::CancelResponse {
            std::cout << "invoked cancel_cb" << std::endl;
            (void) goal_handle; // <- これ何？
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        /* Acceptedのコールバック */
        auto accepted_cb = [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<Range>> goal_handle
        ) -> void {
            std::cout << "invoked accepted_cb" << std::endl;
            (void) goal_handle; // <- これ何？
            std::thread {
                std::bind(&Sample1Server::execute, this, _1), goal_handle
            }.detach();
        };

        /* Actionサーバーの生成 */
        action_server_ = rclcpp_action::create_server<Range>(
            this,
            "sample1",
            goal_cb,
            cancel_cb,
            accepted_cb
        );
    }
private:
    /* Acceptedを受け取った後に実行される本体*/
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Range>> goal_handle) {
        std::cout << "invoked execute" << std::endl;
        const auto goal = goal_handle->get_goal();
        std::cout << "goal.begin = " << goal->begin << ", goal.end = " << goal->end << std::endl;

        auto result = std::make_shared<Range::Result>();
        
        /* 色々処理を行う。*/
        auto feedback = std::make_shared<Range::Feedback>();
        for (uint64_t i = goal->begin; i < goal->end; i++) {
            if (i % 10000000 == 0) {
                feedback->finished = i;
                goal_handle->publish_feedback(feedback);
                std::cout << "send feedback" << std::endl;
            }
        }

        result->find = true;
        result->value = (goal->begin + goal->end) / 2;
        goal_handle->succeed(result);
    }

    rclcpp_action::Server<Range>::SharedPtr action_server_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto sample1_server = std::make_shared<Sample1Server>();
    rclcpp::spin(sample1_server);
    rclcpp::shutdown();
    return 0;
}