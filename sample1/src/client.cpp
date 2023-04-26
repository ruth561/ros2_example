#include <iostream>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_interface/action/range.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using my_interface::action::Range;

class Sample1Client : public rclcpp::Node {
public:
    Sample1Client() : Node("sample1_client") {
        /* Actionクライアントの生成 */
        action_client_ = rclcpp_action::create_client<Range>(
            this,
            "sample1"
        );

        /* Goalの返答を処理するコールバック関数 */
        auto goal_response_callback = [this](
            rclcpp_action::ClientGoalHandle<Range>::SharedPtr goal_handle
        ) {
            if (goal_handle) {
                std::cout << "Goal is accepted" << std::endl;
            } 
        };

        /* Feedbackが送られてきたときに呼ばれるコールバック関数 */
        auto feedback_callback = [this](
            rclcpp_action::ClientGoalHandle<Range>::SharedPtr goal_handle,
            const std::shared_ptr<const Range::Feedback> feedback
        ) {
            (void) goal_handle;
            std::cout << "[ feedback ] " << feedback->finished << std::endl;
        };

        /* Resultの処理をするコールバック */
        auto result_callback = [this](
            const rclcpp_action::ClientGoalHandle<Range>::WrappedResult &result
        ) {
            (void) result;
            std::cout << "Receive a result" << std::endl;
        };

        /* 送信するGoalリクエストの作成 */
        auto goal_msg = Range::Goal();
        goal_msg.begin = 100000000;
        goal_msg.end = 300000000;

        /* 各種コールバック関数の設定 */
        auto send_goal_options = rclcpp_action::Client<Range>::SendGoalOptions();
        send_goal_options.goal_response_callback = goal_response_callback;
        send_goal_options.feedback_callback = feedback_callback;
        send_goal_options.result_callback = result_callback;

        /* Goalの送信 */
        action_client_->async_send_goal(goal_msg, send_goal_options);
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

    rclcpp_action::Client<Range>::SharedPtr action_client_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto sample1_client = std::make_shared<Sample1Client>();
    rclcpp::spin(sample1_client);
    rclcpp::shutdown();
    return 0;
}