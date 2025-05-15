#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rp_homework/srv/serviceinterface.hpp"
#include "rp_homework/action/actioninterface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using Serviceinterface = rp_homework::srv::Serviceinterface;
using Actioninterface = rp_homework::action::Actioninterface;

class Node1 : public rclcpp::Node
{
public:
  Node1() : Node("node1"), idx_(0), student_id_("2023741042")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("student_id", 10);

    timer_ = this->create_wall_timer(1s, std::bind(&Node1::publish_id_digit, this));

    client_ = this->create_client<Serviceinterface>("multiply_two_ints");
    while (!client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }
    send_service_request(77, 11);

    action_client_ = rclcpp_action::create_client<Actioninterface>(this, "sumnumber");
    while (!action_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }
    send_action_goal();
  }

private:
  void publish_id_digit()
  {
    if (idx_ < student_id_.size()) {
      auto msg = std_msgs::msg::String();
      msg.data = student_id_.substr(idx_, 1);
      publisher_->publish(msg);
      idx_++;
    }
  }

  void send_service_request(int64_t a, int64_t b)
  {
    auto request = std::make_shared<Serviceinterface::Request>();
    request->a = a;
    request->b = b;
    auto result_future = client_->async_send_request(request,
      [this](rclcpp::Client<Serviceinterface>::SharedFuture response) {
        RCLCPP_INFO(this->get_logger(), "Service result: %ld", response.get()->result);
      });
  }

  void send_action_goal()
  {
    Actioninterface::Goal goal_msg;
    goal_msg.goal = static_cast<int32_t>(student_id_.size());

    auto send_goal_options = rclcpp_action::Client<Actioninterface>::SendGoalOptions();
    send_goal_options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<Actioninterface>::SharedPtr,
             const std::shared_ptr<const Actioninterface::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Action feedback: %d", feedback->feedback);
      };
    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<Actioninterface>::WrappedResult & result) {
        RCLCPP_INFO(this->get_logger(), "Action result: %d", result.result->result);
      };
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  std::string student_id_;
  size_t idx_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Client<Serviceinterface>::SharedPtr client_;
  rclcpp_action::Client<Actioninterface>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node1>());
  rclcpp::shutdown();
  return 0;
}
