#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rp_homework/srv/serviceinterface.hpp"
#include "rp_homework/action/actioninterface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <thread>

using Serviceinterface = rp_homework::srv::Serviceinterface;
using Actioninterface = rp_homework::action::Actioninterface;

class Node2 : public rclcpp::Node
{
public:
  Node2() : Node("node2"), student_id_("2023741042")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "student_id", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received ID digit: %s", msg->data.c_str());
      });

    service_ = this->create_service<Serviceinterface>(
      "multiply_two_ints",
      [this](const std::shared_ptr<Serviceinterface::Request> request,
             std::shared_ptr<Serviceinterface::Response> response) {
        response->result = request->a * request->b;
        RCLCPP_INFO(this->get_logger(), "Service: %ld * %ld = %ld",
                    request->a, request->b, response->result);
      });

    action_server_ = rclcpp_action::create_server<Actioninterface>(
      this, "sumnumber",
      std::bind(&Node2::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Node2::handle_cancel, this, std::placeholders::_1),
      std::bind(&Node2::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, const std::shared_ptr<const Actioninterface::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Actioninterface>>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Actioninterface>> goal_handle)
  {
    std::thread(
      [this, goal_handle]() {
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Actioninterface::Result>();
        auto feedback = std::make_shared<Actioninterface::Feedback>();
        int32_t total = 0;
        for (int i = 0; i < goal->goal; ++i) {
          total += student_id_[i] - '0';
          feedback->feedback = total;
          goal_handle->publish_feedback(feedback);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        result->result = total;
        goal_handle->succeed(result);
      }).detach();
  }

  std::string student_id_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<Serviceinterface>::SharedPtr service_;
  rclcpp_action::Server<Actioninterface>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Node2>());
  rclcpp::shutdown();
  return 0;
}
