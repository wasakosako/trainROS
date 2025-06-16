#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const std::string & topic_name)
  : Node("listener")
  {
    // chatterトピックのコールバック関数
    auto callback =
      [this](const std_msgs::msg::String::UniquePtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
      };

    // chatterトピックの受信設定
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    sub_ = create_subscription<std_msgs::msg::String>(
      topic_name, qos, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};