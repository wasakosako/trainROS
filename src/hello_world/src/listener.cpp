#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Listener :public rclcpp::Node
{
  public:
    explicit Listener(const std::string & topic_name)
    :Node("listener")
    {
      auto callback=
      [this](const std_msgs::msg::String::UniquePtr msg)->void
      {
        RCLCPP_INFO(this->get_logger(),"%s",msg->data.c_str());
      };
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      sub_ = create_subscription<std_msgs::msg::String>(
        topic_name, qos, callback);
    }
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <topic_name>\n", argv[0]);
    return 1;
  }
  auto node = std::make_shared<Listener>(argv[1]);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}