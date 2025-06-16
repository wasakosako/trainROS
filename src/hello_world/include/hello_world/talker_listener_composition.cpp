#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "talker_no_main.cpp"
#include "listener_no_main.cpp"

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto talker = std::make_shared<Talker>("chatter");
  auto listener = std::make_shared<Listener>("chatter");
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}