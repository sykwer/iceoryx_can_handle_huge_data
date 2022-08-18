#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "interfaces/msg/static_size_array.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<interfaces::msg::StaticSizeArray>(
      "topic", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const interfaces::msg::StaticSizeArray::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard message ID: '%ld'", msg->id);
  }

  rclcpp::Subscription<interfaces::msg::StaticSizeArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
