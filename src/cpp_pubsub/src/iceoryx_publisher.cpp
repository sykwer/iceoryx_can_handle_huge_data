#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <sys/time.h>

#include "interfaces/msg/static_size_array.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    //auto qos = rclcpp::QoS(1).best_effort().durability_volatile().keep_last(1);
    publisher_ = this->create_publisher<interfaces::msg::StaticSizeArray>("topic", 1);
    timer_ = this->create_wall_timer(3000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std::make_unique<interfaces::msg::StaticSizeArray>();
    message->id = count_++;

    RCLCPP_INFO(this->get_logger(), "Publishing Message ID: '%ld'", message->id);

    struct timeval tv;
    gettimeofday(&tv, NULL);
    message->timestamp = tv.tv_sec * 1000 * 1000 + tv.tv_usec;

    publisher_->publish(std::move(message));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::StaticSizeArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
