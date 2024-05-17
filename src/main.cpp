#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "lidarlite_v3.h"

using namespace std::chrono_literals;

using std::placeholders::_1;

class LidarLiteNode : public rclcpp::Node, public LIDARLite_v3 {
  public:
    LidarLiteNode() : Node("lidarlite_node") {
      RCLCPP_INFO(this->get_logger(), "Initializing Garmin LiDAR-Lite v3");
      pub_range = this->create_publisher<sensor_msgs::msg::Range>("lidarlite_v3", 10);

      timer_sensor = this->create_wall_timer(30ms, std::bind(&LidarLiteNode::reading_loop, this));
    }

  private:

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range;
    rclcpp::TimerBase::SharedPtr timer_sensor;

    void reading_loop() {
      return;
    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLiteNode>());
  rclcpp::shutdown();
  return 0;
}
