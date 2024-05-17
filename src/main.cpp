#include <memory>
#include <string>
#include <chrono>

#include "rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "lidarlite_v3.h"

class LidarLiteNode : public rclcpp::Node, public LIDARLite_v3 {
  public:
    LidarLiteNode() : Node("lidarlite_node") {
      RCLCPP_INFO(this->get_logger(), "Initializing Garmin LiDAR-Lite v3");
      pub_message = this->create_publisher<std_msgs::msg::String>("lidarlite_v3", 10);

      timer = this->create_wall_timer(30ms, std::bind(&LidarLiteNode::reading_loop, this));
    }

  private:
    void reading_loop {

    }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLiteNode>());
  rclcpp::shutdown();
  return 0;
}
