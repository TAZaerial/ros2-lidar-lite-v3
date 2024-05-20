#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "lidarlite_srvs/srv/configure.hpp"

#include "lidarlite_v3.h"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class LidarLiteNode : public rclcpp::Node {
  public:
    LidarLiteNode() : Node("lidarlite_node") {
      RCLCPP_INFO(this->get_logger(), "Initializing Garmin LiDAR-Lite v3");

      this->declare_parameter<int>("lidarlite_v3/i2c_address", 0x62);

      lidarlite_i2c_addr = this->get_parameter("lidarlite_v3/i2c_address").as_int();
      
      lidar_flag = lidar.i2c_init();
      if (lidar_flag != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error initializing LiDAR-Lite v3 sensor");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "LiDAR-Lite v3 sensor initialized");
      if (lidarlite_i2c_addr != 0x62) {
        RCLCPP_INFO(this->get_logger(), "Setting LiDAR-Lite v3 I2C address to 0x%02X", lidarlite_i2c_addr);
        lidar.setI2Caddr(lidarlite_i2c_addr, true);
      }
      lidar.configure(0, lidarlite_i2c_addr);

      sensor_msg.header.frame_id = "lidarlite_v3";
      sensor_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      sensor_msg.field_of_view = 0.008;
      sensor_msg.min_range = 5;
      sensor_msg.max_range = 40;

      pub_range = this->create_publisher<sensor_msgs::msg::Range>("lidarlite_v3", 10);

      timer_sensor = this->create_wall_timer(30ms, std::bind(&LidarLiteNode::cb_sensor, this));

      ser_lidar = this->create_service<lidarlite_srvs::srv::Configure>("lidarlite_v3/configure", 
                                                                        std::bind(&LidarLiteNode::callback_srv_configure, this, _1, _2));
    }

  private:

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_range;
    rclcpp::TimerBase::SharedPtr timer_sensor;

    rclcpp::Service<lidarlite_srvs::srv::Configure>::SharedPtr ser_lidar;

    sensor_msgs::msg::Range sensor_msg;

    unsigned int lidarlite_i2c_addr;

    int lidar_flag; 
    LIDARLite_v3 lidar;

    void cb_sensor() {
      if (lidar.getBusyFlag()) {
        lidar.takeRange();
        sensor_msg.header.stamp = this->now();
        sensor_msg.range = lidar.readDistance();
        pub_range->publish(sensor_msg);
      }
    }

    void callback_srv_configure(const lidarlite_srvs::srv::Configure::Request::SharedPtr request,
                      const lidarlite_srvs::srv::Configure::Response::SharedPtr response) {
      lidar.configure(request->mode, lidarlite_i2c_addr);
      response->success = true;
    }                     

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarLiteNode>());
  rclcpp::shutdown();
  return 0;
}
