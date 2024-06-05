#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stdio.h>

using namespace std::chrono_literals;

class JSNSR04TNode : public rclcpp::Node
{
public:
  JSNSR04TNode() : Node("jsn_sr04t_node")
  {
    // 파라미터 선언 및 기본값 설정
    this->declare_parameter<std::string>("device", "/dev/ttySOFTa0");
    this->declare_parameter<std::string>("topic", "front_left");

    this->get_parameter("device", device_);
    this->get_parameter("topic", topic_);
	
	  RCLCPP_INFO(this->get_logger(), "device : %s",device_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic : %s",topic_.c_str());

    // 시리얼 포트 초기화
    try
    {
      serial_.open(device_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
      throw;
    }

    range_publisher_ = this->create_publisher<sensor_msgs::msg::Range>(topic_, 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&JSNSR04TNode::publish_range, this));

    start_byte_ = 0xFF;
  }

private:
  void publish_range()
  {
    std::vector<uint8_t> buffer(1);
    bool start_byte_found = false;
	// RCLCPP_INFO(this->get_logger(), "publish == ");
    flush_serial_port();

    while (boost::asio::read(serial_, boost::asio::buffer(buffer)) == 1)
    {
      if (buffer[0] == start_byte_)
      {
        start_byte_found = true;
        break;
      }
    }

    if (start_byte_found)
    {
      buffer.resize(3);
      boost::asio::read(serial_, boost::asio::buffer(buffer));

      uint16_t high_byte = buffer[0];
      uint16_t low_byte = buffer[1];
      uint8_t checksum = buffer[2];

      uint16_t distance = (high_byte << 8) | low_byte;

      if (checksum == ((high_byte + low_byte) & 0xFF))
      {
        if (distance == 0) {
          // RCLCPP_WARN(this->get_logger(), "Fail Range Data : %d %d", high_byte, low_byte);
          // return;
          distance = 6000;
        }


        auto message = sensor_msgs::msg::Range();
        message.header.stamp = this->now();
        message.header.frame_id = topic_;
        message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        message.field_of_view = 0.1; // 10 degrees
        message.min_range = 0.2;    // 20 cm
        message.max_range = 4.00;    // 4 meters
        message.range = std::min<float>(std::max<float>((distance / 1000.0) - 0.05, 0.2), 4.0);

        range_publisher_->publish(message);
		    // RCLCPP_INFO(this->get_logger(), "range : %f", message.range);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Checksum mismatch");
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Start byte not found");
    }
  }

  void flush_serial_port()
  {
    int fd = serial_.native_handle();
    if (tcflush(fd, TCIFLUSH) != 0)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to flush serial port");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  boost::asio::io_service io_;
  boost::asio::serial_port serial_{io_};
  uint8_t start_byte_;

  std::string device_;
  std::string topic_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JSNSR04TNode>());
  rclcpp::shutdown();
  return 0;
}
