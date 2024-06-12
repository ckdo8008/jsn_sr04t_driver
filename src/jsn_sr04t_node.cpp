#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stdio.h>

using namespace std::chrono_literals;

class JSNSR04TNode : public rclcpp::Node
{
public:
  JSNSR04TNode() : Node("jsn_sr04t_node"),
                   serial_(io_),
                   deadline_timer_(io_),
                   filtered_distance_(0.0), // Initialize filtered distance
                   alpha_(0.5) // Initialize alpha for LPF
  {
    // 파라미터 선언 및 기본값 설정
    this->declare_parameter<std::string>("device", "/dev/ttySOFTa0");
    this->declare_parameter<std::string>("topic", "front_left");
    this->declare_parameter<float>("alpha", 0.5); // LPF alpha parameter

    this->get_parameter("device", device_);
    this->get_parameter("topic", topic_);
    this->get_parameter("alpha", alpha_);
    
    RCLCPP_INFO(this->get_logger(), "device : %s", device_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic : %s", topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "alpha : %f", alpha_);

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

    ros_timer_ = this->create_wall_timer(
        100ms, std::bind(&JSNSR04TNode::publish_range, this));

    start_byte_ = 0xFF;
  }

private:
  void publish_range()
  {
    std::vector<uint8_t> buffer(4);
    flush_serial_port();

    // 데이터 전송
    int8_t startdata = 0x55;
    boost::asio::write(serial_, boost::asio::buffer(&startdata, 1));
    rclcpp::sleep_for(std::chrono::milliseconds(80));

    // Asynchronous read with timeout
    read_completed_ = false;

    boost::asio::async_read(serial_, boost::asio::buffer(buffer),
                            [this, &buffer](const boost::system::error_code &ec, std::size_t length)
                            {
                              if (!ec)
                              {
                                handle_read(ec, length, buffer);
                                read_completed_ = true;
                                deadline_timer_.cancel(); // Cancel the timer if read completes
                              }
                            });

    deadline_timer_.expires_from_now(boost::posix_time::milliseconds(20)); // Set timeout to 20ms
    deadline_timer_.async_wait([this](const boost::system::error_code &ec)
                               {
                                 if (!ec && !read_completed_)
                                 {
                                   // Timer expired before read completed
                                   serial_.cancel();
                                   RCLCPP_WARN(this->get_logger(), "Read operation timed out : %s", topic_.c_str());
                                 }
                               });

    io_.run();
    io_.reset();
  }

  void handle_read(const boost::system::error_code &ec, std::size_t length, std::vector<uint8_t> &buffer)
  {
    if (!ec)
    {
      if (length == 4 && buffer[0] == start_byte_)
      {
        uint16_t high_byte = buffer[1];
        uint16_t low_byte = buffer[2];
        uint8_t checksum = buffer[3];

        uint16_t distance = (high_byte << 8) | low_byte;
        if (checksum == ((start_byte_ + high_byte + low_byte) & 0xFF))
        {
          if (distance == 0)
          {
            RCLCPP_WARN(this->get_logger(), "%s Fail Range Data : %d %d", topic_.c_str(), high_byte, low_byte);
            distance = 6000; // Fail-safe distance in mm
          }

          // Apply LPF to the distance
          float current_distance_m = distance / 1000.0; // Convert to meters
          filtered_distance_ = alpha_ * current_distance_m + (1 - alpha_) * filtered_distance_;

          auto message = sensor_msgs::msg::Range();
          message.header.stamp = this->now();
          message.header.frame_id = topic_;
          message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          message.field_of_view = 0.1; // 10 degrees
          message.min_range = 0.2;     // 20 cm
          message.max_range = 4.00;    // 4 meters
          message.range = std::min<float>(std::max<float>(filtered_distance_, 0.2), 4.0);

          range_publisher_->publish(message);
          RCLCPP_INFO(this->get_logger(), "%s range Data : %f", topic_.c_str(), filtered_distance_);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "%s Checksum mismatch", topic_.c_str());
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "%s Start byte not found or invalid data length", topic_.c_str());
      }
    }
    else if (ec != boost::asio::error::operation_aborted)
    {
      RCLCPP_WARN(this->get_logger(), "Error on receive: %s", ec.message().c_str());
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
  rclcpp::TimerBase::SharedPtr ros_timer_;

  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  boost::asio::deadline_timer deadline_timer_;
  uint8_t start_byte_;
  bool read_completed_;

  std::string device_;
  std::string topic_;
  float alpha_;
  float filtered_distance_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JSNSR04TNode>());
  rclcpp::shutdown();
  return 0;
}
