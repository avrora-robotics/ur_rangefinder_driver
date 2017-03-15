#include "rangefinder_node.h"

constexpr uint8_t RangefinderNode::PREAMBLE[PREAMBLE_SIZE];
constexpr uint8_t RangefinderNode::ENDING[ENDING_SIZE];

RangefinderNode::RangefinderNode(ros::NodeHandle &node_handle, std::shared_ptr<serial::Serial> serial_port)
  : serial_port_(serial_port)
{
  range_publisher_ = node_handle.advertise<sensor_msgs::Range>("/range", 1000);
}

void RangefinderNode::update()
{
  auto bytes_available = serial_port_->available();
  if (bytes_available < PACKET_SIZE) {
    return;
  }

  static uint8_t current_packet_size = 0;
  while (bytes_available--) {
    uint8_t current_byte;
    serial_port_->read(&current_byte, 1);

    if (current_packet_size < PREAMBLE_SIZE) {
      if (current_byte == PREAMBLE[current_packet_size]) {
        current_packet_size++;
      } else {
        current_packet_size = 0;
      }

      continue;
    }

    if (current_packet_size < PREAMBLE_SIZE + DATA_SIZE) {
      float range_value = static_cast<float>(current_byte) * 0.1;

      if (current_byte == NO_SENSOR_CONNECTED) {
        range_value = current_byte;
      } else if (range_value > MAXIMUM_RANGE_VALUE) {
        range_value = MAXIMUM_RANGE_VALUE;
      }

      current_packet_.ranges[current_packet_size - PREAMBLE_SIZE] = range_value;
      current_packet_size++;

      continue;
    }

    if (current_packet_size < PREAMBLE_SIZE + DATA_SIZE + ENDING_SIZE) {
      if (current_byte == ENDING[current_packet_size - PREAMBLE_SIZE - DATA_SIZE]) {
        current_packet_size++;
      } else {
        ROS_WARN_STREAM(ros::this_node::getName() << "Final symbols are incorrect! Packet will be dropped");
        current_packet_size = 0;
      }

      continue;
    }

    current_packet_.offset = current_byte;
    sendCurrentPacket();

    current_packet_size = 0;
  }

}

void RangefinderNode::sendCurrentPacket()
{
  for (uint8_t range = 0; range < DATA_SIZE; ++range) {
    uint8_t index = current_packet_.offset * DATA_SIZE + range + 1;
    if (current_packet_.ranges[range] == NO_SENSOR_CONNECTED) {
      ROS_WARN_STREAM(ros::this_node::getName() << "Rangefinder #" << index << " is not connected");
      continue;
    }

    sendRangeMessage(index, current_packet_.ranges[range]);
  }
}

void RangefinderNode::sendRangeMessage(uint8_t rangefinder_index, float data)
{
  sensor_msgs::Range message;
  message.header.stamp = ros::Time::now();
  message.header.frame_id = "rangefinder_" + std::to_string((uint8_t)rangefinder_index);

  message.min_range = MINIMUM_RANGE_VALUE;
  message.max_range = MAXIMUM_RANGE_VALUE;

  message.field_of_view = 10 * M_PI / 180.0; // approx 10 degrees
  message.radiation_type = sensor_msgs::Range::ULTRASOUND;
  message.range = data;

  range_publisher_.publish(message);
}
