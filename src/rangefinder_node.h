#ifndef UR_RANGEFINDER_NODE_H
#define UR_RANGEFINDER_NODE_H

#include "ros/ros.h"
#include "serial/serial.h"
#include <sensor_msgs/Range.h>

struct RangefinderPacket
{
  float ranges[4] = {0};
  uint8_t offset = 0;
};

class RangefinderNode
{

public:
  RangefinderNode(ros::NodeHandle& node_handle, std::shared_ptr<serial::Serial> serial_port);
  void update();

private:
  static const size_t PACKET_SIZE = 9;
  static const size_t PREAMBLE_SIZE = 2;
  static const size_t DATA_SIZE = 4;
  static const size_t ENDING_SIZE = 2;

  static constexpr uint8_t PREAMBLE[PREAMBLE_SIZE] = { 0x0A, 0x0D };
  static constexpr uint8_t ENDING[ENDING_SIZE] = { 0xFA, 0xFA };

  static constexpr float MINIMUM_RANGE_VALUE = 0;
  static constexpr float MAXIMUM_RANGE_VALUE = 2.5;
  static constexpr uint8_t NO_SENSOR_CONNECTED = 0xF0;

  std::shared_ptr<serial::Serial> serial_port_;
  RangefinderPacket current_packet_;

  ros::Publisher range_publisher_;

  void sendCurrentPacket();
  void sendRangeMessage(size_t rangefinder_index, float data);
};

#endif // RR_RANGEFINDER_NODE_H
