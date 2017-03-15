#ifndef UR_RANGEFINDER_NODE_H
#define UR_RANGEFINDER_NODE_H

#include "ros/ros.h"
#include "serial/serial.h"
#include <sensor_msgs/Range.h>

class RangefinderNode
{

public:
  RangefinderNode(ros::NodeHandle& node_handle, std::shared_ptr<serial::Serial> serial_port);
  void update();

private:
  static constexpr uint8_t PREAMBLE_SIZE = 2;
  static constexpr uint8_t DATA_SIZE = 4;
  static constexpr uint8_t ENDING_SIZE = 2;
  static constexpr uint8_t PACKET_SIZE = PREAMBLE_SIZE + DATA_SIZE + ENDING_SIZE + 1;

  static constexpr uint8_t PREAMBLE[PREAMBLE_SIZE] = { 0x0A, 0x0D };
  static constexpr uint8_t ENDING[ENDING_SIZE] = { 0xFA, 0xFA };

  static constexpr float MINIMUM_RANGE_VALUE = 0;
  static constexpr float MAXIMUM_RANGE_VALUE = 2.5;
  static constexpr uint8_t NO_SENSOR_CONNECTED = 0xF0;

  std::shared_ptr<serial::Serial> serial_port_;
  ros::Publisher range_publisher_;

  struct RangefinderPacket
  {
    float ranges[DATA_SIZE] = {0};
    uint8_t offset = 0;
  } current_packet_;

  void sendCurrentPacket();
  void sendRangeMessage(uint8_t rangefinder_index, float data);
};

#endif // RR_RANGEFINDER_NODE_H
