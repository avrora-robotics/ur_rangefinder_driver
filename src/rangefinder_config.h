#ifndef UR_RANGEFINDER_CONFIG_H
#define UR_RANGEFINDER_CONFIG_H

#include "ros/ros.h"

class RangefinderConfig
{

public:
  void init()
  {
    ros::NodeHandle parameter_node_handle("~");
    parameter_node_handle.param<int>("baud_rate", baud_rate, 9600);
    parameter_node_handle.param<std::string>("port", port, "/dev/ttyACM1");
  }

  int baud_rate = 9600;
  std::string port = "/dev/ttyACM1";

};

extern RangefinderConfig rangefinder_config;


#endif
