#ifndef _SENSOR_DATA_NODE_H_
#define _SENSOR_DATA_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <eigen_conversions/eigen_msg.h>

class sensor_data_node
{
  public:
    sensor_data_node(ros::NodeHandle nh);
    ~sensor_data_node();
  private:
    double kFrequency_ {2};

    double message_queue_;

    ros::NodeHandle nh_;
    ros::Rate rate_;
    /* data */

};

#endif // _SENSOR_DATA_NODE_H_
