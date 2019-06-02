#ifndef _SENSOR_DATA_NODE_H_
#define _SENSOR_DATA_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <eigen_conversions/eigen_msg.h>

#include "kalman_filter_matrices.h"
#include "sensor_data.h"

class SensorDataNode
{
  public:
    SensorDataNode(ros::NodeHandle nh);
    ~SensorDataNode();
    void PublishMesurementData();

  private:
    double kFrequency_ {100};
    double kMessageQueue_ {100};
    std::string measurement_topic_name_ {"measured_data"};

    SensorData sd_;
    
    ros::NodeHandle nh_;
    ros::Rate rate_;
    ros::Publisher measurement_publisher_;
};

#endif // _SENSOR_DATA_NODE_H_
