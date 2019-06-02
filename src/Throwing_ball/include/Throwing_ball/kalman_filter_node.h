#ifndef _KALMAN_FILTER_NODE_H_
#define _KALMAN_FILTER_NODE_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include "kalman_filter.h"

class KalmanFilterNode
{
  public:
    KalmanFilterNode(ros::NodeHandle nh);
    ~KalmanFilterNode();

    void SubscribeNoiseData();
    
  private:
    Eigen::MatrixXd TransformRosMultiArrayToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void MeasurementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    double kMessageQueue_ {100};
    std::string measurement_topic_name_ {"measured_data"};
    
    KalmanFilter kalmanfilter_;
    ros::NodeHandle nh_;
    ros::Subscriber measurement_subscriber_;

};

#endif //_KALMAN_FILTER_NODE_H_