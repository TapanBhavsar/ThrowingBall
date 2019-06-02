#include <iostream>
#include "kalman_filter_node.h"


KalmanFilterNode::KalmanFilterNode(ros::NodeHandle nh)
  :nh_(nh),
   kalmanfilter_(nh)
{
    measurement_subscriber_ = nh_.subscribe(measurement_topic_name_, kMessageQueue_, &KalmanFilterNode::MeasurementCallback,this);
}

KalmanFilterNode::~KalmanFilterNode()
{

}

void KalmanFilterNode::SubscribeNoiseData()
{
    ros::spin();
}

Eigen::MatrixXd KalmanFilterNode::TransformRosMultiArrayToEigen(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    float matrix_height = msg->layout.dim[0].size;
	float matrix_width = msg->layout.dim[1].size;

    std::vector<double> data = msg->data;
    Eigen::MatrixXd matrix_data = Eigen::Map<Eigen::MatrixXd>(data.data(), matrix_height, matrix_width);  
    return matrix_data;
}

void KalmanFilterNode::MeasurementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    Eigen::MatrixXd sensor_data;
    Eigen::MatrixXd filtered_data;
    sensor_data = TransformRosMultiArrayToEigen(msg);

    kalmanfilter_.SetSubscribeData(sensor_data);
    kalmanfilter_.CalculateEstimateState();
    filtered_data = kalmanfilter_.GetEstimatState();

    std::cout << "filtered data: " << filtered_data << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"kalman_filter_node");
    ros::NodeHandle nh;

    KalmanFilterNode kfd(nh);
    kfd.SubscribeNoiseData();

    return 0;
}