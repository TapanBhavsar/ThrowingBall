#include <eigen3/Eigen/Dense>

#include "sensor_data_node.h"

SensorDataNode::SensorDataNode(ros::NodeHandle nh)
    :rate_(kFrequency_),
    nh_(nh),
    sd_(nh)
{
    measurement_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(measurement_topic_name_,kMessageQueue_);
}

SensorDataNode::~SensorDataNode()
{

}

void SensorDataNode::PublishMesurementData()
{
    while (ros::ok())
    {
        Eigen::MatrixXd measurments = sd_.GetMesurements();
        std_msgs::Float64MultiArray measurement_msg;
        tf::matrixEigenToMsg(measurments,measurement_msg);
        
        if(sd_.isGroundTouched())
        {
            ros::shutdown();
        } 
        measurement_publisher_.publish(measurement_msg);
        
        ros::spinOnce();
        rate_.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sensor_data_node");
    ros::NodeHandle nh;

    SensorDataNode sdn(nh);
    sdn.PublishMesurementData();

    return 0;
}
