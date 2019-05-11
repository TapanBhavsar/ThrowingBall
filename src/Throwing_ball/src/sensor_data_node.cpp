#include "sensor_data_node.h"
#include "kalman_filter_matrices.h"

sensor_data_node::sensor_data_node(ros::NodeHandle nh)
    :rate_(kFrequency_),
    nh_(nh)
{

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"sensor_data_node");
    
    ros::NodeHandle nh;
    
    KalmanFilterMatrices kfm(nh);
    kfm.SetAllMatrices();
    
    return 0;
}
