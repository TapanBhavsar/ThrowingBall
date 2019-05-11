#include "sensor_data.h"

SensorData::SensorData(ros::NodeHandle nh)
  : kalman_matrices(nh)
{
    kalman_matrices.SetAllMatrices();
}

void SensorData::CalculateGroundTruth()
{

}

void SensorData::AddNoise(){

}

bool SensorData::isGroundTouched()
{
    
}
