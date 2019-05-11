#ifndef _SENSOR_DATA_H_
#define _SENSOR_DATA_H_

#include <eigen3/Eigen/Dense>
#include <random>
#include <cmath>
#include "kalman_filter_matrices.h"

class SensorData
{
  public:
    SensorData(ros::NodeHandle nh);
  private:
    void CalculateGroundTruth();
    void AddNoise();
    bool isGroundTouched();

    KalmanFilterMatrices kalman_matrices;
};

#endif // _SENSOR_DATA_H_