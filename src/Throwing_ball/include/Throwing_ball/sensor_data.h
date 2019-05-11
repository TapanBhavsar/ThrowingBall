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
    void InitializeState();

    void SetPriorPredictState();
    void SetInputState();

    KalmanFilterMatrices kalman_matrices;

    Eigen::MatrixXd prior_predict_state_;
    Eigen::MatrixXd predict_state_;
    Eigen::MatrixXd input_;
    Eigen::MatrixXd ground_truth_state_;

    ros::NodeHandle nh_;

};

#endif // _SENSOR_DATA_H_