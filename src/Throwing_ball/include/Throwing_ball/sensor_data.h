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
    ~SensorData();
    bool isGroundTouched();
    Eigen::MatrixXd GetMesurements();
  
  private:
    void CalculateGroundTruth();
    void AddNoise();
    void InitializeState();

    void SetPriorPredictState();
    void SetInputState();

    const double k_mean_value_ {0};
    const double k_standard_deviation_ {0.1}; 

    KalmanFilterMatrices kalman_matrices;

    Eigen::MatrixXd prior_predict_state_;
    Eigen::MatrixXd predict_state_;
    Eigen::MatrixXd input_;
    Eigen::MatrixXd ground_truth_state_;
    Eigen::MatrixXd measurements_;
  
    ros::NodeHandle nh_;
};

#endif // _SENSOR_DATA_H_