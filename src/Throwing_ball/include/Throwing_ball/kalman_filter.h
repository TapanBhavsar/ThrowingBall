#ifndef _KALMAN_FILTER_H_
#define _KALMAN_FILTER_H_

#include "kalman_filter_matrices.h"

class KalmanFilter
{
  public:
    KalmanFilter(ros::NodeHandle nh);
    ~KalmanFilter();
    void CalculateEstimateState();
    void SetSubscribeData(const Eigen::MatrixXd&);
    Eigen::MatrixXd GetEstimatState() const;
  
  private:
    void InitializeState();
    void SetPriorPredictState();
    void SetInputState();
    void SetQprocessNoise();
    void SetPriorCovariance();
    void SetRmeasurementNoise();

    void Predict();
    void Correct();

    Eigen::MatrixXd prior_predict_state_;
    Eigen::MatrixXd predict_state_;
    Eigen::MatrixXd input_;
    Eigen::MatrixXd ground_truth_state_;
    Eigen::MatrixXd measurements_;
  
    Eigen::MatrixXd prior_covariance_;
    Eigen::MatrixXd predict_covariance_;
    Eigen::MatrixXd kalman_gain_;
    Eigen::MatrixXd estimated_state_;
    Eigen::MatrixXd subscribe_data_;


    Eigen::MatrixXd Q_process_noise_;
    Eigen::MatrixXd R_measurement_noise_;
    
    KalmanFilterMatrices kalman_matrices;
    ros::NodeHandle nh_;

};

#endif // _KALMAN_FILTER_H_