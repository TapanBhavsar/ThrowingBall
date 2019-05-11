#ifndef _KALMAN_FILTER_MATRICES_H_
#define _KALMAN_FILTER_MATRICES_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

class KalmanFilterMatrices
{
  public:
    KalmanFilterMatrices(ros::NodeHandle nh);
    ~KalmanFilterMatrices();
    void SetAllMatrices();

  private:
    void SetSystemMatrix();
    void SetInputGainMatrix();
    void SetMeasurementMatrix();
    
    ros::NodeHandle nh_;

    Eigen::MatrixXd A_system_matrix_;
    Eigen::MatrixXd B_input_gain_matrix_;
    Eigen::MatrixXd c_measurement_matrix_;
        
};

# endif // _KALMAN_FILTER_MATRICES_H_