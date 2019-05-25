#include "kalman_filter_matrices.h"

KalmanFilterMatrices::KalmanFilterMatrices(ros::NodeHandle nh)
    :nh_(nh)
{

}

KalmanFilterMatrices::~KalmanFilterMatrices()
{

}

void KalmanFilterMatrices::SetAllMatrices()
{
    SetSystemMatrix();
    SetInputGainMatrix();
    SetMeasurementMatrix();
}

Eigen::MatrixXd KalmanFilterMatrices::GetSystemMatrix() const
{
    return A_system_matrix_.transpose();
}

Eigen::MatrixXd KalmanFilterMatrices::GetInputGainMatrix() const
{
    return B_input_gain_matrix_;
}

Eigen::MatrixXd KalmanFilterMatrices::GetMeasurementMatrix() const
{
    return c_measurement_matrix_;
}

void KalmanFilterMatrices::SetSystemMatrix()
{
    double A_system_matrix_height;
    double A_system_matrix_width;

    std::vector<double> buffer;


    nh_.getParam("A_system_matrix_height",A_system_matrix_height);
    nh_.getParam("A_system_matrix_width",A_system_matrix_width);
    nh_.getParam("A_system_matrix",buffer);
    
    A_system_matrix_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                   A_system_matrix_height,
                                                   A_system_matrix_width);

}

void KalmanFilterMatrices::SetInputGainMatrix()
{
    double B_input_gain_matrix_height;
    double B_input_gain_matrix_width;

    std::vector<double> buffer;

    nh_.getParam("B_input_gain_height",B_input_gain_matrix_height);
    nh_.getParam("B_input_gain_width",B_input_gain_matrix_width);
    nh_.getParam("B_input_gain_matrix",buffer);

    B_input_gain_matrix_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                   B_input_gain_matrix_height,
                                                   B_input_gain_matrix_width);
}

void KalmanFilterMatrices::SetMeasurementMatrix()
{
    double c_measurement_matrix_height;
    double c_measurement_matrix_width;

    std::vector<double> buffer;

    nh_.getParam("c_measurement_matrix_height",c_measurement_matrix_height);
    nh_.getParam("c_measurement_matrix_width",c_measurement_matrix_width);
    nh_.getParam("c_measurement_matrix",buffer);

    c_measurement_matrix_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                   c_measurement_matrix_height,
                                                   c_measurement_matrix_width);
}


