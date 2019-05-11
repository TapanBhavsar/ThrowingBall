#include "sensor_data.h"

SensorData::SensorData(ros::NodeHandle nh)
  : kalman_matrices(nh),
    nh_(nh)
{
    kalman_matrices.SetAllMatrices();
    InitializeState();
    
}

void SensorData::CalculateGroundTruth()
{
    Eigen::MatrixXd A_system_matrix = kalman_matrices.GetSystemMatrix();
    Eigen::MatrixXd B_input_gain_matrix = kalman_matrices.GetMeasurementMatrix();
    Eigen::MatrixXd c_measurement_matrix = kalman_matrices.GetInputGainMatrix();

    predict_state_ = A_system_matrix * prior_predict_state_ + B_input_gain_matrix * input_;
    ground_truth_state_ = c_measurement_matrix * predict_state_;
    prior_predict_state_ = predict_state_; 
}

void SensorData::AddNoise(){

}

bool SensorData::isGroundTouched()
{
    
}

void SensorData::InitializeState()
{
    SetPriorPredictState();
    SetInputState();
}

void SensorData::SetPriorPredictState()
{
    double prior_state_height;
    double prior_state_width;

    std::vector<double> buffer;

    nh_.getParam("prior_state_height",prior_state_height);
    nh_.getParam("prior_state_width",prior_state_width);
    nh_.getParam("prior_state",buffer);
 
    prior_predict_state_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                       prior_state_height,
                                                       prior_state_width);
}

void SensorData::SetInputState()
{
    double input_state_height;
    double input_state_width;

    std::vector<double> buffer;

    nh_.getParam("input_state_height",input_state_height);
    nh_.getParam("input_state_width",input_state_width);
    nh_.getParam("input_state",buffer);
 
    input_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                         input_state_height,
                                         input_state_width);
}