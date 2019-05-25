#include "sensor_data.h"
#include <iostream>

SensorData::SensorData(ros::NodeHandle nh)
  : kalman_matrices(nh),
    nh_(nh)
{
    kalman_matrices.SetAllMatrices();
    InitializeState();  
}

bool SensorData::isGroundTouched()
{
    if(predict_state_(0,0) > 0 && predict_state_(1,0) < 0)
    { 
        return true;
    }
    else
    {
        return false;
    }
}

Eigen::MatrixXd SensorData::GetMesurements()
{
    CalculateGroundTruth();
    AddNoise();
    return measurements_;
}

void SensorData::CalculateGroundTruth()
{
    Eigen::MatrixXd A_system_matrix = kalman_matrices.GetSystemMatrix();
    Eigen::MatrixXd B_input_gain_matrix = kalman_matrices.GetInputGainMatrix();
    Eigen::MatrixXd c_measurement_matrix = kalman_matrices.GetMeasurementMatrix();
    
    predict_state_ = A_system_matrix * prior_predict_state_ + B_input_gain_matrix * input_;
    std::cout << "measurement matrix: " <<  c_measurement_matrix << std::endl;
    std::cout << "prediction: " <<  predict_state_ << std::endl;
    ground_truth_state_ = c_measurement_matrix * predict_state_;
    std::cout << "ground truth: " <<  ground_truth_state_ << std::endl;
    prior_predict_state_ = predict_state_; 
}

void SensorData::AddNoise()
{
    std::random_device random_generator;
    std::mt19937 random_seed_x(random_generator());
    std::mt19937 random_seed_y(random_generator());
    std::normal_distribution<double> distribution(k_mean_value_,k_standard_deviation_);
    double random_sample_x = distribution(random_seed_x);
    double random_sample_y = distribution(random_seed_y);
    measurements_ = ground_truth_state_ + (Eigen::Matrix<double, 2, 1> () 
                                            << random_sample_x, random_sample_y).finished();
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