#include "kalman_filter.h"

KalmanFilter::KalmanFilter(ros::NodeHandle nh)
  :kalman_matrices(nh),
   nh_(nh)
{
    kalman_matrices.SetAllMatrices();
    InitializeState();
}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::CalculateEstimateState()
{
    Predict();
    Correct();
}

void KalmanFilter::SetSubscribeData(const Eigen::MatrixXd& subscribe_data)
{
    subscribe_data_ = subscribe_data;
}

Eigen::MatrixXd KalmanFilter::GetEstimatState() const
{
    return estimated_state_;
}

void KalmanFilter::InitializeState()
{
    SetPriorPredictState();
    SetInputState();
    SetQprocessNoise();
    SetPriorCovariance();
    SetRmeasurementNoise();
}

void KalmanFilter::SetPriorPredictState()
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

void KalmanFilter::SetInputState()
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

void KalmanFilter::SetQprocessNoise()
{
    double Q_process_noise_height;
    double Q_process_noise_width;

    std::vector<double> buffer;

    nh_.getParam("Q_process_noise_height",Q_process_noise_height);
    nh_.getParam("Q_process_noise_width",Q_process_noise_width);
    nh_.getParam("Q_process_noise",buffer);
 
    Q_process_noise_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                       Q_process_noise_height,
                                                       Q_process_noise_width);
}

void KalmanFilter::SetPriorCovariance()
{
    double prior_covariance_height;
    double prior_covariance_width;

    std::vector<double> buffer;

    nh_.getParam("prior_covariance_height",prior_covariance_height);
    nh_.getParam("prior_covariance_width",prior_covariance_width);
    nh_.getParam("prior_covariance",buffer);
 
    prior_covariance_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                       prior_covariance_height,
                                                       prior_covariance_width);
}

void KalmanFilter::SetRmeasurementNoise()
{
    double R_measurement_noise_height;
    double R_measurement_noise_width;

    std::vector<double> buffer;

    nh_.getParam("R_measurement_noise_height",R_measurement_noise_height);
    nh_.getParam("R_measurement_noise_width",R_measurement_noise_width);
    nh_.getParam("R_measurement_noise",buffer);
 
    R_measurement_noise_ = Eigen::Map<Eigen::MatrixXd>(buffer.data(),
                                                       R_measurement_noise_height,
                                                       R_measurement_noise_width);
}

void KalmanFilter::Predict()
{
    Eigen::MatrixXd A_system_matrix = kalman_matrices.GetSystemMatrix();
    Eigen::MatrixXd B_input_gain_matrix = kalman_matrices.GetInputGainMatrix();

    predict_state_ = A_system_matrix * prior_predict_state_ + B_input_gain_matrix * input_;
    predict_covariance_ = A_system_matrix * prior_covariance_ * (A_system_matrix.transpose()) + Q_process_noise_;
}

void KalmanFilter::Correct()
{   
    double identity_matrix_size {4}; // This parameter needs to be changed as per input matrices

    Eigen::MatrixXd c_measurement_matrix = kalman_matrices.GetMeasurementMatrix();
    kalman_gain_ = predict_covariance_ * (c_measurement_matrix.transpose()) *
                   (((c_measurement_matrix * predict_covariance_ * (c_measurement_matrix.transpose())) +
                   R_measurement_noise_).inverse());

    estimated_state_ = predict_state_ + kalman_gain_ * (subscribe_data_ - (c_measurement_matrix * predict_state_));
    prior_covariance_ = (Eigen::MatrixXd::Identity(identity_matrix_size,identity_matrix_size) - (kalman_gain_ * c_measurement_matrix)) *
                         predict_covariance_;

    prior_predict_state_ = estimated_state_;  
}