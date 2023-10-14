#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

struct KalmanFilter{
    VectorXd State; // 状态向量
    MatrixXd Covariance; // 估计误差协方差矩阵
    MatrixXd Transition; // 状态转移矩阵
    MatrixXd Control; // 控制矩阵
    MatrixXd observation; // 测量矩阵
    MatrixXd ProcessNoise; // 过程噪声协方差矩阵
    MatrixXd MeasurementNoise; // 测量噪声协方差矩阵

    void predict(const VectorXd& controlInput){
        State = Transition * State + Control * controlInput;
        Covariance = Transition * Covariance * Transition.transpose() + ProcessNoise;
    }

    void update(const VectorXd& measurement){
        MatrixXd kalmanGain = Covariance * observation.transpose() * (observation * Covariance * observation.transpose() + MeasurementNoise).inverse();
        State = State + kalmanGain * (measurement - observation * State);
        Covariance = (MatrixXd::Identity(State.size(), State.size()) - kalmanGain * observation) * Covariance;
    }
};

#endif // _KALMANFILTER_H_

