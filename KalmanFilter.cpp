/*
 * @Author: Gao Zhiyi gaozhiyi127@gmail.com
 * @Date: 2023-10-14 20:26:47
 * @LastEditors: Gao Zhiyi gaozhiyi127@gmail.com
 * @LastEditTime: 2023-10-14 21:35:10
 * @FilePath: \KalmanFilter\KalmanFilter.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "KalmanFilter.h"

int main(){
    
    MatrixXd A(2, 2);
    A << 1, 1, 0, 1;
    MatrixXd B(2, 1);
    B << 0.5, 1;
    MatrixXd C(1, 2);
    C << 1, 0;
    MatrixXd Q(2, 2);
    Q << 0.1, 0, 0, 0.1;
    MatrixXd R(1, 1);
    R << 1;

    KalmanFilter kf;
    kf.State = VectorXd(2);
    kf.State << 0, 0;
    kf.Covariance = MatrixXd::Identity(2, 2);
    kf.Transition = A;
    kf.Control = B;
    kf.observation = C;
    kf.ProcessNoise = Q;
    kf.MeasurementNoise = R;

    VectorXd measurement(10);
    measurement << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

    for(int i = 0; i < measurement.size(); i++){
        kf.predict(VectorXd::Zero(1));
        kf.update(VectorXd::Constant(1, measurement(i)));
        std::cout << "Estimated state at time step" << i + 1<<":"
        << kf.State.transpose() << std::endl;
    }

    return 0;
}