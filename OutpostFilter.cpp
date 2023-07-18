#include "OutpostFilter.h"


//
// Created by MicDZ on 2023/7/16.
//

ly::OutpostFilter::OutpostFilter() {
    is_kalman_init = false;
}

void ly::OutpostFilter::setTransitionMatrix() {

    double
    x = ekf.posteriori_state_estimate(0),
    y = ekf.posteriori_state_estimate(1),
    theta = ekf.posteriori_state_estimate(2),
    velocity = ekf.posteriori_state_estimate(3),
    acceleration = ekf.posteriori_state_estimate(4);

    ekf.transition_matrix<<
        1, 0, -1./2*sin(theta)*updatedTime, cos(theta)*updatedTime,0,
        0, 1, -1./2*cos(theta)*updatedTime, -sin(theta)*updatedTime,0,
        0, 0, 1, -acceleration/(velocity*velocity)*updatedTime, updatedTime/velocity,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

}

void ly::OutpostFilter::setUpdatedTime(double delta_t) {
    if (fabs(delta_t) < 1e-4) // 防止时间差为0
    {
        updatedTime = 8.0 / 1000.0;
    }
    else
    {
        updatedTime = delta_t / 1000.0;
    }
}

void ly::OutpostFilter::setMeasurementNoise() {
    //TODO: 根据装甲板的距离设置噪声

    double
    measurementNoiseX = 0.1,
    measurementNoiseY = 0.1;

    ekf.measurement_noise_cov<<
        measurementNoiseX, 0,
        0, measurementNoiseY;
}



void ly::OutpostFilter::setProcessNoise() {
    double
    processNoiseX = 0.1,
    processNoiseY = 0.1,
    processNoiseTheta = 0,
    processNoiseVelocity = 0.1,
    processNoiseAcceleration = 0.1;
    Eigen::Matrix<double, 5, 5> process_noise;
    process_noise<<
        processNoiseX, 0, 0, 0, 0,
        0, processNoiseY, 0, 0, 0,
        0, 0, processNoiseTheta, 0, 0,
        0, 0, 0, processNoiseVelocity, 0,
        0, 0, 0, 0, processNoiseAcceleration;

    Eigen::Matrix<double, 5, 2> process_noise_matrix;
    process_noise_matrix<<
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        updatedTime, 0, 1, 0, 0,
        0, updatedTime, 0, 1, 0,
        0, 0, 0, 0, 1;


    //FIXME: 这里的设置有点问题
    ekf.process_noise_cov = process_noise_matrix * process_noise * process_noise_matrix.transpose();
}
void ly::OutpostFilter::rebootEkf(Eigen::Matrix<double, 2, 1> measurement) {
    ekf.posteriori_state_estimate<<
                                 measurement(0), measurement(1), 0, 0, 0;
    ekf.error_cov_post = Eigen::Matrix<double, 5, 5>::Identity();
}



Eigen::Matrix<double, 2, 1> ly::OutpostFilter::correct(Eigen::Matrix<double, 2, 1> measurement) {
    //setUpdatedTime(measurement(2));
    puts("in");
    setProcessNoise(); // 更新过程噪声
    setMeasurementNoise(); // 更新测量噪声
    std::cout<<measurement<<std::endl;
    ekf.predict(measureToState, measurement);
    ekf.update();

    Eigen::Vector2d result;
    result<<ekf.posteriori_state_estimate(0), ekf.posteriori_state_estimate(1);
    return result;
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::runKalman(Eigen::Matrix<double, 2, 1> measurement, double delta_t) {
    // 第一次初始化
    if (!is_kalman_init)
    {
        // set signal values
        is_kalman_init = true;

        // reset kalman
        rebootEkf(measurement);
        return measurement;
    }
    else
    {
        // set update time
        setUpdatedTime(delta_t);

        // update transition matrix
        setTransitionMatrix();

        // 返回滤波后的位置
        return correct(measurement);
    }
}


