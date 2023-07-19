#include "OutpostFilter.h"

//
// Created by MicDZ on 2023/7/16.
//

ly::OutpostFilter::OutpostFilter(double measureNoise, double processNoise) {
    is_kalman_init = false;
    ekf.measurement_noise_cov << measureNoise, 0,
            0, measureNoise;
	this->processNoise=processNoise;
    angleVelocity = 1;
}

void ly::OutpostFilter::setTransitionMatrix() {
	ekf.transition_matrix << 1, sin(angleVelocity*updatedTime)/angleVelocity, 0, -(1-cos(angleVelocity*updatedTime))/angleVelocity,
                             0, cos(angleVelocity*updatedTime), 0, -sin(angleVelocity*updatedTime),
                             0, (1-cos(angleVelocity*updatedTime))/angleVelocity, 1, sin(angleVelocity*updatedTime)/angleVelocity,
                             0, sin(angleVelocity*updatedTime), 0, cos(angleVelocity*updatedTime);
}

void ly::OutpostFilter::setUpdatedTime(double delta_t) {
    if (fabs(delta_t) < 1e-4) // 防止时间差为0
    {
        updatedTime = 8.0 / 1000.0;
    } else {
        updatedTime = delta_t / 1000.0;
    }
}

void ly::OutpostFilter::setMeasurementNoise() {
    // TODO: 根据装甲板的距离设置噪声
    // TODO: 根据实际设置噪声
    double
            measurementNoiseX = 0.01,
            measurementNoiseY = 0.01;

    ekf.measurement_noise_cov << measurementNoiseX, 0,
            0, measurementNoiseY;
}

void ly::OutpostFilter::setProcessNoise() {
	ekf.process_noise_cov<<pow(updatedTime,4)/4,pow(updatedTime,3)/2,0,0,
                            pow(updatedTime,3)/2,pow(updatedTime,2),0,0,
                            0,0,pow(updatedTime,4)/4,pow(updatedTime,3)/2,
                            0,0,pow(updatedTime,3)/2,pow(updatedTime,2);
    //double k=5e13;
    ekf.process_noise_cov *= processNoise;
    std::cout<<"process_noise_cov:"<<ekf.process_noise_cov<<std::endl;

    // TODO: 这里过程噪声的设置还有一个变换
    // ekf.process_noise_cov = process_noise_matrix * process_noise * process_noise_matrix.transpose();
}

void ly::OutpostFilter::rebootEkf(Eigen::Matrix<double, 2, 1> measurement) {
    ekf.posteriori_state_estimate << measurement(0), 0, measurement(1), 0;
    ekf.error_cov_post = Eigen::Matrix<double, 4, 4>::Identity();
    ekf.measurement_matrix <<
    		1, 0, 0, 0,
            0, 0, 1, 0;
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::correct(const Eigen::Matrix<double, 2, 1>& measurement) {
    // setUpdatedTime(measurement(2));
    puts("in");
    setProcessNoise();     // 更新过程噪声
    setMeasurementNoise(); // 更新测量噪声
    ekf.predict(measurement);
    ekf.update();
    Eigen::Vector2d result;
    result << ekf.posteriori_state_estimate(0), ekf.posteriori_state_estimate(2);
    return result;
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::runKalman(Eigen::Matrix<double, 2, 1> measurement, double delta_t) {
    // 第一次初始化

    if (!is_kalman_init) {
        // set signal values
        is_kalman_init = true;
        // reset kalman
        rebootEkf(measurement);
        return measurement;
    } else {
        // set update time
        setUpdatedTime(delta_t);

        // update transition matrix
        setTransitionMatrix();
		puts("in");
        // 返回滤波后的位置
        return correct(measurement);
    }
}
