#include "OutpostFilter.h"

//
// Created by MicDZ on 2023/7/16.
//

// ekf3: est:x,y,center_x,center_y

ly::OutpostFilter::OutpostFilter(double radius, double angleVelocity, double measureNoise, double processNoise) {
    is_kalman_init = false;
    ekf.measurement_noise_cov << measureNoise, 0,
            0, measureNoise;
	this->processNoise=processNoise;
    this->radius = radius;
    this->angleVelocity = angleVelocity;
}

void ly::OutpostFilter::setTransitionMatrix() {
	ekf.transition_matrix << 1, sin(angleVelocity*updatedTime)/angleVelocity, 0, -(1-cos(angleVelocity*updatedTime))/angleVelocity,
                             0, cos(angleVelocity*updatedTime), 0, -sin(angleVelocity*updatedTime),
                             0, (1-cos(angleVelocity*updatedTime))/angleVelocity, 1, sin(angleVelocity*updatedTime)/angleVelocity,
                             0, sin(angleVelocity*updatedTime), 0, cos(angleVelocity*updatedTime);
    ekf2.transition_matrix << 1,0,
                                0,1;
//    ekf3.transition_matrix << cos(angleVelocity*updatedTime), -sin(angleVelocity*updatedTime), 0, 0,
//                              sin(angleVelocity*updatedTime), cos(angleVelocity*updatedTime), 0, 0,
//                              0, 0, 1, 0,
//                              0, 0, 0, 1;
}

void ly::OutpostFilter::setUpdatedTime(double delta_t) {

        updatedTime = delta_t;

}

void ly::OutpostFilter::setMeasurementNoise() {
    // TODO: 根据装甲板的距离设置噪声
    // TODO: 根据实际设置噪声
    double
            measurementNoiseX = 0.01,
            measurementNoiseY = 0.01;

    ekf.measurement_noise_cov << measurementNoiseX, 0,
            0, measurementNoiseY;
    ekf2.measurement_noise_cov << 0.5, 0,
                                    0, 0.5;
//    ekf3.measurement_noise_cov << measurementNoiseX, 0,
//            0, measurementNoiseY;
}

void ly::OutpostFilter::setProcessNoise() {
	ekf.process_noise_cov<<pow(updatedTime,4)/4,pow(updatedTime,3)/2,0,0,
                            pow(updatedTime,3)/2,pow(updatedTime,2),0,0,
                            0,0,pow(updatedTime,4)/4,pow(updatedTime,3)/2,
                            0,0,pow(updatedTime,3)/2,pow(updatedTime,2);
    //double k=5e13;
    ekf.process_noise_cov *= processNoise;
//    std::cout<<"process_noise_cov:"<<ekf.process_noise_cov<<std::endl;
// TODO: 认为中心是静止的，直接写死
    ekf2.process_noise_cov << 0.001, 0,
            0, 0.001;
    // TODO: 这里的过程噪声还需要计算一下
//    ekf3.process_noise_cov<< 0.01, 0.01, 0, 0,
//                            0.01, 0.01, 0, 0,
//                            0, 0, 0.01, 0,
//                            0, 0, 0, 0.01;

    // TODO: 这里过程噪声的设置还有一个变换
    // ekf.process_noise_cov = process_noise_matrix * process_noise * process_noise_matrix.transpose();
}

void ly::OutpostFilter::rebootEkf1(Eigen::Matrix<double, 2, 1> measurement) {
    ekf.posteriori_state_estimate << measurement(0), 0, measurement(1), 0;
    ekf.error_cov_post = Eigen::Matrix<double, 4, 4>::Identity();
    ekf.measurement_matrix <<
    		1, 0, 0, 0,
            0, 0, 1, 0;


//    ekf3.measurement_matrix << Eigen::Matrix<double, 4, 4>::Identity();
//    ekf3.measurement_noise_cov<< Eigen::Matrix<double, 4, 4>::Identity()*0.01;
//    ekf3.error_cov_post << Eigen:: Matrix<double, 4, 4>::Identity();
}
void ly::OutpostFilter::rebootEkf2(Eigen::Matrix<double, 2, 1> centerMeasurement) {
    ekf2.measurement_matrix <<
                            1, 0,
            0, 1;

    ekf2.error_cov_post << Eigen::Matrix<double, 2, 2>::Identity();
    ekf2.posteriori_state_estimate << centerMeasurement(0), centerMeasurement(1);

}

void ly::OutpostFilter::rebootEkf3(Eigen::Matrix<double, 4, 1> measurement) {
    ekf.posteriori_state_estimate << measurement;

//    ekf3.measurement_matrix << Eigen::Matrix<double, 4, 4>::Identity();
//    ekf3.measurement_noise_cov<< Eigen::Matrix<double, 4, 4>::Identity()*0.01;
//    ekf3.error_cov_post << Eigen:: Matrix<double, 4, 4>::Identity();
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::correct1(const Eigen::Matrix<double, 2, 1>& measurementArmor) {
    // setUpdatedTime(measurement(2));
    setProcessNoise();     // 更新过程噪声
    setMeasurementNoise(); // 更新测量噪声
    ekf.predict(measurementArmor);
    ekf.update();
    Eigen::Vector2d result;
    result << ekf.posteriori_state_estimate(0), ekf.posteriori_state_estimate(2);
    return result;
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::correct2(const Eigen::Matrix<double, 2, 1>& measurementCenter) {
    // setUpdatedTime(measurement(2));
    setProcessNoise();     // 更新过程噪声
    setMeasurementNoise(); // 更新测量噪声

    ekf2.predict(measurementCenter);
    ekf2.update();
    Eigen::Vector2d result;
    result << ekf2.posteriori_state_estimate(0), ekf2.posteriori_state_estimate(1);
    return result;
}


Eigen::Matrix<double, 4, 1> ly::OutpostFilter::runKalman(const Eigen::Matrix<double, 2, 1>& measurement, double delta_t) {
    // 第一次初始化
    if (!is_kalman_init) {
        // set signal values
        is_kalman_init = true;
        // reset kalman
        rebootEkf1(measurement);
        // 结合centerPosition输出
        Eigen::Matrix<double, 4, 1> result;
        result << measurement, measurement;
        return result;
    } else {
        // set update time
        setUpdatedTime(delta_t);
        // update transition matrix
        setTransitionMatrix();
        setMeasurementNoise();
        setProcessNoise();
        Eigen::Matrix<double, 4, 1> result;

        if(!get_center_init) {
            Eigen::Matrix<double, 2, 1> armorEst;
            armorEst<< correct1(measurement);
            result<<armorEst, armorEst;
            Eigen::Vector2d centerPosition = getCenterPosition();
            rebootEkf2(getCenterPosition());
            get_center_init=true;
        }
        else {
        // 返回滤波后的位置
            std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<getCenterPosition()<<std::endl;
            result << correct1(measurement), correct2(getCenterPosition());

        }
        return result;
    }
}

Eigen::Matrix<double, 2, 1> ly::OutpostFilter::getCenterPosition() {
    Eigen::Matrix<double, 2, 1> speed, pos;
    speed << ekf.posteriori_state_estimate(1), ekf.posteriori_state_estimate(3);
    pos << ekf.posteriori_state_estimate(0), ekf.posteriori_state_estimate(2);
    // 获取与速度垂直的方向
    Eigen::Matrix<double, 2, 1> verticalDirection;
    verticalDirection << -speed(1), speed(0);
    // 从当前点向沿速度的垂直方向移动半径的距离，得到中心
    Eigen::Vector2d centerPosition = pos + verticalDirection.normalized() * radius;
    return centerPosition;
}

Eigen::Vector2d ly::OutpostFilter::getEstimate(double delta_t) {
    setUpdatedTime(delta_t);
    setTransitionMatrix();

    Eigen::Vector4d estimate = ekf.transition_matrix * ekf.posteriori_state_estimate + ekf.control_matrix * ekf.control_vector;
    return Eigen::Vector2d(estimate(0), estimate(2));
}

