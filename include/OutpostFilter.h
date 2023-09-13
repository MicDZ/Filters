//
// Created by MicDZ on 2023/7/16.
//

#ifndef OUTPOSTFILTER_OUTPOSTFILTER_H
#define OUTPOSTFILTER_OUTPOSTFILTER_H
#include <ExtendedKalman.hpp>

namespace ly {
    class MeasureToState {
    public:
        template<class T>
        // x,x_v, y,y_v, z,z_v
        void operator()(const T measure[2], T state[5]) // ms
        {
            state[0] = measure[0]; // x
            state[1] = measure[1];                                      // y
        }
    };

    class OutpostFilter {
    private:
        ExtendedKalman<double, 4, 2> ekf; // est: x,x_v,y,y_v, measure: x,y
        ExtendedKalman<double, 2, 2> ekf2; // stabilizer
        ExtendedKalman<double, 4, 4> ekf3; // est: x,y,center_x,center_y, measure x,y,center_x,center_y
        double updatedTime;
		double processNoise;
        double radius;
        void setTransitionMatrix();
        void setMeasurementNoise();
        void setProcessNoise();
        Eigen::Matrix<double, 2, 1> correct1(const Eigen::Matrix<double, 2, 1>& measurementArmor);
        Eigen::Matrix<double, 2, 1> correct2(const Eigen::Matrix<double, 2, 1>& measurementCenter);

        MeasureToState measureToState;
    public:
        bool is_kalman_init;
        bool get_center_init;
        double angleVelocity;
        void rebootEkf1(Eigen::Matrix<double, 2, 1> measurement);
        void rebootEkf2(Eigen::Matrix<double, 2, 1> centerMeasurement);
        void rebootEkf3(Eigen::Matrix<double, 4, 1> measurement);
        Eigen::Vector2d getEstimate(double delta_t);
        Eigen::Matrix<double, 4, 1> runKalman(const Eigen::Matrix<double, 2, 1>& measurement, double delta_t);
        OutpostFilter(double radius, double angleVelocity, double measureNoise, double processNoise);
        void setUpdatedTime(double delta_t);
        Eigen::Matrix<double, 2, 1> getCenterPosition();
    };
}
#endif //OUTPOSTFILTER_OUTPOSTFILTER_H
