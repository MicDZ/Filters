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
        ExtendedKalman<double, 5, 2> ekf; // est: x,y,theta,velocity,acceleration obs: x,y
        double updatedTime;
        bool is_kalman_init;
        void setTransitionMatrix();

        void setMeasurementNoise();

        void setProcessNoise();

        Eigen::Matrix<double, 2, 1> correct(Eigen::Matrix<double, 2, 1> measurement);
        MeasureToState measureToState;
    public:
        void rebootEkf(Eigen::Matrix<double, 2, 1> measurement);
        Eigen::Matrix<double, 2, 1> runKalman(Eigen::Matrix<double, 2, 1> measurement, double delta_t);
        OutpostFilter();

        void setUpdatedTime(double delta_t);
    };
}
#endif //OUTPOSTFILTER_OUTPOSTFILTER_H
