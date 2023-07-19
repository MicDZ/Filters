//
// Created by MicDZ on 2023/7/16.
//

#ifndef OUTPOSTFILTER_OUTPOSTSIM_H
#define OUTPOSTFILTER_OUTPOSTSIM_H
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>
namespace ly {

    class OutpostSim {
    private:
        Eigen::Matrix<double, 3, 1> centerPosition;
        double radius, height, angle{};
        double angularVelocity;
        double sigma;
        //double angularAcceleration;
        Eigen::Matrix<double, 3, 1> armorPosition;

    public:
        OutpostSim(Eigen::Matrix<double, 3, 1>, double radius, double angularVelocity, double sigma);

        Eigen::Matrix<double, 3, 1> getCenterPosition();

        double getRadius() const;

        double getHeight() const;

        double getAngularVelocity() const;

        double getAngle() const;

        //double getAngularAcceleration();
        Eigen::Matrix<double, 3, 1> getArmorPosition();
//    void setCenterPosition(Eigen::Matrix<double, 3, 1> centerPosition);
//    void setRadius(double radius);
//    void setHeight(double height);
//    void setAngularVelocity(double angularVelocity);
        //void setAngularAcceleration(double angularAcceleration);
//    void setArmorPosition(Eigen::Matrix<double, 3, 1> armorPosition);

        std::vector<Eigen::Matrix<double, 3, 1>> update(double time);


    };
}
#endif //OUTPOSTFILTER_OUTPOSTSIM_H
