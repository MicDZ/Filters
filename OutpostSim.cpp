//
// Created by MicDZ on 2023/7/16.
//
#include <OutpostSim.h>
using namespace ly;
double uniformRand(double min, double max) {
    return min + (max - min) * rand() / RAND_MAX;
}

double guassRand(double mean, double sigma) {
    double x, y, r2;
    do {
        x = -1.0 + 2.0 * uniformRand(0.0, 1.0);
        y = -1.0 + 2.0 * uniformRand(0.0, 1.0);
        r2 = x * x + y * y;
    } while (r2 > 1.0 || r2 == 0.0);
    return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

double normAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Eigen::Matrix<double, 3, 1> getNoise(int randSeed, double mean=0, double sigma=0.01) {
    //srand((unsigned)randSeed);
    Eigen::Matrix<double, 3, 1> noise;
    noise << guassRand(mean, sigma), guassRand(mean, sigma), guassRand(mean, sigma);
    return noise;
}

OutpostSim::OutpostSim(Eigen::Matrix<double, 3, 1> centerPosition, double radius, double angularVelocity) {
    this->centerPosition = centerPosition;
    this->radius = radius;
    this->height = centerPosition(2);
    this->angularVelocity = angularVelocity;
    this->armorPosition = armorPosition;
}

Eigen::Matrix<double, 3, 1> OutpostSim::getCenterPosition() {
    return this->centerPosition;
}

double OutpostSim::getRadius() const {
    return this->radius;
}

double OutpostSim::getHeight() const {
    return this->height;
}

double OutpostSim::getAngularVelocity() const {
    return this->angularVelocity;
}

double OutpostSim::getAngle() const {
    return angle;
}

Eigen::Matrix<double, 3, 1> OutpostSim::getArmorPosition() {
    return this->armorPosition;
}

std::vector<Eigen::Matrix<double, 3, 1> >OutpostSim::update(double time) {

    Eigen::Matrix<double, 3, 1> measureArmorPosition = this->armorPosition;
    Eigen::Matrix<double, 3, 1> measureCenterPosition = this->centerPosition;
    this->angle += this->angularVelocity * time;

    // norm and update angle
    double angle_ = normAngle(this->angle);
    this->angle = angle_;

    std::vector<double> armorsAngle;

    armorsAngle.push_back(angle_);
    armorsAngle.push_back(normAngle(angle_ + M_PI * 2 / 3));
    armorsAngle.push_back(normAngle(angle_ + M_PI * 4 / 3));

//    bool gotArmorInSight = false;
//    for(double armor : armorsAngle) {
//        if(armor>0 && armor<60.0/360*2*M_PI) { // 60 degree
//            angle_ = armor;
//            gotArmorInSight=true;
//            std::cout<<"angle_: "<<angle_<<std::endl;
//            break;
//        }
//    }
//    if(!gotArmorInSight) {
//        Eigen::Matrix<double, 3, 1> zeros = Eigen::Matrix<double, 3, 1>::Zero();;
//        return std::vector<Eigen::Matrix<double, 3, 1> >{zeros, zeros};
//
//    }
    // add gaussian noise to armorPosition

    Eigen::Matrix<double, 3, 1> posNoise = getNoise(rand());
    Eigen::Matrix<double, 3, 1> centerNoise = getNoise(rand());

    measureArmorPosition(0) = this->radius * cos(angle_) + this->centerPosition(0);
    measureArmorPosition(1) = this->radius * sin(angle_) + this->centerPosition(1);
    measureArmorPosition(2) = this->height;

    // update armorPosition
    this->armorPosition = measureArmorPosition;

    // add measure noise
    measureArmorPosition += posNoise;
    measureCenterPosition += centerNoise;

    std::vector<Eigen::Matrix<double, 3, 1> > result;

    result.push_back(measureCenterPosition);
    result.push_back(measureArmorPosition);

    return result;
}

