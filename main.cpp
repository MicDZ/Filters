#include <iostream>
#include <OutpostSim.h>
#include <thread>
#include <UDPSender.hpp>
#include <OutpostFilter.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    Mat img(2000, 2000, CV_8UC3, Scalar(0, 0, 0));

    ly::OutpostSim outpostSim(Eigen::Matrix<double, 3, 1>(0.2, 0.5, 0.5), 0.3, 2., 0.01);
    srand(1273);
    ly::UDPSender sender("10.13.49.42", 1347);
    ly::OutpostFilter *filter[3];
    filter[0] = new ly::OutpostFilter(0.3, 2., .01, 0.001);

    double stalledTime = .01;
    Eigen::Vector4d pos;
    while(1) {
        vector<Eigen::Matrix<double, 3, 1> > points = outpostSim.update(0.01);
        //double angle = outpostSim.getAngle();
        Eigen::Matrix<double, 3, 1> armorPosition = points[1];
        cout <<"["<< armorPosition <<"]"<< endl;
        if(armorPosition(0) < 1e-5 && armorPosition(1) < 1e-5 && armorPosition(2)< 1e-5) {
            stalledTime+=0.01;
            Eigen::Vector2d estArmor = filter[0]->getEstimate(stalledTime);
            //estCenterPosition = filter[0]->
            circle(img, Point2d(estArmor(0) * 1000 + 960, estArmor(1) * 1000 + 540), 1, Scalar(0, 255, 0), -1);
            imshow("img", img);
            waitKey(1);
            continue;
        }
        ly::Outpost outpost;
        outpost.x = (float)armorPosition(0);
        outpost.y = (float)armorPosition(1);
        outpost.z = (float)armorPosition(2);



        pos<<outpost.x,outpost.y,pos(2),pos(3);
        cout<<"<<<<<<<<<<<<<<<<<<<"<<pos(2)<<" "<<pos(3)<<endl;
        pos=filter[0]->runKalman(pos.head(2), stalledTime);
        stalledTime = 0.01;

        outpost.x_est=pos(0);
        outpost.y_est=pos(1);
        Eigen::Vector2d centerPosition=outpostSim.getCenterPosition().head(2);
        Eigen::Vector2d estCenterPosition;
        estCenterPosition<<pos(2),pos(3);
        cout<<"estCenterPosition:"<<estCenterPosition<<endl;
        //sender.send(outpost);
        circle(img, Point2d(armorPosition(0) * 1000 + 960, armorPosition(1) * 1000 + 540), 1, Scalar(0, 0, 255), -1);
        circle(img, Point2d(outpost.x_est * 1000 + 960, outpost.y_est * 1000 + 540), 1, Scalar(0, 255, 0), -1);
        circle(img, Point2d(centerPosition(0) * 1000 + 960, centerPosition(1) * 1000 + 540), 5, Scalar(255, 0, 0), -1);
        circle(img, Point2d(estCenterPosition(0) * 1000 + 960, estCenterPosition(1) * 1000 + 540), 2, Scalar(255, 255, 0), -1);
        imshow("img", img);
        waitKey(1);
    }
    return 0;
}
