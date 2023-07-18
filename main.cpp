#include <iostream>
#include <OutpostSim.h>
#include <thread>
#include <UDPSender.hpp>
#include <OutpostFilter.h>

using namespace std;

int main() {
    ly::OutpostSim outpostSim(Eigen::Matrix<double, 3, 1>(4, 0, 1.5), 0.3, 1);
    srand(123);
    ly::UDPSender sender("10.13.49.42", 1347);
    while(1) {

        vector<Eigen::Matrix<double, 3, 1> > points = outpostSim.update(0.01);
        //double angle = outpostSim.getAngle();
        Eigen::Matrix<double, 3, 1> armorPosition = points[1];
        cout <<"["<< armorPosition <<"]"<< endl;

        ly::Outpost outpost;
        outpost.x = (float)armorPosition(0);
        outpost.y = (float)armorPosition(1);
        outpost.z = (float)armorPosition(2);

        ly::OutpostFilter filter;
        Eigen::Vector2d pos;
        pos<<outpost.x,outpost.y;

        pos=filter.runKalman(pos, 0.01);
        outpost.x_est=pos(0);
        outpost.y_est=pos(1);

        sender.send(outpost);

        this_thread::sleep_for(chrono::milliseconds(10));
    }
    return 0;
}
