#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <iostream>
#include <thread>
#include <ctime>

#include "classes/point.hh"
#include "classes/angle.hh"
#include "classes/scan.hh"
#include "classes/scanner.hh"

using fp = double;

using Point = Point_T<fp>;
using Angle = Angle_T<fp>;
using Scan = Scan_T<fp>;
using Scanner = Scanner_T<fp>;

void runner(Scanner* my_scanner){
    Scan* local_scan;
    while(true){
        if(my_scanner->queueSize() > 0){
            local_scan = my_scanner->getScan();
            std::cout << local_scan->getDataString() << std::endl;
            usleep(10000); // Used to avoid race condition between cout and delete
            delete local_scan;
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "slam_3d");

    ros::NodeHandle n;

    printf("This program does run.\n");

    Scanner* my_scanner = new Scanner(&n, "scan");

    std::thread myRunner(runner, my_scanner);

    ros::spin();

}
