// Own codes
#include "classes/point.hh"
#include "classes/vector.hh"
#include "classes/angle.hh"
#include "classes/scan.hh"
#include "classes/scanner.hh"
#include "scanner/lidar_data.h"

// Ros libraries
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

// External libraries
#include <sstream>
#include <iostream>
#include <thread>
#include <ctime>

// Aliases
using fp = double;
using Point = Point_T<fp>;
using Vector = Vector_T<fp>;
using Angle = Angle_T<fp>;
using Scan = Scan_T<fp>;
using Scanner = Scanner_T<fp>;

// Function headers
void runner(Scanner* my_scanner, ros::Publisher* pub);

// Main
int main(int argc, char **argv){

    ros::init(argc, argv, "scanner");

    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<scanner::lidar_data>("lidar_data", 1000);

    printf("This program does run.\n");  


    Scanner* my_scanner = new Scanner(&n, "/scan");

    std::thread myRunner(runner, my_scanner, &pub);

    ros::spin();

    myRunner.join();

    delete my_scanner;

}

// Functions Bodies
void runner(Scanner* my_scanner, ros::Publisher *pub){
    Scan* local_scan;
    std::string data_string;
    

    while(ros::ok()){
        if(my_scanner->queueSize() > 0){
            local_scan = my_scanner->getScan();
            data_string = local_scan->getDataString();
            //std::cout << data_string << std::endl;
            pub->publish(local_scan->getDataMessage());
            delete local_scan;
            ros::spinOnce();
        }
    }
}
