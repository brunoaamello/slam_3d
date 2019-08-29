#ifndef SCANNER_TYPE
#define SCANNER_TYPE

// Own codes
#include "angle.hh"
#include "scan.hh"

// Ros libraries
#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"

// External libraries
#include <queue>
#include <iostream>
#include <mutex>


template<class numeric>
class Scanner_T{
    // aliases
    using Scan = Scan_T<numeric>;
    using queue = std::queue<Scan*>;
    using Time = ros::Time;
    // public attributes
    public:

    // private attributes
    private:
        unsigned _queue_warning;
        unsigned _queue_size;
        queue _scan_queue;
        std::mutex _queue_mutex;
        ros::NodeHandle* _node;
        ros::Subscriber _subscriber;

    // public functions
    public:
        Scanner_T(ros::NodeHandle* n, const char* topic = "scan", unsigned q_size = 1024, unsigned q_warning = 128){
            _node = n;
            _queue_size = q_size;
            _queue_warning = q_warning;
            _subscriber = _node->subscribe(topic, 1000, &Scanner_T::scanCallback, this);
        }
        ~Scanner_T(){
            while(!_scan_queue.empty()){
                delete getScan();
            }
        }
        Scan* getScan(){
            _queue_mutex.lock();
            Scan* front = _scan_queue.front();
            _scan_queue.pop();
            _queue_mutex.unlock();
            return front;
        }
        unsigned queueSize(){
            return _scan_queue.size();
        }

    // private functions
    private:
        void scanCallback(const sensor_msgs::LaserScan scan_data){
            Time receive_time = Time::now();
            numeric elapsed_time = ((numeric)receive_time.toNSec())/1e9;

            Scan* scan = new Scan(elapsed_time, scan_data);

            _queue_mutex.lock();
            _scan_queue.push(scan);
            _queue_mutex.unlock();

            if(_scan_queue.size() > _queue_size){
                std::cerr << "Scanner queue full, discarding data!" << std::endl;
                delete getScan();
            }else if(_scan_queue.size() == _queue_warning){
                std::cerr << "Scanner queue reached warning level!" << std::endl;
            }

        }
};

#endif