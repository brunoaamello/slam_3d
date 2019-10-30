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
#include <array>
#include <iostream>
#include <mutex>


template<class numeric, unsigned queue_size = 1024>
class Scanner_T{
    // aliases
    using Scan = Scan_T<numeric>;
    using array = std::array<Scan*, queue_size>;
    using Time = ros::Time;
    // public attributes
    public:

    // private attributes
    private:
        unsigned _queue_warning;
        array _scan_array;
        unsigned _array_front, _array_back;
        std::mutex _queue_mutex;
        ros::NodeHandle* _node;
        ros::Subscriber _subscriber;


    // public functions
    public:
        Scanner_T(ros::NodeHandle* n, const char* topic = "/scan", unsigned q_warning = 128){
            _node = n;

            _array_front = 0;
            _array_back = 0;

            _queue_warning = q_warning;
            _subscriber = _node->subscribe(topic, 1000, &Scanner_T::scanCallback, this);
        }
        ~Scanner_T(){
            while(queueSize()>0){
                delete getScan();
            }
        }
        Scan* getScan(){
            _queue_mutex.lock();
            Scan* front = _scan_array[_array_front];
            incFront();
            _queue_mutex.unlock();
            return front;
        }
        unsigned queueSize(){
            unsigned size;
            if(_array_back >= _array_front){
                size = _array_back - _array_front;
            }else{
                size = (queue_size - _array_front) + _array_back;
            }
        }

    // private functions
    private:
        void incFront(){
            _array_front++;
            if(_array_front == queue_size){
                _array_front = 0;
            }
        }

        void incBack(){
            _array_back++;
            if(_array_back == queue_size){
                _array_back = 0;
            }
        }

        void scanCallback(const sensor_msgs::LaserScan scan_data){
            Time receive_time = Time::now();
            numeric elapsed_time = ((numeric)receive_time.toNSec())/1e9;

            Scan* scan = new Scan(elapsed_time, scan_data);

            _queue_mutex.lock();
            _scan_array[_array_back] = scan;
            incBack();
            _queue_mutex.unlock();

            if(queueSize() == _queue_warning){
                std::cerr << "Scanner queue reached warning level!" << std::endl;
            }


        }
};

#endif