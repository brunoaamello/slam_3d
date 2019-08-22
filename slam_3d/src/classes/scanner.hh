#ifndef SCANNER_TYPE

#define SCANNER_TYPE

#include "angle.hh"
#include "scan.hh"

#include <queue>
#include <ctime>
#include <iostream>
#include <mutex>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"



template<class numeric>
class Scanner_T{
    using Scan = Scan_T<numeric>;
    using queue = std::queue<Scan*>;
    private:
        unsigned _queue_warning;
        unsigned _queue_size;
        queue _scan_queue;
        time_t _start_time;
        numeric _time_offset;
        std::mutex _queue_mutex;
        ros::NodeHandle* _node;
        ros::Subscriber _subscriber;

    private:
        void scanCallback(const sensor_msgs::LaserScan scan_data){
            time_t receive_time = std::time(NULL);
            numeric elapsed_time = (numeric) std::difftime(receive_time, _start_time);
            elapsed_time += _time_offset;

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

    public:
        Scanner_T(ros::NodeHandle* n, const char* topic = "scan", unsigned q_size = 1024, unsigned q_warning = 128, numeric offset = 0){
            _node = n;
            _queue_size = q_size;
            _queue_warning = q_warning;
            _time_offset = offset;
            _start_time = std::time(NULL);
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
};

#endif