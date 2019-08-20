#ifndef SCAN_TYPE

#define SCAN_TYPE

#include "angle.hh"

#include <vector>
#include <tuple>
#include <string>
#include "sensor_msgs/LaserScan.h"

template<class numeric>
class Scan_T{
    using Angle = Angle_T<numeric>;
    private:
        numeric _scan_reception_time;
        std::vector<numeric> _time;
        std::vector<Angle> _angles;
        std::vector<numeric> _range;

    public:
        Scan_T(numeric reception_time, const sensor_msgs::LaserScan scan_data){
            _scan_reception_time = reception_time;
            setScanData(scan_data);
        }

        const numeric getReceptionTime() {
            return _scan_reception_time;
        }
        const std::vector<numeric> getTimes() const{
            return _time;
        }
        const std::vector<Angle> getAngles() const{
            return _angles;
        }
        const std::vector<numeric> getRanges() const{
            return _range;
        }
        const std::tuple<std::vector<Angle>, std::vector<numeric> > getData() const{
            return make_tuple(getAngles(), getRanges());
        }
        const std::tuple<std::vector<numeric>, std::vector<Angle>, std::vector<numeric> > getTimedData() const{
            return make_tuple(getTimes(), getAngles(), getRanges());
        }

        const std::string getDataString() const{
            std::stringstream ss;
            ss << "_____________________________________" << std::endl;
            ss << "| id|     time     |  angle  | range |" << std::endl;
            for(unsigned i = 0; i < _time.size(); i++){
                ss << "| " << i << " | " << _time[i] << " | " << _angles[i].deg() << " | " << _range[i] << " |" << std::endl;
            }
            ss << "----------------------------------------" << std::endl;

            return ss.str();
        }

    private:
        void setScanData(const sensor_msgs::LaserScan scan_data){
            int count = floor((scan_data.angle_max-scan_data.angle_min)/scan_data.angle_increment);
            numeric local_range;
            numeric time_offset = -count*scan_data.time_increment;
            int idx = 0;
            Angle angle = scan_data.angle_min;
            while(angle <= scan_data.angle_max){
                local_range = scan_data.ranges[idx];
                if(!std::isnan(local_range)){
                    if(scan_data.range_min <= local_range &&  local_range <= scan_data.range_max){
                        _time.push_back(_scan_reception_time+time_offset);
                        _angles.push_back(angle);
                        _range.push_back(local_range);
                    }
                }
                time_offset += scan_data.time_increment;
                idx++;
                angle += scan_data.angle_increment;
            }
        }



};

#endif