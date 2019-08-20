#ifndef POINT_TYPE
#define POINT_TYPE

#include <cmath>

#include "angle.hh"

template<class numeric>
class Point_T{
    using Angle = Angle_T<numeric>;
    private:
        numeric _x, _y, _z;    

    public:
        Point_T(numeric x=0, numeric y=0, numeric z=0){
            _x = x;
            _y = y;
            _z = z;
        }
        // Copy constructor
        Point_T(const Point_T& a){
            _x = a.x();
            _y = a.y();
            _z = a.z();
        }
        // Spheric coordinate constructor
        Point_T(numeric r, Angle theta, Angle phi = Angle::toRad(90)){
            _x = r*theta.cos()*phi.sin();
            _y = r*theta.sin()*phi.sin();
            _z = r*phi.cos();
        }
        // Cylindrical coordinate constructor
        Point_T(numeric ro, Angle theta, numeric z){
            _x = ro*theta.cos();
            _y = ro*theta.sin();
            _z = z;
        }

        numeric x() const{
            return _x;
        }
        numeric y() const{
            return _y;
        }
        numeric z() const{
            return _z;
        }
        

        // Static functions
        static const numeric dot(const Point_T a, const Point_T b){
            return (a.x()*b.x())+(a.y()*b.y())+(a.z()*b.z());
        }
        static const Point_T cross(const Point_T a, const Point_T b){
            numeric local_x = (a.y()*b.z())-(a.z()*b.y());
            numeric local_y = (a.z()*b.x())-(a.x()*b.z());
            numeric local_z = (a.x()*b.y())-(a.y()*b.x());
            return Point_T(local_x, local_y, local_z);
        }

        // Assignment operators
        Point_T& operator+=(Point_T a){
            _x += a.x();
            _y += a.y();
            _z += a.z();
        }
        Point_T& operator-=(Point_T a){
            _x -= a.x();
            _y -= a.y();
            _z -= a.z();
        }
        Point_T& operator*=(numeric a){
            _x *= a;
            _y *= a;
            _z *= a;
        }
        Point_T& operator/=(numeric a){
            _x /= a;
            _y /= a;
            _z /= a;
        }
        Point_T& operator^=(numeric a){
            _x = std::pow(_x, a);
            _y = std::pow(_y, a);
            _z = std::pow(_z, a);
        }
        Point_T& operator^=(Point_T a){
            *this = Point_T::cross(*this, a);
        }
        
        // Arithmetic operators
        Point_T operator+(numeric a) const{
            return Point_T(this->x()+a, this->y()+a, this->z()+a);
        }
        Point_T operator-(numeric a) const{
            return Point_T(this->x()-a, this->y()-a, this->z()-a);
        }
        Point_T operator*(numeric a) const{
            return Point_T(this->x()*a, this->y()*a, this->z()*a);
        }
        Point_T operator/(numeric a) const{
            return Point_T(this->x()/a, this->y()/a, this->z()/a);
        }
        Point_T operator^(numeric a) const{
            return Point_T(std::pow(this->x(), a), std::pow(this->y(), a), std::pow(this->z(), a));
        }
        Point_T operator^(Point_T a) const{
            return Point_T::cross(this, a);
        }
        numeric operator*(Point_T a) const{
            return Point_T::dot(this, a);
        }
        



};

#endif