#ifndef POINT_TYPE
#define POINT_TYPE

// Own codes
#include "angle.hh"

// External libraries
#include <cmath>


template<class numeric>
class Point_T{
    // aliases
    using Angle = Angle_T<numeric>;

    // public attributes
    public:

    // private attributes
    private:
        numeric _x, _y, _z;    

    // public functions
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
        /*
        const std::string str() const{
            std::stringstream ss;
            ss << "[" << x() << ", " << y() << ", " << z() << "]";
            return ss.str();
        }
        */



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



};

#endif