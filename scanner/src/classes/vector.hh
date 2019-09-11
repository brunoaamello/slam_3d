#ifndef VECTOR_TYPE
#define VECTOR_TYPE

// Own codes
#include "point.hh"
#include "angle.hh"


// External libraries
#include <cmath>


template<class numeric>
class Vector_T : public Point_T<numeric>{
    // aliases
    using Angle = Angle_T<numeric>;
    using Point = Point_T<numeric>;

    // public attributes
    public:

    // private attributes
    private:
        numeric _x, _y, _z;    

    // public functions
    public:
        Vector_T(numeric x=0, numeric y=0, numeric z=0) : Point(x, y, z){};
        // Copy constructors
        Vector_T(const Vector_T& a) : Point(a){};
        Vector_T(const Point& a) : Point(a){};
        // Spheric coordinate constructor
        Vector_T(numeric r, Angle theta, Angle phi = Angle::toRad(90)) : Point(r, theta, phi){};
        // Cylindrical coordinate constructor
        Vector_T(numeric ro, Angle theta, numeric z) : Point(ro, theta, z){};

        // Static functions
        static const numeric dot(const Vector_T a, const Vector_T b){
            return (a.x()*b.x())+(a.y()*b.y())+(a.z()*b.z());
        }
        static const Vector_T cross(const Vector_T a, const Vector_T b){
            numeric local_x = (a.y()*b.z())-(a.z()*b.y());
            numeric local_y = (a.z()*b.x())-(a.x()*b.z());
            numeric local_z = (a.x()*b.y())-(a.y()*b.x());
            return Vector_T(local_x, local_y, local_z);
        }
        static const numeric norm(const Vector_T a){
            return std::sqrt(Vector_T::dot(a, a));
        }
        static const Vector_T normal(const Vector_T a, const Vector_T b){
            Vector_T local_vector = a.cross(b);
            return local_vector/local_vector.norm();
        }
        static const Angle angle(const Vector_T a, const Vector_T b){
            numeric cosine = a.dot(b)/(a.norm()*b.norm());
            return Angle::arccosine(cosine);
        }
        // Specified functions
        numeric dot(const Vector_T a) const{
            return Vector_T::dot(*this, a);
        }
        Vector_T cross(const Vector_T a) const{
            return Vector_T::cross(*this, a);
        }
        numeric norm() const{
            return Vector_T::norm(*this);
        }

        // Assignment Operators
        Vector_T& operator^=(Vector_T a){
            *this = Vector_T::cross(*this, a);
        }
        
        // Arithmetic operators
        Vector_T operator^(Vector_T a) const{
            return this->cross(a);
        }
        numeric operator*(Vector_T a) const{
            return this->dot(a);
        }
        



};

#endif