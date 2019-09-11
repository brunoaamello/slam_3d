#ifndef ANGLE_TYPE
#define ANGLE_TYPE

// External libraries
#include <cmath>
#include <stdexcept>

template<class numeric>
class Angle_T{
    // public attributes
    public:
        static constexpr numeric pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647093844609550582231725359408128481117450284102701938521105559644622948954930381964428810975665933446128475648233786783165271201909145648566923460348610454326648213393607260249141273;
    
    // private attributes
    private:
        numeric _angle;

    // public functions
    public:
        Angle_T(numeric a = 0){
            _angle = a;
        }

        // Static functions
        static const numeric toDeg(const numeric a){
            return a*(180/pi);
        }
        static const numeric toRad(const numeric a){
            return a/(180/pi);
        }
        static const numeric cosine(const numeric a){
            return std::cos(a);
        }
        static const numeric sine(const numeric a){
            return std::sin(a);
        }
        static const numeric tangent(const numeric a){
            return std::tan(a);
        }
        static const Angle_T arccosine(const numeric a){
            return Angle_T(_acos(a));
        }
        static const Angle_T arcsine(const numeric a){
            return Angle_T(_asin(a));
        }
        static const Angle_T arctangent(const numeric a){
            return Angle_T(std::atan(a));
        }
        static const Angle_T unwrapAngle(const numeric a){
            return Angle_T(_unwrap(a));
        }
        

        // Assignment operators
        Angle_T& operator=(numeric a){
            _angle = a;
        }
        Angle_T& operator+=(numeric a){
            _angle += a;
        }
        Angle_T& operator-=(numeric a){
            _angle -= a;
        }
        Angle_T& operator*=(numeric b){
            _angle *= b;
        }
        Angle_T& operator/=(numeric b){
            _angle /= b;
        }
        Angle_T& operator%=(numeric b){
            _angle = _modulo(_angle, b);
        }
        Angle_T& operator^=(numeric b){
            _angle = std::pow(_angle, b);
        }

        // Arithmetic operators
        Angle_T operator+(numeric b) const{
            return Angle_T(_angle+b);
        }
        Angle_T operator-(numeric b) const{
            return Angle_T(_angle-b);
        }
        Angle_T operator*(numeric b) const{
            return Angle_T(_angle*b);
        }
        Angle_T operator/(numeric b) const{
            return Angle_T(_angle/b);
        }
        Angle_T operator%(numeric b) const{
            return Angle_T(_modulo(_angle, b));
        }
        Angle_T operator^(numeric b) const{
            return Angle_T(std::pow(_angle,b));
        }
        
        // Cast operator
        operator numeric(){
            return _angle;
        } 
        
        // Other functions
        const numeric deg() const{
            return toDeg(_angle);
        }
        const numeric rad() const{
            return _angle;
        }
        const numeric cos() const{
            return cosine(_angle);
        }
        const numeric sin() const{
            return sine(_angle);
        }
        const numeric tan() const{
            return tangent(_angle);
        }
        const numeric unwrap() const{
            _angle = unwrapAngle(_angle);
            return _angle;
        }
    
    // private functions
    private:
        static const numeric _modulo(const numeric a, const numeric b){
            if(b == 0.0){
                throw std::domain_error("Division by 0!");
            }
            numeric local_a = a;
            bool negative = false;
            if(local_a<0.0){
                negative = true;
                local_a = -local_a;
            }
            while(local_a>=b){
                local_a -= b;
            }
            if(negative){
                local_a = -local_a;
            }
            return local_a;
        }
        static const numeric _acos(const numeric a){
            if(-1 > a || a > 1){
                throw std::out_of_range("Cosine out of [-1,1]");
            }
            return std::acos(a);
        }
        static const numeric _asin(const numeric a){
            if(-1 > a || a > 1){
                throw std::out_of_range("Sine out of [-1,1]");
            }
            return std::asin(a);
        }
        static const numeric _unwrap(const numeric a){
            numeric local_a = _modulo(a, 2*pi);
            if(local_a < 0.0){
                local_a += 2*pi;
            }
            return local_a;
        }

};

#endif