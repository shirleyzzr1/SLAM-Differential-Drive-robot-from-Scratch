#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include <string>
#include <cmath>

namespace turtlelib{

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        is >> v.x >> v.y;
        return is;
    }

    Vector2D Vector2Dnormalize(Vector2D v){
        double length = sqrt(pow(v.x,2) + pow(v.y,2));
        v.x/= length;
        v.y/= length;
        return v;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        
        os << "deg: " << rad2deg(tf.rotation()) << " "
        <<"x: " << tf.translation().x<< " "
        <<"y: " << tf.translation().y << std::endl;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        double degrees, dx, dy;
        is >> degrees >> dx >> dy;
        Vector2D v = {dx,dy};
        Transform2D tf2(v,deg2rad(degrees));
        tf = tf2;
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs *= rhs;
        return lhs;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        Transform2D trans2d = *this;
        this->trans = trans2d(rhs.trans);
        this->radians += rhs.radians;
        return *this;
    }

    Transform2D::Transform2D(){
        this->radians = 0;
        Vector2D v = {1,1};
        this->trans = v;
    }

    Transform2D::Transform2D(Vector2D trans){
        this->radians = 0;
        this->trans = trans;
    }

    Transform2D::Transform2D(double radians){
        this->radians = radians;
        Vector2D v = {0,0};
        this->trans = v;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        this->radians = radians;
        this->trans = trans;
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D v2 = {v.x*cos(this->radians)-v.y*sin(this->radians)+this->trans.x,
        v.x*sin(this->radians)+v.y*cos(this->radians)+this->trans.y};
        return v2;
    }

    Twist2D Transform2D::operator()(Twist2D tw) const{
        Twist2D tw1 = {tw.thetadot,
        this->trans.y*tw.thetadot+cos(this->radians)*tw.xdot-sin(this->radians)*tw.ydot,
        this->trans.x*tw.thetadot+sin(this->radians)*tw.xdot+cos(this->radians)*tw.ydot};
        return tw1;

    }

    Transform2D Transform2D::inv() const{
        double radianstmp = this->radians;
        Vector2D transtmp = this->trans;
        Vector2D trans = {-transtmp.x*cos(radianstmp)-transtmp.y*sin(radianstmp),
                    -transtmp.y*cos(radianstmp)+transtmp.x*sin(radianstmp)};
        Transform2D tf(trans,-radianstmp);
        return tf;
    }

    Vector2D Transform2D::translation() const{
        return this->trans;
    }

    double Transform2D::rotation() const{
        return this->radians;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[ " << tw.thetadot << " " << tw.xdot << " " << tw.ydot 
        << " " << "]" << std::endl;
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        is >> tw.thetadot >> tw.xdot >> tw.ydot;
        return is;
    }
}