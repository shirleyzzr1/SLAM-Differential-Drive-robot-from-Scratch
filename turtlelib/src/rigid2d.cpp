#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include <string>
#include <cmath>

namespace turtlelib{

    double normalize_angle(double rad){
        // if ((rad>=-PI) & (rad<=PI)) return rad;
        // int factor = floor(rad/PI);
        // if (factor<0)factor+=1;
        // rad  = rad-factor*PI;
        // return rad;
        // reduce the angle  
        // int angle;
        // angle = int(rad2deg(rad));
        // angle =  angle % 360; 

        // // force it to be the positive remainder, so that 0 <= angle < 360  
        // angle = (angle + 360) % 360;  

        // // force into the minimum absolute value residue class, so that -180 < angle <= 180  
        // if (angle > 180)  
        //     angle -= 360; 
        // return deg2rad(angle);
        while(rad<-PI || rad>PI){
            if(rad<-PI){
                rad=rad+2*PI;
            }
            else if(rad>PI){
                rad=rad-2*PI;
            }
        }
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        char c1 = is.peek();
        if (c1=='['){
            is.get();
            v.x =  is.get();
            is.get();
            v.y = is.get();
        }
        else{
            is >> v.x >> v.y;
        }
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
        <<"y: " << tf.translation().y;
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
        Vector2D v = {0,0};
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
        << " " << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        is >> tw.thetadot >> tw.xdot >> tw.ydot;
        return is;
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
        this->x+=rhs.x;
        this->y+=rhs.y;
        return *this;
    }
    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
        this->x-=rhs.x;
        this->y-=rhs.y;
        return *this;
    }
    Vector2D & Vector2D::operator*=(const double scalar){
        this->x*=scalar;
        this->y*=scalar;
        return *this;
    }
   
    Vector2D operator+(Vector2D lhs,const Vector2D & rhs){
        
        return {lhs.x+rhs.x,lhs.y+rhs.y};
    }

    Vector2D operator-(Vector2D lhs,const Vector2D & rhs){
        return lhs-=rhs;
    }

    Vector2D  operator*(Vector2D lhs,const double scalar){
        return lhs*=scalar;
    }

    Vector2D  operator*(const double scalar,Vector2D rhs){
        return rhs*=scalar;
    }

    double dot(const Vector2D &lhs ,const Vector2D & rhs){
        return lhs.x*rhs.x+lhs.y*rhs.y;
    }

    double magnitude(const Vector2D& v){
        return sqrt(pow(v.x,2)+pow(v.y,2));
    }

    double angle(const Vector2D& v1, const Vector2D& v2){
        return acos(dot(v1,v2)/(magnitude(v1)*magnitude(v2)));
    }

    Transform2D integrate_twist(const Twist2D&tw){
        if (tw.thetadot==0){
            Vector2D v={tw.xdot,tw.ydot};
            Transform2D Tbbp(v);
            return Tbbp;
        }
        else{
            double ys = -tw.xdot / tw.thetadot;
            double xs = tw.ydot / tw.thetadot;
            Vector2D psb = {xs,ys};
            Transform2D Tsb(psb);
            Transform2D Tssp(tw.thetadot);
            Transform2D Tbs = Tsb.inv();
            Transform2D Tbbp =  Tbs * Tssp * Tsb;
            return Tbbp;
        }
    }




}