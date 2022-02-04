#include "turtlelib/diff_drive.hpp"
namespace turtlelib{
    DiffDrive::DiffDrive(){
        this->radius = 0.033;
        this->track = 0.018;
    }

    DiffDrive::DiffDrive(float radius,float track):
    radius(radius),track(track){}

    ///\brief add two robot configure together
    Configure & operator+(Configure& lhs,Configure& rhs){
        Configure conf;
        conf.theta = lhs.theta + rhs.theta;
        conf.x = lhs.x + rhs.x;
        conf.y = lhs.y + rhs.y;
    }


    void DiffDrive::FK_calculate(Vector2D new_wheel_pos,double period){
        ///calculate the wheel_relocity 
        Vector2D wheel_velocity = (new_wheel_pos - this->wheel_pos)*(1/period);
        //update the new wheel pos
        this->wheel_pos = new_wheel_pos;
        /// write the transformation matrix
        Vector2D temp1{-1/(2*this->track),1/(this->track)};
        Vector2D temp2{1/2,1/2};
        // calulate the wheel_config
        this->body_twist.thetadot = dot(this->radius*temp1,wheel_velocity);
        this->body_twist.xdot = dot(this->radius*temp2,wheel_velocity);

        //update the body pos given the time is 1
        Configure config_diff{body_twist.thetadot*period,body_twist.xdot*period,body_twist.ydot*period};
        this->body_pos = this->body_pos+config_diff;

    }

    Vector2D DiffDrive::IK_calculate(Twist2D& twist){
        this->wheel_velocity.x = (-this->track/2*twist.thetadot+twist.xdot)/this->radius;
        this->wheel_velocity.y = (this->track/2*twist.thetadot+twist.xdot)/this->radius;
    }



}