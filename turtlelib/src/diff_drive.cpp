#include "turtlelib/diff_drive.hpp"
namespace turtlelib{
    DiffDrive::DiffDrive(){
        this->radius = 0.033;
        this->track = 0.018;
        this->wheel_velocity = {0,0};
        this->wheel_position = {0,0};
        this->body_twist = {0,0,0};
        this->body_position = Transform2D();
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


    void DiffDrive::FK_calculate(Vector2D new_wheel_pos){
        /// write the transformation matrix
        Vector2D temp1{(double)-1/(2*this->track),(double)1/(this->track)};
        Vector2D temp2{(double)1/2,(double)1/2};
        // calulate the wheel_config
        this->wheel_velocity = new_wheel_pos-this->wheel_position;
        this->body_twist.thetadot = dot(this->radius*temp1,this->wheel_velocity);
        this->body_twist.xdot = dot(this->radius*temp2,this->wheel_velocity);
        this->body_twist.ydot = 0;
        this->wheel_position = new_wheel_pos;

        //update the body pos given the time is 1
        Transform2D tf;
        tf = integrate_twist(this->body_twist);
        this->body_position *= tf;
    }

    Vector2D DiffDrive::IK_calculate(Twist2D& twist){
        this->wheel_velocity.x = (-this->track*twist.thetadot+twist.xdot)/this->radius;
        this->wheel_velocity.y = (this->track*twist.thetadot+twist.xdot)/this->radius;
        return this->wheel_velocity;
    }

    void DiffDrive::set_param(float radius,float track){
        this->radius = radius;
        this->track = track;
    }

    Vector2D DiffDrive::wheel_vel(){
        return this->wheel_velocity;
    }

    Vector2D& DiffDrive::wheel_pos(){
        return this->wheel_position;
    }

    Twist2D DiffDrive::bodytwist(){
        return this->body_twist;
    }

    Transform2D DiffDrive::body_pos(){
        return this->body_position;
    }
    void DiffDrive::set_body_pos(Transform2D body_pos){
        this->body_position = body_pos;
    }
    void DiffDrive::set_wheel_vel(Vector2D wheel_vel){
        this->wheel_velocity = wheel_vel;
    }








}