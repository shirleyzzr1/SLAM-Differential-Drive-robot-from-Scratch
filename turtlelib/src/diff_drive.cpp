#include "turtlelib/diff_drive.hpp"
#include <iostream>
namespace turtlelib{
    DiffDrive::DiffDrive(){
        this->radius = 0.033;
        this->track = 0.08;
        this->wheel_velocity = {0,0};
        this->wheel_position = {0,0};
        this->body_twist = {0,0,0};
        this->body_position = Transform2D();
    }

    DiffDrive::DiffDrive(float radius,float track):
    radius(radius),track(track){}

    void DiffDrive::FK_calculate(Vector2D new_wheel_pos){
        /// write the transformation matrix
        Vector2D temp1{(double)-1/(2*this->track),(double)1/(2*this->track)};
        Vector2D temp2{(double)1/2,(double)1/2};

        // calulate the wheel_velocity, given the time unit is 1
        this->wheel_velocity = new_wheel_pos-this->wheel_position;
        // std::cout << "wheel_postion" << new_wheel_pos <<std::endl;
        // std::cout << "this_wheel_position" << this->wheel_position;
        // std::cout << "wheel_velocity" << this->wheel_velocity<< std::endl;

        //calculate the body twist;
        this->body_twist.thetadot = dot(this->radius*temp1,this->wheel_velocity);
        this->body_twist.xdot = dot(this->radius*temp2,this->wheel_velocity);
        this->body_twist.ydot = 0;
        this->wheel_position = new_wheel_pos;

        //update the body translation from the previsou configuration given the time is 1
        Transform2D tf;
        tf = integrate_twist(this->body_twist);
        this->body_position *= tf;
    }

    Vector2D DiffDrive::IK_calculate(Twist2D& twist){
        if(twist.ydot){
            throw std::logic_error("the wheels cannot slipping");
        }
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
    void DiffDrive::set_wheel_pos(Vector2D wheel_pos){
        this->wheel_position = wheel_pos;
    }








}