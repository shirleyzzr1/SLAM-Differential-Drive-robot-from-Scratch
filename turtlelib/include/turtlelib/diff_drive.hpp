#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Differential robot drive
#include "turtlelib/rigid2d.hpp"
namespace turtlelib{
    /// \brief a class function for the differential drive robot
    class DiffDrive{
    private: 
        float radius;
        float track;

        Vector2D wheel_velocity;
        Vector2D wheel_position;
        Twist2D body_twist;
        Transform2D body_position;
    public:
        /// \brief use some default settings to create a diffdrive
        DiffDrive();
        
        /// \brief give the basic parameter of calculating 
        /// \param radius: the radius of the wheels
        /// \param track: the the wheel track
        DiffDrive(float radius,float track);

        /// \brief use forward kinematics to update the body 
        /// configuration given the new wheel position
        /// \param new_wheel_pos: the new pos of the wheel
        /// \param period: the period for the robot moving
        void FK_calculate(Vector2D new_wheel_pos);

        void FK_calculate_vel(Vector2D wheel_vel);


        /// \brief use inverse kinematics to update the wheel
        /// velocity given the body twist
        /// \param twist the body twist
        Vector2D IK_calculate(Twist2D& twist); 

        /// \brief set the radius and track of the robot
        /// \param radius: the given wheel_raidus
        /// \param track: the given body track
        void set_param(float radius,float track);

        /// \brief set the body_pos of the robot
        /// \param body_pos: the new body_pos;
        void set_body_pos(Transform2D body_pos);
        
        /// \brief set the wheel_velocity of the robot
        /// \param wheel_vel: the new wheel velocity
        void set_wheel_vel(Vector2D wheel_vel);

        /// \brief set the weel position of the robot
        /// \param wheel_pos: the new position of the wheel
        void set_wheel_pos(Vector2D wheel_pos);

        /// \brief get the private wheel_velocity
        Vector2D wheel_vel();

        /// \brief get the private wheel_position
        Vector2D& wheel_pos();

        /// \brief get the private body_twist
        Twist2D bodytwist();

        /// \brief get the private body_pos
        Transform2D body_pos();


    };
}
#endif
