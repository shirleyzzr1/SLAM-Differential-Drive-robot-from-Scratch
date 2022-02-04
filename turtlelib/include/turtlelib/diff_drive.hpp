#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Differential robot drive
#include "turtlelib/rigid2d.hpp"
namespace turtlelib{
    struct Configure{
        double theta;
        double x;
        double y;
    };
    ///add to configure together
    Configure & operator+(Configure& lhs,Configure& rhs);

    /// \brief a class function for the differential drive robot
    class DiffDrive{
    private: 
        float radius;
        float track;

        Vector2D wheel_velocity;
        Vector2D wheel_pos;
        Twist2D body_twist;
        Configure body_pos;
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
        void FK_calculate(Vector2D new_wheel_pos,double period);

        /// \brief use inverse kinematics to update the wheel
        /// velocity given the body twist
        /// \param twist the body twist
        Vector2D IK_calculate(Twist2D& twist); 
        
    };
}
#endif
