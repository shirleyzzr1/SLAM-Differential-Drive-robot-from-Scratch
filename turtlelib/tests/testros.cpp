#include<catch_ros/catch.hpp>
#include"turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
using namespace turtlelib;
TEST_CASE("test_forward_kinematics",""){
    DiffDrive diff;
    diff.set_wheel_pos({5,5});
    Vector2D wheel_pos = {10,10};
    diff.FK_calculate(wheel_pos);
    REQUIRE(diff.bodytwist().thetadot==Approx(0));
    REQUIRE(diff.bodytwist().xdot==Approx(0));

}
TEST_CASE("test_normalized_angle_function","test vector"){
    REQUIRE(normalize_angle(PI)==Approx(PI));
    REQUIRE(normalize_angle(-PI)==Approx(-PI));
    REQUIRE(normalize_angle(0)==Approx(0));
    REQUIRE(normalize_angle(-PI/4)==Approx(-PI/4));
    REQUIRE(normalize_angle(3*PI/2)==Approx(PI/2));
    REQUIRE(normalize_angle(-5*PI/2)==Approx(-PI/2));

}
TEST_CASE("test_twist","integrate_twist_function"){
    Twist2D tw1 = {0,2,3};
    Twist2D tw2 = {1,0,0};
    Twist2D tw3 = {1,2,3};

    REQUIRE(integrate_twist(tw1).rotation()==0);
    REQUIRE(integrate_twist(tw1).translation().x==2);
    REQUIRE(integrate_twist(tw1).translation().y==3);

    REQUIRE(integrate_twist(tw2).rotation()==1);
    REQUIRE(integrate_twist(tw2).translation().x==0);
    REQUIRE(integrate_twist(tw2).translation().y==0);

    REQUIRE(integrate_twist(tw3).rotation()==1);
    REQUIRE(integrate_twist(tw3).translation().x==2);
    REQUIRE(integrate_twist(tw3).translation().y==3);


}