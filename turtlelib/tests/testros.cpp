#include<catch_ros/catch.hpp>
#include"turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
using namespace turtlelib;
TEST_CASE("test_integrate_twist","3 differenct cases"){
    //pure translation
    Twist2D tw1= {0,1,0};
    Transform2D trans1 = integrate_twist(tw1);
    REQUIRE(trans1.rotation()==0);
    REQUIRE(trans1.translation().x ==1);
    REQUIRE(trans1.translation().y==0);

    //pure rotation
    Twist2D tw2 = {1,0,0};
    Transform2D trans2 = integrate_twist(tw2);
    REQUIRE(trans2.rotation()==1);
    REQUIRE(trans2.translation().x == 0);
    REQUIRE(trans2.translation().y==0);


    //translation & rotation
    Twist2D tw3 = {1,2,0};
    Transform2D trans3 = integrate_twist(tw3);
    REQUIRE(trans3.rotation()==1);
    REQUIRE(trans3.translation().x == Approx(1.68).epsilon(0.01));
    REQUIRE(trans3.translation().y== Approx(0.92).epsilon(0.01));

}
TEST_CASE("test dirve foward kinematics","dirve foward"){
    DiffDrive diff;
    diff.FK_calculate({5,5});
    // REQUIRE(diff.body_pos().rotation()==Approx(0));
    // REQUIRE(diff.body_pos().translation().x==Approx(0.165).epsilon(0.01));
    // REQUIRE(diff.body_pos().translation().y==Approx(0));

    Twist2D tw = {0,0.22,0};
    diff.IK_calculate(tw);
    REQUIRE(diff.wheel_vel().x==Approx(30.30).epsilon(0.01));
    REQUIRE(diff.wheel_vel().y==Approx(30.30).epsilon(0.01));

}
TEST_CASE("test drive rotational kinematics","dirve rotational"){
    DiffDrive diff;
    // diff.FK_calculate({5,-5});
    // REQUIRE(diff.body_pos().rotation()==Approx(-9.16).epsilon(0.01));
    // REQUIRE(diff.body_pos().translation().x==Approx(0));
    // REQUIRE(diff.body_pos().translation().y==Approx(0));

    Twist2D tw = {2.84,0,0};
    diff.IK_calculate(tw);
    REQUIRE(diff.wheel_vel().x==Approx(-0.545).epsilon(0.01));
    REQUIRE(diff.wheel_vel().y==Approx(0.545).epsilon(0.01));

}
TEST_CASE("test drive circle kinematics","dirve circle"){
    DiffDrive diff;
    diff.FK_calculate({29.757,30.848});
    REQUIRE(diff.body_pos().rotation()==Approx(1).epsilon(0.01));
    REQUIRE(diff.body_pos().translation().x==Approx(0.84).epsilon(0.01));
    REQUIRE(diff.body_pos().translation().y==Approx(0.460).epsilon(0.01));

    Twist2D tw = {1,1,0};
    diff.IK_calculate(tw);
    REQUIRE(diff.wheel_vel().x==Approx(29.757).epsilon(0.01));
    REQUIRE(diff.wheel_vel().y==Approx(30.848).epsilon(0.01));

}
TEST_CASE("test impossible twist","dirve impossible"){
    DiffDrive diff;
    Twist2D tw = {1,1,1};
    diff.IK_calculate(tw);
    // REQUIRE(diff.wheel_vel().x==Approx(29.757).epsilon(0.01));
    // REQUIRE(diff.wheel_vel().y==Approx(30.848).epsilon(0.01));

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

    //pure translation
    REQUIRE(integrate_twist(tw1).rotation()==0);
    REQUIRE(integrate_twist(tw1).translation().x==2);
    REQUIRE(integrate_twist(tw1).translation().y==3);

    //pure rotation
    REQUIRE(integrate_twist(tw2).rotation()==1);
    REQUIRE(integrate_twist(tw2).translation().x==0);
    REQUIRE(integrate_twist(tw2).translation().y==0);

    //rotation and translation
    REQUIRE(integrate_twist(tw3).rotation()==1);
    REQUIRE(integrate_twist(tw3).translation().x==Approx(0.303).epsilon(0.01));
    REQUIRE(integrate_twist(tw3).translation().y==Approx(3.44).epsilon(0.01));


}