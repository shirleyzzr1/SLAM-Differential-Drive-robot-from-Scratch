#include<catch_ros/catch.hpp>
#include"turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <cmath>
using namespace turtlelib;
TEST_CASE("test_integrate_twist","3 differenct cases"){
    //pure translation
    Twist2D tw1= {0,1,0};
    Transform2D trans1 = integrate_twist(tw1);
    CHECK(trans1.rotation()==0);
    CHECK(trans1.translation().x ==1);
    CHECK(trans1.translation().y==0);

    //pure rotation
    Twist2D tw2 = {1,0,0};
    Transform2D trans2 = integrate_twist(tw2);
    CHECK(trans2.rotation()==1);
    CHECK(trans2.translation().x == 0);
    CHECK(trans2.translation().y==0);


    //translation & rotation
    Twist2D tw3 = {1,2,0};
    Transform2D trans3 = integrate_twist(tw3);
    CHECK(trans3.rotation()==1);
    CHECK(trans3.translation().x == Approx(1.68).epsilon(0.01));
    CHECK(trans3.translation().y== Approx(0.92).epsilon(0.01));

}
TEST_CASE("test dirve foward kinematics","dirve foward"){
    DiffDrive diff;
    diff.FK_calculate({5,5});
    CHECK(diff.body_pos().rotation()==Approx(0));
    CHECK(diff.body_pos().translation().x==Approx(0.165).epsilon(0.01));
    CHECK(diff.body_pos().translation().y==Approx(0));

    Twist2D tw = {0,0.22,0};
    diff.IK_calculate(tw);
    CHECK(diff.wheel_vel().x==Approx(6.66).epsilon(0.01));
    CHECK(diff.wheel_vel().y==Approx(6.66).epsilon(0.01));

}
TEST_CASE("test drive rotational kinematics","dirve rotational"){
    DiffDrive diff;
    diff.FK_calculate({5,-5});
    CHECK(diff.body_pos().rotation()==Approx(-2.06).epsilon(0.01));
    CHECK(diff.body_pos().translation().x==Approx(0));
    CHECK(diff.body_pos().translation().y==Approx(0));

    Twist2D tw = {2.84,0,0};
    diff.IK_calculate(tw);
    CHECK(diff.wheel_vel().x==Approx(-6.88).epsilon(0.01));
    CHECK(diff.wheel_vel().y==Approx(6.88).epsilon(0.01));

}
TEST_CASE("test drive circle kinematics","dirve circle"){
    DiffDrive diff;
    diff.FK_calculate({2,3});
    CHECK(diff.body_pos().rotation()==Approx(0.206).epsilon(0.01));
    CHECK(diff.body_pos().translation().x==Approx(0.081).margin(0.01));
    CHECK(diff.body_pos().translation().y==Approx(0.008).margin(0.01));

    DiffDrive diff1;
    Twist2D tw = {0.2,0.081,0};
    diff1.IK_calculate(tw);
    CHECK(diff.wheel_vel().x==Approx(2).epsilon(0.01));
    CHECK(diff.wheel_vel().y==Approx(3).epsilon(0.01));

}
TEST_CASE("test impossible twist","dirve impossible"){
    DiffDrive diff;
    Twist2D tw = {1,1,1};
    CHECK_THROWS(diff.IK_calculate(tw));
}
TEST_CASE("test_normalized_angle_function","test vector"){
    CHECK(normalize_angle(PI)==Approx(PI));
    CHECK(normalize_angle(-PI)==Approx(-PI));
    CHECK(normalize_angle(0)==Approx(0));
    CHECK(normalize_angle(-PI/4)==Approx(-PI/4));
    CHECK(normalize_angle(3*PI/2)==Approx(PI/2));
    CHECK(normalize_angle(-5*PI/2)==Approx(-PI/2));

}
TEST_CASE("test_twist","integrate_twist_function"){
    Twist2D tw1 = {0,2,3};
    Twist2D tw2 = {1,0,0};
    Twist2D tw3 = {1,2,3};

    //pure translation
    CHECK(integrate_twist(tw1).rotation()==0);
    CHECK(integrate_twist(tw1).translation().x==2);
    CHECK(integrate_twist(tw1).translation().y==3);

    //pure rotation
    CHECK(integrate_twist(tw2).rotation()==1);
    CHECK(integrate_twist(tw2).translation().x==0);
    CHECK(integrate_twist(tw2).translation().y==0);

    //rotation and translation
    CHECK(integrate_twist(tw3).rotation()==1);
    CHECK(integrate_twist(tw3).translation().x==Approx(0.303).epsilon(0.01));
    CHECK(integrate_twist(tw3).translation().y==Approx(3.44).epsilon(0.01));


}