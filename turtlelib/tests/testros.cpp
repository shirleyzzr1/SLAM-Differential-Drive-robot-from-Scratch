#include<catch_ros/catch.hpp>
#include"turtlelib/rigid2d.hpp"
#include <cmath>
using namespace turtlelib;
TEST_CASE("test_normalized_angle_function","test vector"){
    REQUIRE(normalize_angle(PI)==Approx(PI));
    REQUIRE(normalize_angle(-PI)==Approx(0));
    REQUIRE(normalize_angle(0)==Approx(0));
    REQUIRE(normalize_angle(-PI/4)==Approx(-PI/4));
    REQUIRE(normalize_angle(3*PI/2)==Approx(PI/2));
    REQUIRE(normalize_angle(-5*PI/2)==Approx(-PI/2));

}