#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "turtlelib/rigid2d.hpp"
using namespace turtlelib;

/// \brief test all the vectors related function
TEST_CASE("vector test","[vectors]"){
    std::ostringstream out;
    Vector2D v1 = {1,2};
    out << v1;
    REQUIRE( "[1 2]\n"==out.str() );

    std::istringstream in("1 2");
    Vector2D v2;
    in >> v2;
    REQUIRE( v2.x==Approx(1));
    REQUIRE( v2.y==Approx(2));

    Vector2D v3;
    v3 = Vector2Dnormalize(v2);
    REQUIRE( v3.x==Approx(0.44).margin(1e-1));
    REQUIRE( v3.y==Approx(0.89).margin(1e-1));
}

/// \brief test all the transformation related function
TEST_CASE("transform test","[transform2D]"){
    std::ostringstream out1;
    Transform2D tf1 ;
    out1 << tf1;
    REQUIRE( "deg: 0 x: 1 y: 1\n"==out1.str() );

    Vector2D v2 = {2,1};
    std::ostringstream out2;

    Transform2D tf2(v2);
    out2 << tf2;
    REQUIRE( "deg: 0 x: 2 y: 1\n"==out2.str() );

    double radius = 2.2;
    Transform2D tf3(radius);
    REQUIRE( tf3.rotation()==Approx(2.2));

    std::istringstream in("90 0 1");
    Transform2D tf4;
    in >> tf4;
    REQUIRE( tf4.rotation()==Approx(1.57).margin(1e-1));
    REQUIRE( tf4.translation().x==0);

    Transform2D tf5;
    tf5 = tf4.inv();
    REQUIRE(tf5.rotation()==Approx(-1.57).margin(1e-1));
    REQUIRE(tf5.translation().x ==-1);
    REQUIRE(tf5.translation().y == Approx(0).margin(1e-3) );

    Transform2D tf6;
    tf6 = tf4*tf5;
    std::ostringstream out3;
    out3 << tf6;
    REQUIRE( "deg: 0 x: 0 y: 0\n"==out3.str() );

    tf4*=tf5;
    std::ostringstream out4;
    out4 << tf4;
    REQUIRE( "deg: 0 x: 0 y: 0\n"==out4.str() );
    
}

///\brief test all the transformation on matrix and vector
TEST_CASE("transform and vectors","[]"){
    Vector2D v1 = {1,3};
    std::istringstream in("0 1 1");
    Transform2D tf1;
    in >> tf1;
    Vector2D v2 = tf1(v1);

    REQUIRE(v2.x==2);
    REQUIRE(v2.y == 4);

}
