#include<catch_ros/catch.hpp>
#include "turtlelib/circle_detect.hpp"

using namespace turtlelib;
TEST_CASE("circle fitting test 1","[vectors]"){
    std::vector<std::vector<Vector2D>> xy_clusters;
    std::vector<Vector2D> cur_clusters;
    cur_clusters.push_back({1,7});
    cur_clusters.push_back({2,6});
    cur_clusters.push_back({5,8});
    cur_clusters.push_back({7,7});
    cur_clusters.push_back({9,5});
    cur_clusters.push_back({3,7});
    xy_clusters.push_back(cur_clusters);
    Circle circle = Circle(xy_clusters);
    circle.circle_fitting(0.1);
    REQUIRE( circle.centers[0].x==Approx(4.615482).margin(1e-4));
    REQUIRE( circle.centers[0].y==Approx(2.807354).margin(1e-4));
    REQUIRE( circle.radius[0]==Approx(4.8275).margin(1e-4));
}
TEST_CASE("circle fitting test 2","[vectors]"){
    std::vector<std::vector<Vector2D>> xy_clusters;
    std::vector<Vector2D> cur_clusters;
    cur_clusters.push_back({-1,0});
    cur_clusters.push_back({-0.3,-0.06});
    cur_clusters.push_back({0.3,0.1});
    cur_clusters.push_back({1,0});
    xy_clusters.push_back(cur_clusters);
    Circle circle = Circle(xy_clusters);
    circle.circle_fitting(0.1);
    REQUIRE( circle.centers[0].x==Approx(0.4908357).margin(1e-4));
    REQUIRE( circle.centers[0].y==Approx(-22.15212).margin(1e-4));
    REQUIRE( circle.radius[0]==Approx(22.17979).margin(1e-4));
}
TEST_CASE("circle classification test 1","[vectors]"){
    std::vector<std::vector<Vector2D>> xy_clusters;
    std::vector<Vector2D> cur_clusters;
    cur_clusters.push_back({0,2});
    cur_clusters.push_back({1.414,1.414});
    cur_clusters.push_back({2,0});
    cur_clusters.push_back({1.414,-1.414});
    cur_clusters.push_back({0,-2});
    xy_clusters.push_back(cur_clusters);
    Circle circle = Circle(xy_clusters);
    circle.classification(0.15,90,140);
    REQUIRE( circle.xy_clusters.size()==1);
}
TEST_CASE("circle classification test 2","not circle"){
    std::vector<std::vector<Vector2D>> xy_clusters;
    std::vector<Vector2D> cur_clusters;
    cur_clusters.push_back({0,2});
    cur_clusters.push_back({1.414,1.414});
    cur_clusters.push_back({2,0});
    cur_clusters.push_back({1.414,-1.414});
    cur_clusters.push_back({0,-3});
    xy_clusters.push_back(cur_clusters);
    Circle circle = Circle(xy_clusters);
    circle.classification(0.15,90,140);
    REQUIRE( circle.xy_clusters.size()==0);
}