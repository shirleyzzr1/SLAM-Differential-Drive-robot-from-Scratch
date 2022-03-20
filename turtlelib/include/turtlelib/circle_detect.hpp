#ifndef CIRCLE_DETECT_INCLUDE_GUARD_HPP
#define CIRCLE_DETECT_INCLUDE_GUARD_HPP
#include <vector>
#include "turtlelib/rigid2d.hpp"

/// \file
/// \brief Circle detection algorithm
namespace turtlelib{
    /// \brief a class function for the circle detection
    class Circle{
    public: 
        std::vector<std::vector<float>> clusters;
        std::vector<std::vector<Vector2D>> xy_clusters;
        std::vector<Vector2D> centers;
        std::vector<float> radius;
    public:
        Circle();
        /// \brief the constructor for the circle fitting
        Circle(std::vector<std::vector<Vector2D>> xy_clusters);
        /// \brief clustering points into different groups
        /// \param the lidar ranges in 360 degree
        void range_cluster(std::vector<float>range,float thresh);

        /// \brief fit the clustered arc to generate a cirlce
        void circle_fitting(double radius_range);

        /// \brief classify the clusters
        void classification(double std_angle,double min_mean_angle,double max_mean_angle);

        /// \brief clear the detected circles
        void clear();

    };
}
#endif
