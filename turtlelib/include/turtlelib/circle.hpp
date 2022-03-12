#ifndef CIRCLE_INCLUDE_GUARD_HPP
#define CIRCLE_INCLUDE_GUARD_HPP
#include <vector>
/// \file
/// \brief Circle detection algorithm
namespace turtlelib{
    /// \brief a class function for the circle detection
    class Circle{
    private: 
        std::vector<std::vector<double>> clusters;
        std::vector<std::vector<Vector2D>> xy_clusters;
    public:
        /// \brief clustering points into different groups
        /// \param the lidar ranges in 360 degree
        void range_cluster(std::vector<double>range,double thresh);

        /// \brief fit the clustered arc to generate a cirlce
        void circle_fitting();

        /// \brief classify the clusters
        void classification();

    };
}
#endif
