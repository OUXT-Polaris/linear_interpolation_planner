// headers for ros
#include <ros/ros.h>

//headers in this package
#include <linear_interpolation_planner/linear_interpolation_planner.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "linear_interpolation_planner_node");
    linear_interpolation_planner lip;
    ros::spin();
    return 0;
}
