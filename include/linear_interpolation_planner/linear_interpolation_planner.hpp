#ifndef __LINEAR_INTERPOLATION_PLANNER__
#define __LINEAR_INTERPOLATION_PLANNER__

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <usv_navigation_msgs/Waypoint.h>
#include <usv_navigation_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

class linear_interpolation_planner
{
public:
    linear_interpolation_planner();
    ~linear_interpolation_planner() = default;
    void waypoint_cb(const usv_navigation_msgs::Waypoint::ConstPtr msg);
private:
    usv_navigation_msgs::Path calc_path(const usv_navigation_msgs::Waypoint& wp,const geometry_msgs::TransformStamped& cur_pos);
    visualization_msgs::MarkerArray generateMarker(usv_navigation_msgs::Path path);
    ros::NodeHandle nh_;
    ros::Subscriber wp_sub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    int resolution_;
    std::string input_topic_;
    std::string robot_frame_;
    double split_length_;
};

#endif
