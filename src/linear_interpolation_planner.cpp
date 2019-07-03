#include <linear_interpolation_planner/linear_interpolation_planner.hpp>

#include <geometry_msgs/Pose.h>

linear_interpolation_planner::linear_interpolation_planner() :
    tf_listener_(tf_buffer_)
{
    nh_.param<int>(ros::this_node::getName()+"/resolution", resolution_, 20);

    wp_sub_ = nh_.subscribe(ros::this_node::getName()+"/waypoint",1,&linear_interpolation_planner::waypoint_cb, this);
    path_pub_ = nh_.advertise<usv_navigation_msgs::Path>(ros::this_node::getName()+"/path",1);
}

void linear_interpolation_planner::waypoint_cb(const usv_navigation_msgs::Waypoint::ConstPtr msg)
{
    geometry_msgs::TransformStamped transform;
    while(1){
        try{
            transform = tf_buffer_.lookupTransform(msg->header.frame_id, "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
        break;
    }

    auto path = calc_path(*msg, transform);
    path_pub_.publish(path);
}

usv_navigation_msgs::Path linear_interpolation_planner::calc_path(const usv_navigation_msgs::Waypoint& wp,
                                    const geometry_msgs::TransformStamped& cur_pos)
{
    usv_navigation_msgs::Path path;
    path.header = wp.header;
    path.header.stamp = ros::Time::now();
    
    geometry_msgs::Pose error;
    error.position.x = wp.pose.position.x - cur_pos.transform.translation.x;
    error.position.y = wp.pose.position.y - cur_pos.transform.translation.y;
    error.position.z = wp.pose.position.z - cur_pos.transform.translation.z;
    tf2::Quaternion rotation;
    rotation.setRPY(0,0,atan2(error.position.y,error.position.x));
    error.orientation.x = rotation.x();
    error.orientation.y = rotation.y();
    error.orientation.z = rotation.z();
    error.orientation.w = rotation.w();

    usv_navigation_msgs::Waypoint route_wp = wp;
    route_wp.header = path.header;
    route_wp.longitudinal_torelance /= static_cast<double>(resolution_);
    route_wp.lateral_torelance /= static_cast<double>(resolution_);
    route_wp.yaw_torelance /= static_cast<double>(resolution_);
    route_wp.pose.orientation = error.orientation;
    
    for(int i=0; i<resolution_; i++){
        route_wp.pose.position.x = cur_pos.transform.translation.x + error.position.x/resolution_ * static_cast<double>(i);
        route_wp.pose.position.y = cur_pos.transform.translation.y + error.position.y/resolution_ * static_cast<double>(i);
        route_wp.pose.position.z = cur_pos.transform.translation.z + error.position.z/resolution_ * static_cast<double>(i);
        path.waypoints.push_back(route_wp);
    }
    route_wp = wp;
    route_wp.header = path.header;
    path.waypoints.push_back(route_wp);

    return path;
}
