#include <linear_interpolation_planner/linear_interpolation_planner.hpp>

#include <geometry_msgs/Pose.h>

linear_interpolation_planner::linear_interpolation_planner() :
    tf_listener_(tf_buffer_)
{
    //nh_.param<int>(ros::this_node::getName()+"/resolution", resolution_, 20);
    nh_.param<double>(ros::this_node::getName()+"/split_length", split_length_, 1.0);
    nh_.param<std::string>(ros::this_node::getName()+"/input_topic", input_topic_, ros::this_node::getName()+"/waypoint");
    wp_sub_ = nh_.subscribe(input_topic_,1,&linear_interpolation_planner::waypoint_cb, this);
    path_pub_ = nh_.advertise<usv_navigation_msgs::Path>(ros::this_node::getName()+"/path",1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/path/marker",1);
}

visualization_msgs::MarkerArray linear_interpolation_planner::generateMarker(usv_navigation_msgs::Path path)
{
    visualization_msgs::MarkerArray marker_msg;
    int id = 0;
    for(auto waypoint_itr = path.waypoints.begin(); waypoint_itr != path.waypoints.end(); waypoint_itr++)
    {
        visualization_msgs::Marker pose_marker;
        pose_marker.header = waypoint_itr->header;
        pose_marker.ns = "pose_marker";
        pose_marker.id = id;
        pose_marker.type = visualization_msgs::Marker::ARROW;
        pose_marker.action = visualization_msgs::Marker::ADD;
        pose_marker.pose = waypoint_itr->pose;
        pose_marker.frame_locked = true;
        pose_marker.color.r = 1.0;
        pose_marker.color.g = 0.0;
        pose_marker.color.b = 0.0;
        pose_marker.color.a = 1.0;
        pose_marker.scale.x = 0.5;
        pose_marker.scale.y = 0.1;
        pose_marker.scale.z = 0.1;
        marker_msg.markers.push_back(pose_marker);
        visualization_msgs::Marker text_marker;
        text_marker.header = waypoint_itr->header;
        text_marker.ns = "text_marker";
        text_marker.id = id;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose = waypoint_itr->pose;
        text_marker.pose.position.z = text_marker.pose.position.z + 0.3;
        text_marker.frame_locked = true;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.scale.x = 0.1;
        text_marker.scale.y = 0.1;
        text_marker.scale.z = 0.1;
        text_marker.text = std::to_string(id);
        marker_msg.markers.push_back(text_marker);
        id = id + 1;
    }
    return marker_msg;
}

void linear_interpolation_planner::waypoint_cb(const usv_navigation_msgs::Waypoint::ConstPtr msg)
{
    geometry_msgs::TransformStamped transform;
    while(1)
    {
        try
        {
            transform = tf_buffer_.lookupTransform(msg->header.frame_id, "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(0.5).sleep();
            continue;
        }
        break;
    }

    auto path = calc_path(*msg, transform);
    path_pub_.publish(path);
    marker_pub_.publish(generateMarker(path));
}

usv_navigation_msgs::Path linear_interpolation_planner::calc_path(const usv_navigation_msgs::Waypoint& wp,const geometry_msgs::TransformStamped& cur_pos)
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

    double path_length = std::sqrt(std::pow(error.position.x,2) + std::pow(error.position.y,2));
    resolution_ = std::floor(path_length/split_length_);//(int)fmod(path_length,split_length_);
    if(resolution_<1)
    {
        resolution_ = 1;
    }

    usv_navigation_msgs::Waypoint route_wp = wp;
    route_wp.header = path.header;
    route_wp.longitudinal_torelance /= static_cast<double>(resolution_);
    route_wp.lateral_torelance /= static_cast<double>(resolution_);
    route_wp.yaw_torelance /= static_cast<double>(resolution_);
    route_wp.pose.orientation = error.orientation;
    
    for(int i=0; i<resolution_; i++)
    {
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
