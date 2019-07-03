# linear_interpolation_planner

This package generates linear interpolated path.

## Subscribe
- [usv_navigation_msgs/Waypoint](https://github.com/OUXT-Polaris/usv_navigation_msgs) : `/linear_interpolation_planner/waypoint`  
  Goal waypoint.
- TF2 : `waypont frame_id -> "base_link"`  
  Current position & pose.  

## Publish
- [usv_navigation_msgs/Path](https://github.com/OUXT-Polaris/usv_navigation_msgs) : `/linear_interpolation_planner/path`  
  Generated path.  
  This is mostly used as global path.

## Parameter
- resolution : `int, default : 20`  
  The resolution that divides from start to goal.
