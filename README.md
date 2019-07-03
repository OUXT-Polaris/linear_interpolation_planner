# linear_interpolation_planner

This package generates linear interpolated path.

## Subscrib
#### [usv_navigation_msgs/Waypoint](https://github.com/OUXT-Polaris/usv_navigation_msgs)
Goal waypoint.

#### TF2
Current position & pose.  
This package subscribes from Waypoint frame\_id to "base_link".

## Publish
#### [usv_navigation_msgs/Path](https://github.com/OUXT-Polaris/usv_navigation_msgs)
Generated path.  
This is mostly used as global path.

## Parameter
#### resolution : int
The resolution that divides from start to goal.
