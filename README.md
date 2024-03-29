# linear_interpolation_planner

![Developed By OUXT Polaris](img/logo.png "Logo")

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/linear_interpolation_planner.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/linear_interpolation_planner)|[![Build Status](https://travis-ci.org/OUXT-Polaris/linear_interpolation_planner.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/linear_interpolation_planner)|

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
- split_length : `double, default : 1`  
  The resolution of the path.
- input_topic: `string, default : ~/waypoint`
  The name of the input topic.
