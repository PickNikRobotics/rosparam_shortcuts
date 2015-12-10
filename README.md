# ROS Param Shortcuts

Quickly load variables from rosparam with good command line error checking.

This package enforces the philosphy that there should be no default parameters - everything must be defined by the user in yaml files (or launch files or where ever) otherwise your program should not run. This helps debug why something isn't working correctly - it will tell you exactly what rosparameters are missing.

Features:
 - Outputs all loaded data into consule using ROS_DEBUG, so you won't see it unless you turn it on
 - Namespaces all output within the ``parent_name``
 - Great for having each class have its own parameter namespace
 - Helpful error messages if parameter is missing, explaining where it expects to find it
 - Removes lots of repetitious code
 - Supports datatypes that rosparam does not by default, such as std::size_t, ros::Duration, Eigen::Affine3d
 - Supports loading std::vectors easily, and debugging that data
 - Supports loading an entire list of bool parameters

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/rosparam_shortcuts.svg)](https://travis-ci.org/davetcoleman/rosparam_shortcuts) Travis CI
 * [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-rosparam_shortcuts)](http://jenkins.ros.org/job/devel-indigo-rosparam_shortcuts/) Devel Job Status
 * [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=ros-indigo-rosparam-shortcuts_binarydeb_trusty_amd64)](http://jenkins.ros.org/job/ros-indigo-rosparam-shortcuts_binarydeb_trusty_amd64/) AMD64 Debian Job Status

## Install

### Ubuntu Debian

```
sudo apt-get install ros-indigo-rosparam-shortcuts
```

## Code API

See [Class Reference](http://docs.ros.org/indigo/api/rosparams_shortcuts/html/)

## Example Usage / Demo

See the file ``src/rosparam_shortcuts_example.cpp`` for example code. To run:

    roslaunch rosparam_shortcuts example.launch

Your yaml file would look something like the file ``config/example.yaml``:

    example:
	  control_rate: 100.0
	  param1: 20
	  param2: 30
	  param3: 1
	  param4: [1, 1, 1, 3.14, 0, 0] # x, y, z, roll, pitch, yaw

## Contribute

Please send PRs for new helper functions, fixes, etc!
