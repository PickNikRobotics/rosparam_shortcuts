# ROS Param Shortcuts

Quickly load variables from rosparam with good command line error checking.

This package enforces the philosophy that there should be no default parameters - everything must be defined by the user in yaml files (or launch files or where ever) otherwise your program should not run. This helps debug why something isn't working correctly - it will tell you exactly what rosparameters are missing.

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

 * [![Build Status](https://travis-ci.org/davetcoleman/rosparam_shortcuts.svg)](https://travis-ci.org/davetcoleman/rosparam_shortcuts) Travis - Continuous Integration
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__rosparam_shortcuts__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__rosparam_shortcuts__ubuntu_trusty__source/) ROS Buildfarm - Trusty Devel Source Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__rosparam_shortcuts__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__rosparam_shortcuts__ubuntu_trusty_amd64__binary/) ROS Buildfarm - AMD64 Trusty Debian Build

## Install

### Ubuntu Debian

```
sudo apt-get install ros-kinetic-rosparam-shortcuts
```

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro kinetic

## Code API

See [Class Reference](http://docs.ros.org/kinetic/api/rosparams_shortcuts/html/)

## Usage / Demo

See the file ``src/example.cpp`` for example code. To run:

    roslaunch rosparam_shortcuts example.launch

Your yaml file would look something like the file ``config/example.yaml``:

```
example:
  control_rate: 100.0 # double
  param1: 20 # int
  param2: 30 # size_t
  param3: 1 # ros::Duration
  param4: [1, 1, 1, 3.14, 0, 0] # Eigen::Affine3d - x, y, z, roll, pitch, yaw
  param5: [1.1, 2.2, 3.3, 4.4] # std::vector<double>
```

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
