# ros_param_shortcuts
Quickly load variables from rosparam with proper error checking

## Example Usage

In your C++ file's header:

    // ROS parameter loading
    #include <ros_param_shortcuts/ros_param_shortcuts.h>

And in your class constructor (or other location):

    // Load rosparams
    {
      const std::string parent_name = "my_controller";  // for namespacing logging messages
      ros::NodeHandle rosparam_nh(nh_, parent_name);
      using namespace ros_param_shortcuts;
      std::size_t error = 0;
      error += !getDoubleParam(parent_name, rosparam_nh, "control_rate", control_rate_);
	  // ...
	  shutdownIfParamErrors(parent_name, error);
    }
