/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example of how to use rosparam_shorcuts
*/

// ROS
#include <ros/ros.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

int main(int argc, char** argv)
{
  std::string name = "example";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ROS_INFO_STREAM_NAMED(name, "Starting rosparam shortcuts example...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  double control_rate;
  int param1;
  std::size_t param2;
  ros::Duration param3;
  Eigen::Affine3d param4;

  // Load rosparams
  ros::NodeHandle rpnh(nh, name);
  std::size_t error = 0;
  error += !rosparam_shortcuts::getDoubleParam(name, rpnh, "control_rate", control_rate);
  error += !rosparam_shortcuts::getIntParam(name, rpnh, "param1", param1);
  error += !rosparam_shortcuts::getSizeTParam(name, rpnh, "param2", param2);
  error += !rosparam_shortcuts::getDurationParam(name, rpnh, "param3", param3);
  error += !rosparam_shortcuts::getAffine3dParam(name, rpnh, "param4", param4);
  // add more parameters here to load if desired
  rosparam_shortcuts::shutdownIfParamErrors(name, error);

  // Output values that were read in
  ROS_INFO_STREAM_NAMED(name, "control rate: " << control_rate);
  ROS_INFO_STREAM_NAMED(name, "param1: " << param1);
  ROS_INFO_STREAM_NAMED(name, "param2: " << param2);
  ROS_INFO_STREAM_NAMED(name, "param3: " << param3.toSec());
  ROS_INFO_STREAM_NAMED(name, "param4: Translation:\n" << param4.translation());



  ROS_INFO_STREAM_NAMED(name, "Shutting down.");
  ros::shutdown();

  return 0;
}
