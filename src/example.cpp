// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Dave Coleman
   Desc:   Example of how to use rosparam_shorcuts
*/

// C++
#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

std::ostream& operator<<(std::ostream& out, const geometry_msgs::msg::Pose& pose) {
	out << "position: x=" << pose.position.x << ", y=" << pose.position.y << ", z= " << pose.position.z
	    << " - orientation: x= " << pose.orientation.x << ", y= " << pose.orientation.y << ", z= " << pose.orientation.z
	    << ", w= " << pose.orientation.w;
	return out;
}
int main(int argc, char** argv) {
	std::string name = "example";
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("example");
	RCLCPP_INFO(node->get_logger(), "Starting rosparam shortcuts example...");

	// Allow the action server to recieve and send ros messages
	double control_rate;
	int param1;
	std::size_t param2;
	rclcpp::Duration param3 = rclcpp::Duration::from_nanoseconds(0.0);
	Eigen::Isometry3d param4;
	std::vector<double> param5;
	Eigen::Isometry3d param6;
	geometry_msgs::msg::Pose param7;
	geometry_msgs::msg::Pose param8;

	// Load rosparams
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(node, "control_rate", control_rate);  // Double param
	error += !rosparam_shortcuts::get(node, "param1", param1);  // Int param
	error += !rosparam_shortcuts::get(node, "param2", param2);  // SizeT pa ram
	error += !rosparam_shortcuts::get(node, "param3", param3);  // Duration param
	error += !rosparam_shortcuts::get(node, "param4", param4);  // Isometry3d param
	error += !rosparam_shortcuts::get(node, "param5", param5);  // std::vector<double>
	error += !rosparam_shortcuts::get(node, "param6", param6);  // Isometry3d param
	error += !rosparam_shortcuts::get(node, "param7", param7);  // geometry_msgs::Pose param
	error += !rosparam_shortcuts::get(node, "param8", param8);  // geometry_msgs::Pose param
	// add more parameters here to load if desired
	rosparam_shortcuts::shutdownIfError(error);

	// Output values that were read in
	RCLCPP_INFO_STREAM(node->get_logger(), "control rate: " << control_rate);
	RCLCPP_INFO_STREAM(node->get_logger(), "param1: " << param1);
	RCLCPP_INFO_STREAM(node->get_logger(), "param2: " << param2);
	RCLCPP_INFO_STREAM(node->get_logger(), "param3: " << param3.to_chrono<std::chrono::duration<double>>().count());
	RCLCPP_INFO_STREAM(node->get_logger(), "param4: Translation:\n" << param4.translation());
	RCLCPP_INFO_STREAM(node->get_logger(), "param5[0]: " << param5[0]);
	RCLCPP_INFO_STREAM(node->get_logger(), "param5[3]: " << param5[3]);
	RCLCPP_INFO_STREAM(node->get_logger(), "param6: Translation:\n" << param6.translation());
	RCLCPP_INFO_STREAM(node->get_logger(), "param7: Pose:\n" << param7);
	RCLCPP_INFO_STREAM(node->get_logger(), "param8: Pose:\n" << param8);
	RCLCPP_INFO_STREAM(node->get_logger(), "Shutting down.");
	rclcpp::shutdown();

	return 0;
}
