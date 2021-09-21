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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Helpers for loading parameters from the parameter server
*/

// C++
#include <string>
#include <vector>

// this package
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Eigen/TF conversion
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rosparam_shortcuts");

namespace {
template <typename T>
void declare_parameter(const rclcpp::Node::SharedPtr& node, const std::string& param_name) {
	if (!node->has_parameter(param_name))
		node->declare_parameter<T>(param_name);
}

template <typename T>
bool get_parameter(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
	rclcpp::Parameter parameter;
	if (!node->get_parameter(param_name, parameter)) {
		RCLCPP_DEBUG_STREAM(LOGGER, "Missing parameter '" << node->get_name() << "." << param_name << "'.");
		return false;
	}
	try {
		value = parameter.get_value<T>();
	} catch (const rclcpp::exceptions::InvalidParameterTypeException& ex) {
		RCLCPP_ERROR_STREAM(LOGGER, ex.what());
		return false;
	}
	return true;
}

template <typename T>
bool get_internal(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
	declare_parameter<T>(node, param_name);
	return get_parameter(node, param_name, value);
}
}  // namespace

namespace rosparam_shortcuts {

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, bool& value) {
	// Load a param
	if (!get_internal(node, param_name, value))
		return false;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, double& value) {
	// Load a param
	if (!get_internal(node, param_name, value))
		return false;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, std::vector<double>& values) {
	// Load a param
	if (!get_internal(node, param_name, values))
		return false;
	if (values.empty())
		RCLCPP_WARN_STREAM(LOGGER, "Empty vector for parameter '" << node->get_name() << "." << param_name
		                                                          << "'"
		                                                             ".");

	RCLCPP_DEBUG_STREAM(LOGGER, "Loaded parameter '" << node->get_name() << "." << param_name << "' with values ["
	                                                 << getDebugArrayString(values) << "]");

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, int& value) {
	// Load a param
	if (!get_internal(node, param_name, value))
		return false;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, std::size_t& value) {
	// Load a param
	int64_t temp_value = 0;
	if (!get_internal(node, param_name, temp_value))
		return false;
	value = temp_value;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, std::string& value) {
	// Load a param
	if (!get_internal(node, param_name, value))
		return false;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, std::vector<std::string>& values) {
	// Load a param
	if (!get_internal(node, param_name, values))
		return false;

	if (values.empty())
		RCLCPP_WARN_STREAM(LOGGER, "Empty vector for parameter '" << node->get_name() << "." << param_name
		                                                          << "'"
		                                                             ".");

	RCLCPP_DEBUG_STREAM(LOGGER, "Loaded parameter '" << node->get_name() << "." << param_name << "' with value "
	                                                 << getDebugArrayString(values));

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, rclcpp::Duration& value) {
	double temp_value;
	// Load a param
	if (!get_internal(node, param_name, temp_value))
		return false;
	RCLCPP_DEBUG_STREAM(LOGGER,
	                    "Loaded parameter '" << node->get_name() << "." << param_name << "' with value " << temp_value);

	// Convert to ros::Duration
	value = rclcpp::Duration::from_seconds(temp_value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, Eigen::Isometry3d& value) {
	std::vector<double> values;
	// Load a param
	if (!get_internal(node, param_name, values))
		return false;

	if (values.empty())
		RCLCPP_WARN_STREAM(LOGGER, "Empty vector for parameter '" << node->get_name() << "." << param_name
		                                                          << "'"
		                                                             ".");

	RCLCPP_DEBUG_STREAM(LOGGER, "Loaded parameter '" << node->get_name() << "." << param_name << "' with values ["
	                                                 << getDebugArrayString(values) << "]");

	// Convert to Eigen::Isometry3d
	convertDoublesToEigen(values, value);

	return true;
}

bool get(const rclcpp::Node::SharedPtr& node, const std::string& param_name, geometry_msgs::msg::Pose& value) {
	Eigen::Isometry3d eigen_pose;
	if (!get(node, param_name, eigen_pose))
		return false;

	tf2::convert(eigen_pose, value);
	return true;
}

std::string getDebugArrayString(const std::vector<double>& values) {
	std::stringstream debug_values;
	for (const double value : values) {
		debug_values << value << ",";
	}
	return debug_values.str();
}

std::string getDebugArrayString(const std::vector<std::string>& values) {
	std::stringstream debug_values;
	for (const auto& value : values) {
		debug_values << value << ",";
	}
	return debug_values.str();
}

bool convertDoublesToEigen(const std::vector<double>& values, Eigen::Isometry3d& transform) {
	if (values.size() == 6) {
		// This version is correct RPY
		Eigen::AngleAxisd roll_angle(values[3], Eigen::Vector3d::UnitX());
		Eigen::AngleAxisd pitch_angle(values[4], Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd yaw_angle(values[5], Eigen::Vector3d::UnitZ());
		Eigen::Quaternion<double> quaternion = roll_angle * pitch_angle * yaw_angle;

		transform = Eigen::Translation3d(values[0], values[1], values[2]) * quaternion;

		return true;
	} else if (values.size() == 7) {
		// Quaternion
		transform = Eigen::Translation3d(values[0], values[1], values[2]) *
		            Eigen::Quaterniond(values[3], values[4], values[5], values[6]);
		return true;
	} else {
		RCLCPP_ERROR_STREAM(LOGGER, "Invalid number of doubles provided for transform, size=" << values.size());
		return false;
	}
}

void shutdownIfError(const std::size_t error_count) {
	if (!error_count)
		return;

	RCLCPP_ERROR_STREAM(LOGGER, "Missing " << error_count
	                                       << " ros parameters that are required. Shutting down "
	                                          "to prevent undefined behaviors");
	rclcpp::shutdown();
	exit(0);
}

}  // namespace rosparam_shortcuts
