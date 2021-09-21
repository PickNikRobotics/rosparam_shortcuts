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

/* Author: Boston Cleek
   Desc:   Example of how to use node parameters (dynamic and static)
*/

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

using rosparam_shortcuts::always_accept;
using rosparam_shortcuts::NodeParameters;
using rosparam_shortcuts::static_parameter_validation;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node_parameters");
static bool CHANGE = false;

void parameters_updated() {
	CHANGE = true;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("node_parameters");

	NodeParameters params(node, LOGGER);
	params.declareAndGet("length", 1.0);  // defaults to static parameter;
	params.declareAndGet("damping", 0.5, always_accept);  // any value is valid

	// Validate parameter
	params.declareAndGet("mass", 1.0, [](const rclcpp::Parameter& p) -> rcl_interfaces::msg::SetParametersResult {
		rcl_interfaces::msg::SetParametersResult result;
		if (p.as_double() > 0.0 && p.as_double() < 10.0) {
			result.successful = true;
		} else {
			result.successful = false;
			result.reason = "Parameter (mass) is out of valid range.";
		}

		return result;
	});

	// do NOT call declareAndGet() after this
	params.registerRosCallbackReconfigure();

	// tells node the parameters have changed
	params.registerUpdateCallback(parameters_updated);

	// Try changing parameters at the command line
	// ex: ros2 param set /dynamic_parameters damping 100.0
	while (rclcpp::ok()) {
		if (CHANGE) {
			rclcpp::Parameter p;
			if (params.get("damping", p)) {
				RCLCPP_INFO(LOGGER, "Parameter: %s -> value: %f", p.get_name().c_str(), p.as_double());
			}

			if (params.get("mass", p)) {
				RCLCPP_INFO(LOGGER, "Parameter: %s -> value: %f", p.get_name().c_str(), p.as_double());
			}

			CHANGE = false;
		}

		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();

	return 0;
}
