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
   Desc: handles node parameters dynamic and static
*/

#include <rosparam_shortcuts/node_parameters.h>

namespace rosparam_shortcuts {

rcl_interfaces::msg::SetParametersResult static_parameter_validation(const rclcpp::Parameter& /*unused*/) {
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = false;
	result.reason = "Parameter is not Dynamically Configurable.";
	return result;
}

rcl_interfaces::msg::SetParametersResult always_accept(const rclcpp::Parameter& /*unused*/) {
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	return result;
}

NodeParameters::NodeParameters(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger)
  : node_(node), logger_(logger) {}

void NodeParameters::registerRosCallbackReconfigure() {
	on_set_callback_handle_ = node_->add_on_set_parameters_callback(
	    [this](const std::vector<rclcpp::Parameter>& parameters) { return setParametersCallback(parameters); });
}

void NodeParameters::registerUpdateCallback(const std::function<void()>& callback) const {
	std::lock_guard<std::mutex> guard(mutex_);
	changed_notify_callback_.push_back(callback);
}

bool NodeParameters::get(const std::string& name, rclcpp::Parameter& parameter) const {
	std::lock_guard<std::mutex> guard(mutex_);
	auto internal_param = parameters_.find(name);
	if (internal_param != parameters_.end()) {
		parameter = parameters_.at(name);
		return true;
	}

	return false;
}

rclcpp::Parameter NodeParameters::get(const std::string& name) const {
	std::lock_guard<std::mutex> guard(mutex_);
	auto internal_param = parameters_.find(name);
	if (internal_param != parameters_.end()) {
		return parameters_.at(name);
	} else {
		RCLCPP_ERROR_STREAM(logger_, "Parameter: " << name + " not found.");
	}

	return rclcpp::Parameter();
}

rcl_interfaces::msg::SetParametersResult
NodeParameters::setParametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
	const std::lock_guard<std::mutex> guard(mutex_);
	auto result = rcl_interfaces::msg::SetParametersResult();
	result.successful = true;

	// First validate
	for (const auto& parameter : parameters) {
		const auto internal_param = parameters_.find(parameter.get_name());
		const auto search = validate_.find(parameter.get_name());

		// Only checks if parameter was declared
		if (search != validate_.end()) {
			// will fail if parameter is not in internalized
			if (parameter.get_type_name() != internal_param->second.get_type_name()) {
				result.successful = false;
				result.reason = "Parameter: " + parameter.get_name() + " type does not match internal type.";
				return result;
			}

			// Parameter callback
			result = search->second(parameter);
			if (!result.successful) {
				// Handle automatic parameter set failure
				return result;
			}
		}

		else {
			result.successful = false;
			result.reason = "Parameter: " + parameter.get_name() + " was not declared.";
			return result;
		}
	}

	// Set parameters
	for (const auto& parameter : parameters) {
		RCLCPP_DEBUG_STREAM(logger_, "setParametersCallback - "
		                                 << parameter.get_name() << "<" << parameter.get_type_name()
		                                 << ">: " << rclcpp::to_string(parameter.get_parameter_value()));

		parameters_.at(parameter.get_name()) = parameter;
	}

	// Lastly notify
	for (const auto& changed_notify_callback : changed_notify_callback_) {
		changed_notify_callback();
	}

	return result;
}

}  // namespace rosparam_shortcuts
