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

#ifndef NODE_PARAMETERS_H
#define NODE_PARAMETERS_H

// C++
#include <functional>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace rosparam_shortcuts {

/**
 * @brief Parameter is not dynamic
 * @return results is always false
 */
rcl_interfaces::msg::SetParametersResult static_parameter_validation(const rclcpp::Parameter& /*unused*/);

/**
 * @brief Parameter is dynamic
 * @return results is always true
 */
rcl_interfaces::msg::SetParametersResult always_accept(const rclcpp::Parameter& /*unused*/);

/** @brief Parameters for a node */
class NodeParameters
{
	/** @brief Validate parameter */
	typedef std::function<rcl_interfaces::msg::SetParametersResult(const rclcpp::Parameter&)> ParamCallback;

public:
	/** @brief Constructor */
	NodeParameters(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger);

	/**
	 * @brief Declare and get a parameter
	 * @param name - parameter name
	 * @param default_value - initial value to be used if at run-time user did not override it
	 * @param parameter_descriptor - custom description for the parameter
	 * @param callback - parameter validation callback
	 * @details will not delcare a parameter twice
	 */
	template <typename ParameterT>
	void declareAndGet(const std::string& name, const ParameterT& default_value,
	                   const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
	                   const ParamCallback& callback = static_parameter_validation);

	/**
	 * @brief Declare and get a parameter
	 * @param name - parameter name
	 * @param default_value - initial value to be used if at run-time user did not override it
	 * @param callback - parameter validation callback
	 * @details will not delcare a parameter twice
	 */
	template <typename ParameterT>
	void declareAndGet(const std::string& name, const ParameterT& default_value,
	                   const ParamCallback& callback = static_parameter_validation);

	/**
	 * @brief Declare and get a parameter
	 * @param name - parameter name
	 * @param callback - parameter validation callback
	 * @details will not delcare a parameter twice
	 */
	template <typename ParameterT>
	void declareAndGet(const std::string& name, const ParamCallback& callback = static_parameter_validation);

	/**
	 * @brief Registers ros2 parameter callback function
	 * @details **IMPORTANT**: do NOT call declareAndGet() after this
	 */
	void registerRosCallbackReconfigure();

	/**
	 * @brief Register user defined parameter callback
	 * @param callback - user defined callback to indicate a parameter has changed
	 * @details allows user to register multiple callbacks
	 */
	void registerUpdateCallback(const std::function<void()>& callback) const;

	/**
	 * @brief Retrieve parameter
	 * @param name - parameter name
	 * @param parameter[out] - parameter
	 * @return true if the parameter is retrieved
	 */
	bool get(const std::string& name, rclcpp::Parameter& parameter) const;

	/**
	 * @brief Retrieve parameter
	 * @param name - parameter name
	 * @return if not found returns rclcpp::ParameterValue()
	 */
	rclcpp::Parameter get(const std::string& name) const;

private:
	/**
	 * @brief ros2 parameter validation/update callback
	 * @param parameters - vector of declared parameters
	 * @return result is set to false if parameters are not validated
	 * @details All the parameters all validated, then updated, and finally the user defined callback is called
	 */
	rcl_interfaces::msg::SetParametersResult setParametersCallback(const std::vector<rclcpp::Parameter>& parameters);

private:
	const rclcpp::Node::SharedPtr node_;
	const rclcpp::Logger logger_;

	std::map<std::string, ParamCallback> validate_;  // map parameter name to validation callback
	std::map<std::string, rclcpp::Parameter> parameters_;  // map parameter name to ros2 parameter

	mutable std::mutex mutex_;  // protect parameters
	mutable std::vector<std::function<void()>> changed_notify_callback_;  // enable user to have callbacks for changed
	                                                                      // parameters

	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;  // ros2 callback method
	                                                                                            // for updating
	                                                                                            // parameters
};

template <typename ParameterT>
void NodeParameters::declareAndGet(const std::string& name, const ParameterT& default_value,
                                   const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor,
                                   const ParamCallback& callback) {
	std::lock_guard<std::mutex> guard(mutex_);
	// set our internal state for validate
	validate_.emplace(name, callback);

	// declare and get through ros
	if (!node_->has_parameter(name)) {
		node_->declare_parameter(name, default_value, parameter_descriptor);
	}

	// set our internal state of the parameter
	parameters_.emplace(name, node_->get_parameter(name));
}

template <typename ParameterT>
void NodeParameters::declareAndGet(const std::string& name, const ParameterT& default_value,
                                   const ParamCallback& callback) {
	std::lock_guard<std::mutex> guard(mutex_);
	// set our internal state for validate
	validate_.emplace(name, callback);

	// declare and get through ros
	if (!node_->has_parameter(name)) {
		node_->declare_parameter(name, default_value);
	}

	// set our internal state of the parameter
	parameters_.emplace(name, node_->get_parameter(name));
}

template <typename ParameterT>
void NodeParameters::declareAndGet(const std::string& name, const ParamCallback& callback) {
	std::lock_guard<std::mutex> guard(mutex_);
	// set our internal state for validate
	validate_.emplace(name, callback);

	// declare and get through ros
	if (!node_->has_parameter(name)) {
		node_->declare_parameter<ParameterT>(name);
	}

	// set our internal state of the parameter
	parameters_.emplace(name, node_->get_parameter(name));
}

}  // namespace rosparam_shortcuts
#endif  // NODE_PARAMETERS_H
