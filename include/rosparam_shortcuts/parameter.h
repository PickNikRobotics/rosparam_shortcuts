#pragma once

#include <mutex>
#include <shared_mutex>

#include <rclcpp>

namespace rosparam_shortcuts {

/*
 * Thread-safe container for generic parameter values, providing callback functions.
 * This class implements a tree structure of parameters which are declared recursively.
 * All leafs are of type Parameter<rclcpp::Parameter> which are used for interacting with
 * the rclcpp node parameter interface using the NodeConfig class further down.
 */
template <typename ParameterT>
class Parameter
{
public:
	Parameter(const std::string& name);
	Parameter(const std::string& name, const ParameterT& default_value);

	// Fields
	const std::string& name() const noexcept;
	std::string description() const noexcept;
	std::string additional_constraints() const noexcept;
	bool isNodeParameter() const noexcept;  // true if this parameter contains an rclcpp::Parameter

	// Properties
	Parameter<ParameterT>& describe(const std::string& description, const std::string& additional_constraints = "");
	// Allow setting additional properties. i.e. read_only, dynamic_typing?
	// void setProperties(...);

	// Validity Status
	const bool isInitialized() const noexcept;  // true when created with default
	const bool isValid() const noexcept;  // errors empty?
	const bool hasErrors() const noexcept;
	const bool hasWarnings() const noexcept;
	// TODO: Insert additional thread-safe access to errors/warnings

	// Read/write-locked data access using std::shared_mutex
	// See: https://www.youtube.com/watch?v=ozOgzlxIsdg
	ReadLockedPtr<ParameterT> operator->const;
	WriteLockedPtr<ParameterT> operator->;
	// optional: (useful for synchronized vector/map operations)
	void applyRW(std::function<void(ParameterT& value)>);
	void applyRO(std::function<void(const ParameterT& value)>);

	// Copy-set value using read/write lock internally
	ParameterT& operator=(const ParameterT& value);

	// Copy-get value, throws exception if isInitialized is false, uses read/write lock interally
	ParameterT operator() const;  // or:  'operator ParameterT() const;'

	// Validity Callbacks (could also use rcl_interfaces::msg::SetParametersResult like in node_parameters.h).
	// Callbacks are stored in ordered hash map, could also possibly return hash for deleting later on.
	// Alternatively, return Parameter<ParameterT>& for method chaining
	using ParameterValidityCallback = std::function<bool(const ParameterT& value, std::string& message)>;
	Parameter<ParameterT>& warnIf(ParameterValidityCallback warn_if_callback);
	Parameter<ParameterT>& errorIf(ParameterValidityCallback error_if_callback);  // or rejectIf()/failIf()..
	// Predefined generic (type-specific?) conditions, i.e. 'NO_OVERRIDE', 'EMPTY', 'NOT_NORMALIZED'
	enum ParameterCondition
	{
		...
	};
	Parameter<ParameterT>& warnIf(ParameterCondition condition);
	Parameter<ParameterT>& errorIf(ParameterCondition condition);

	// Callback for reacting to simple parameter changes (read-locked)
	// NOTE: this should be called after validity callbacks so that the parameter status can be evaluated
	using ParameterChangedCallback = std::function<void(const Parameter& parameter)>;
	void onChanged(ParameterChangedCallback callback);

protected:
	// Recursively declare parameters and populate child_params_, the actual node parameters (leafs) are
	// collected and returned so that they can be maintained by NodeConfig. Should call declareChildParam()
	// internally. NodeParameter is Parameter<rclcpp::Parameter> and needs to be forward declared..
	virtual std::map<std::string, std::shared_ptr<NodeParameter>> declare();

	// Constructor used for child parameters in the tree that share the same data, so basically all parameters
	// that are neither root nor leaf.
	// NOTE: not sure if this is really a good solution or if we should simply copy the data and synchronize
	// using child/parent callbacks only
	Parameter(const std::string& name, ParameterT& value, std::shared_mutex parent_mutex);

private:
	// Initialize child parameter and call declare() function recursively
	template <ChildParameterT>
	void declareChildParam(const std::string, std::map<std::string, std::shared_ptr<NodeParameter>>& node_parameters);

	// Internal value protected by mutex_. Alternatively use a struct ParameterValue that manages
	// locked acces and use that for internal data (this would allow sharing references between child
	// parameters)
	ParameterT value_;
	mutable std::shared_mutex mutex_;

	std::map<std::string, std::shared_ptr<Parameter>> child_params_;

	const std::string name_;
	std::string description_;
	std::string additional_constraints_;

	// Store callbacks, use alternative data structurs for ordered hashes
	std::map<std::string, ParameterValidityCallback> warn_if_callbacks_;
	std::map<std::string, ParameterValidityCallback> error_if_callbacks_;
	std::map<std::string, ParameterChangedCallback> on_changed_callbacks_;

	// Store messages of failed validity callbacks
	std::vector<std::string> errors_;
	std::vector<std::string> warnings_;
}

// namespace Parameter
using NodeParameter = Parameter<rclcpp::Parameter>;
std::map<std::string, std::shared_ptr<NodeParameter>>
NodeParameter::declare() override{ /* returns shared_from_this */ };

/*
 * Maintains all Parameter instances and does the heavy lifting with actually declaring and interfacing with dynamic
 * rclcpp Parameters. Provides additional convenience functions for validating all parameters, printing, etc...
 */
class NodeConfig
{
public:
	NodeConfig(const rclcpp::Node::SharedPtr& node, const std::string base_name = "");

protected:
	/*
	 * Declare Parameter types, internally calls Parameter<ParameterT>::declare() to get
	 * all actual node parameters which are then declared using the node parameter interface.
	 *
	 * This eventually calls the ros interfaces for declaring and getting the parameter.
	 */
	template <typename ParameterT>
	Parameter<ParameterT>& declare(Parameter<ParameterT>& parameter);

	/*
	 * Undeclare a parameter, remove all callbacks that have been registered using the declare() call
	 */
	template <typename ParameterT>
	bool undeclare(const Parameter<ParameterT>& parameter);
	bool undeclare(const std::string& name);

	/*
	 * Return the parameter for a given name, if not found parameter is nullptr
	 */
	// Not possible when working with references, maybe only for actual node parameters?
	// std::shared_ptr<Parameter<ParameterT>> get(const std::string& name);
	// std::shared_ptr<const Parameter<ParameterT>> get(const std::string& name) const noexcept;

	//
	// TODO: insert utility functions:  validate(), print, ostream<<, errors(), warnings()...
	//

private:
	const rclcpp::Node::SharedPtr node_;
	const std::string base_name_;

	// Class similar to the one in rosparam_shortcuts/node_parameters.h
	// implements ROS calls/checks/callbacks for node_parameters_
	NodeParametersInterface node_parameters_interface_;

	// Store parameters, only node parameters (rclcpp::Parameters) are stored inside NodeConfig
	std::map<std::string, Parameter&> parameters_;
	std::map<std::string, std::shared_ptr<NodeParameter>> node_parameters_;

	// Needed? keep track of registered callbacks that are required for linking SetParameters requests or
	// exposing high-level update changes (invalid/valid states)
	// std::string<std::string, std::string> registered_node_parameter_callbacks_ids_;
	// std::string<std::string, std::string> registered_parameter_callbacks_ids_;
}

struct ExampleConfig : public NodeConfig
{
	ExampleConfig(const rclcpp::Node::SharedPtr& node) : NodeConfig(node) {
		// Build and declare the parameters

		// declare simple parameters just like that
		declare(int1_param.describe("This is the first int parameter"));
		declare(int2_param.describe("This is the second int parameter"));
		declare(int3_param.describe("This is the third int parameter"));

		// "double_param", add some description and validity checks
		declare(double_param
			.describe("This is a very important double parameter", "should be greater than 0 and not 7")
			.warnIf([](const double& val, std::string& message) {
				message = "Value is 7, are you sure this is a good value?";
				return val == 7;
			})
			.errorIf([](const double& val, std::string& message) {
				message = "Value must not be negative!";
				return val < 0;
			})
		);

		// "string_param", use a parameter error condition "EMPTY"
		declare(string_param
			.describe("meh")
			.warnIf([](const std::string& val, std::string& message) {
				message = "string is much too long";
				return val.size() > 100;
			})
			.errorIf(EMPTY)
		);

		// "double_map_param", validate keys and values, reject if not valid
		declare(double_map_param
			.describe("6-dof joint state")
			.errorIf([](const auto& values, std::string& message) {
				message = "Invalid joint state map values, expected 6-dof";
				return values.size() != 6 || checkJointNamesInRobotModel(values);  // from somewhere
			})
		);

		//
		// can already check for errors/warnings here...
		//
	}

	Parameter<double> double_param{ "double_param", 0 };
	Parameter<std::vector<std::string>> string_param{ "string_param", {"first", "second"} };
	Parameter<std::map<std::string, double>> double_map_param{ "double_map_param"};
	Parameter<int> int1_param{ "int1_param", 0 };
	Parameter<int> int2_param{ "int2_param", 1 };
	Parameter<int> int3_param{ "int3_param", 2 };
}

void useConfig(const rclcpp::Node::SharedPtr& node)
{
	// use non-const config for initializing callbacks
	// For safety, it should only be accessed as const in nodes
	const ExampleConfig config(node);

	// access copy of value
	double value = config.double_param();

	// ...

	// React to parameter changes (sub is a bad example, but planning pipeline, plugin etc would work well)
	auto sub = rclcpp::create_subscription(config.string_param, ...);  // sub managed somewhere else
	config.string_param->onChanged(
			[&](const std::string& value) { sub = rclcpp::create_subscription(config.value, ...); });

	// ...

	auto robot_state = /* get robot state copy from somewhere else */;
	auto update_robot_state = [this, robot_state](const std::map<std::string, double> values) {
		for (const auto& [name, val] : values)
			robot_state->setVariablePosition(name, val);

		// other thread-safe member function somewhere
		this->updateRobotState(robot_state);
	};
	config.double_map_param.apply(update_robot_state);
	config.double_map_param.onChanged(update_robot_state);
}
}  // namespace rosparam_shortcuts
