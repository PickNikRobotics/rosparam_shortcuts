#pragma once

#include <mutex>
#include <shared_mutex>

#include <rclcpp>

namespace rosparam_shortcuts

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
	void describe(const std::string& description, const std::string& additional_constraints = "");
	// Allow setting additional properties. i.e. read_only, dynamic_typing?
	// void setProperties(...);

	// Validity Status
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

	// Copy-set/Copy-get value using read/write lock internally
	ParameterT& operator=(const ParameterT& value);
	ParameterT operator() const;  // or:  'operator ParameterT() const;'

	// Validity Callbacks (could also use rcl_interfaces::msg::SetParametersResult like in node_parameters.h).
	// Callbacks are stored in ordered hash map, could also possibly return hash for deleting later on.
	// Alternatively, return Parameter<ParameterT>& for method chaining
	using ParameterValidityCallback = std::function<bool(const ParameterT& value, std::string& message)>;
	void warnIf(ParameterValidityCallback callback);
	void errorIf(ParameterValidityCallback callback);  // or rejectIf()/failIf()..
	// Predefined generic (type-specific?) properties, i.e. 'NO_OVERRIDE', 'EMPTY', 'NORMALIZED'
	enum ParameterProperty
	{
		...
	};
	void warnIf(ParameterProperty property);
	void errorIf(ParameterProperty property);

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
	// NOTE: not sure if this is really a good solution or if we should simply copy the date and synchronize
	// using child/parent callbacks only
	Parameter(const std::string& name, ParameterT& value, std::shared_mutex parent_mutex);

private:
	// Initialize child parameter and call declare() function recursively
	template <ChildParameterT>
	void declareChildParam(const std::string, std::map<std::string, std::shared_ptr<NodeParameter>>);

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

	/*
	 * Declare Parameter types, internally calls Parameter<ParameterT>::declare() to get
	 * all actual node parameters which are then declared using the node parameter interface.
	 */
	template <typename ParameterT>
	void declare(const std::shared_ptr<Parameter<ParameterT>>& parameter);
	template <typename ParameterT>
	std::shared_ptr<Parameter<ParameterT>> declare(const std::string& name, ...);

	/*
	 * Undeclare a parameter, remove all callbacks that have been registered using the declare() call
	 */
	template <typename ParameterT>
	bool undeclare(const std::shared_ptr<Parameter<ParameterT>>& parameter);
	bool undeclare(const std::string& name);

	/*
	 * Return the parameter for a given name, if not found parameter is nullptr
	 */
	std::shared_ptr<Parameter<ParameterT>> get(const std::string& name);

	//
	// TODO: insert utility functions:  validate(), print, ostream<<, errors(), warnings()...
	//

private:
	const rclcpp::Node::SharedPtr node_;
	const std::string base_name_;

	std::map<std::string, std::shared_ptr<Parameter>> parameters_;
	std::map<std::string, std::shared_ptr<NodeParameter>> node_parameters_;

	// Needed? keep track of registered callbacks that are required for linking SetParameters requests or
	// exposing high-level update changes (invalid/valid states)
	// std::string<std::string, std::string> registered_node_parameter_callbacks_ids_;
	// std::string<std::string, std::string> registered_parameter_callbacks_ids_;
}

}  // namespace rosparam_shortcuts
