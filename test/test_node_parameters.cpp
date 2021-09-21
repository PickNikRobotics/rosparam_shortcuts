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
   Desc:   test node parameters dynamic and static
*/

// Testing
#include <gtest/gtest.h>

// C++
#include <thread>

// Parameters
#include <rosparam_shortcuts/rosparam_shortcuts.h>

using rosparam_shortcuts::always_accept;
using rosparam_shortcuts::NodeParameters;
using rosparam_shortcuts::static_parameter_validation;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("test_node_parameters");

class TestNodeParameters : public ::testing::Test
{
public:
	void SetUp() override {
		change_detected_ = false;

		params_.declareAndGet("length", 1.0);  // defaults to static parameter;
		params_.declareAndGet("damping", 0.5, always_accept);  // any value is valid

		// Validate parameter
		params_.declareAndGet("mass", 1.0, [](const rclcpp::Parameter& p) -> rcl_interfaces::msg::SetParametersResult {
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
		params_.registerRosCallbackReconfigure();

		// tells node the parameters have changed
		params_.registerUpdateCallback([this]() { testParameterUpdate(); });

		executor_->add_node(node_);
		executor_thread_ = std::thread([this]() { executor_->spin(); });
	}

	TestNodeParameters()
	  : node_(std::make_shared<rclcpp::Node>("test_node_parameters"))
	  , executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
	  , params_(node_, LOGGER) {}

	void TearDown() override {
		executor_->cancel();
		if (executor_thread_.joinable()) {
			executor_thread_.join();
		}
	}

	void testParameterUpdate() { change_detected_ = true; }

protected:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Executor::SharedPtr executor_;
	std::thread executor_thread_;
	NodeParameters params_;
	bool change_detected_;
};

TEST_F(TestNodeParameters, TestParameters) {
	SCOPED_TRACE("TestParameters");

	ASSERT_EQ(1.0, params_.get("length").as_double());
	ASSERT_EQ(0.5, params_.get("damping").as_double());
	ASSERT_EQ(1.0, params_.get("mass").as_double());

	rclcpp::Parameter p("mass", 2.0);
	node_->set_parameter(p);

	ASSERT_TRUE(change_detected_);

	ASSERT_EQ(1.0, params_.get("length").as_double());
	ASSERT_EQ(0.5, params_.get("damping").as_double());
	ASSERT_EQ(2.0, params_.get("mass").as_double());
}

int main(int argc, char** argv) {
	// It is important we init ros before google test because we are going to
	// create a node durring the google test init.
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);

	int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
}
