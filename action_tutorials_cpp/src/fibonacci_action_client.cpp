#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
	class FibonacciActionClient : public rclcpp::Node
	{
	public:
		// This basically just "aliases" the types.
		using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
		using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

		// explicit means to explicit define the constructor.
		// The FibonacciActionClient is inherited from a Node, based off a Node
		explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
		: Node("fibonacci_action_client", options)
		{
			// this is pseudo-python code
			// self.client_ptr_ = create_client("fibonacci")
			// this is the fibonacci client
			this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
				this,
				"fibonacci");
			// this is pseudo-python code
			// self.timer_ = create_wall_timer_ms(500, send_goal)
			// assuming that send_goal is a function (which it is).
			// We have to pass it as an arg for create_wall_timer()
			this->timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
							       std::bind(&FibonacciActionClient::send_goal, this));
		}

		void send_goal()
		{
			// we use placeholders for some functions
			// _1, _2, _3, etc. These are effectively saying,
			// "It don't matter what these parameters are, we won't be using 'em"
			using namespace std::placeholders;

			// this is pseudo-python code
			// self.timer.cancel()
			this->timer_->cancel();

			// this is pseudo-python code
			// if not self.client_ptr.wait_for_action_server(): # If this action server is not ready
			if (!this->client_ptr_->wait_for_action_server()) {
				// Just say that it is not available after waiting.
				// I presume there is some kind of timeout that ROS2 defines
				// ... because if it's define here, I don't see it! (or I missed something?)
				RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
				// but like, also, my LSP says I didn't write it

				// shutdown function because not available
				rclcpp::shutdown();
			}

			// sigh... I hate "auto" types it's "lazy"
			// anyways... this is pseud-python code (again)
			// goal_msg = Goal()
			auto goal_msg = Fibonacci::Goal();

			// assuming that Goal() from the pseudo-python has order as an attribute...
			goal_msg.order = 10;

			// log it out with INFO that you're sending a goal
			RCLCPP_INFO(this->get_logger(), "Sending goal");

			// This is going to be used to bind to function names
			auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

			// terrible spacing, but whatever. Recall that actions
			// have a request (reesponse), feedback, and result.
			// we are the client, so we need to process the response
			// from the server, the server processes the request.

			// BEGIN RESPONSE
			send_goal_options.goal_response_callback =
			std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
			// END RESPONSE

			// BEGIN FEEDBACK
			send_goal_options.feedback_callback =
			std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
			// END FEEDBACK

			// BEGIN RESULT
			send_goal_options.result_callback =
			std::bind(&FibonacciActionClient::result_callback, this, _1);
			this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
			// END RESULT

			// all of them have the shape of
			// send_goal_options.callback_function_name_goes_here = std::bind(&ClientADDR::callback_function_name_goes_here, this, _1,...);
		}

	// the following contains all the definitions of the client_ptr_, timer_ and methods
	// of the class. These are all to be tossed inside private. Hidden interface from others
	// (Why do we even do this, idk, just seems like it's making our life harder).
	private:
		rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
		rclcpp::TimerBase::SharedPtr timer_;

		// process the response the server gave to us.
		// IMPORTANT INFO: FOR OUR CASE, we may need to reject goals
		// from incoming client requests, depending on if we're "too-busy"
		// to process such requests.
		void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
		{
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
			} else {
				RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
			}
		}

		// process the feedback the server gave to us
		void feedback_callback(
			GoalHandleFibonacci::SharedPtr,
			 const std::shared_ptr<const Fibonacci::Feedback> feedback)
		{
			std::stringstream ss;
			ss << "Next number in sequence received: ";
			for (auto number : feedback->partial_sequence) {
				ss << number << " ";
			}
			RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
		}

		// process the result that the server gave to us
		void result_callback(const GoalHandleFibonacci::WrappedResult & result)
		{
			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					break;
				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
					return;
				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
					return;
				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					return;
			}

			// side note, I hade overloading, what on Earth is this:
			std::stringstream ss;
			ss << "Result received: ";
			for (auto number : result.result->sequence) {
				ss << number << " ";
			}
			RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
			rclcpp::shutdown();
		}
	};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

// soo many macros what on Earth ;-;
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
