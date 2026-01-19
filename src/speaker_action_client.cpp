// C++ standard libraries and other builtin includes
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

// My custom defined actions
#include "ugv_interfaces/action/play_speakers.hpp"

// ROS2 standard interfaces, and other ros2-related includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

// begin writing the client here
namespace ugv_peripherals
{
	class SpeakerActionClient : public rclcpp::Node
	{
	public:
		// helpful but not necessary
		// NOTE: the thing that follows ::action::
		// is going to be the name of the action file
		using PlaySpeakers = ugv_interfaces::action::PlaySpeakers;
		using GoalHandlePlaySpeakers = rclcpp_action::ClientGoalHandle<PlaySpeakers>;

		// constructor function
		explicit SpeakerActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node("speaker_action_client", options)
		{
			// creating the action client pointer
			this->client_ptr_ = rclcpp_action::create_client<PlaySpeakers>(
				this,
				"speaker");

			// optional timer to send the goal after startup
			// wait like 500 ms or so
			this->timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
				std::bind(&SpeakerActionClient::send_goal, this)
			);

			using namespace std::placeholders;
			// create the constructor for the subscriber thing for /mavros/battery

		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;

		// pointer to action client
		rclcpp_action::Client<PlaySpeakers>::SharedPtr client_ptr_;

		// timer for sending goal automatically
		// this timer will get used in a callback for sending the goal.
		rclcpp::TimerBase::SharedPtr timer_;

		// send goal function
		void send_goal()
		{
			using namespace std::placeholders;

			// cancel the timer so it only triggers once
			this->timer_->cancel();

			// wait for server to appear
			if (!this->client_ptr_->wait_for_action_server()) {
				RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
				rclcpp::shutdown();
				return;
			}

			// build goal message
			auto goal_msg = PlaySpeakers::Goal();

			// TODO: choose type here, 0=random wav, 1=sos.wav
			// EDIT: NVM, we're always gonna have 0 here.
			goal_msg.type = 0;
			// we'll read /mavros/battery and play sos.wav if there's a certain battery voltage value we send 1 otherwise by default it is 1

			RCLCPP_INFO(this->get_logger(), "Sending speaker goal (type=%d)", goal_msg.type);

			// prepare send goal options
			auto send_goal_options = rclcpp_action::Client<PlaySpeakers>::SendGoalOptions();

			// function and method stuff for the response, feedback and result
			send_goal_options.goal_response_callback = std::bind(&SpeakerActionClient::goal_response_callback, this, _1);
			send_goal_options.feedback_callback = std::bind(&SpeakerActionClient::feedback_callback, this, _1, _2);
			send_goal_options.result_callback = std::bind(&SpeakerActionClient::result_callback, this, _1);

			// async send goal
			this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
		}

		// goal response callback
		void goal_response_callback(const GoalHandlePlaySpeakers::SharedPtr & goal_handle)
		{
			// check what the server says about the goal

			// the goal was rejected from the server
			if (!goal_handle) {
				RCLCPP_ERROR(this->get_logger(), "Goal rejected");
				rclcpp::shutdown();
			}
			// the goal was accepted from the server
			else
			{
				RCLCPP_INFO(this->get_logger(), "Goal accepted");
			}
		}

		// feedback callback
		void feedback_callback(
			GoalHandlePlaySpeakers::SharedPtr,
			 const std::shared_ptr<const PlaySpeakers::Feedback> feedback)
		{
			// display ffplay time remaining (get it from the action server)
			RCLCPP_INFO(this->get_logger(), "Time remaining: %.2f seconds", feedback->time_remaining);
		}

		// result callback
		void result_callback(const GoalHandlePlaySpeakers::WrappedResult & result)
		{
			// the callback is literally just a result statement
			// it just says what happened, did it finish? Did it fail? etc.
			switch (result.code) {
				case rclcpp_action::ResultCode::SUCCEEDED:
					RCLCPP_INFO(this->get_logger(), "Playback finished successfully: %s",
						    result.result->debug_msg.c_str());
					break;

				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Playback canceled");
					break;

				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Playback aborted");
					break;

				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					break;
			}

			// shutdown node after receiving result (we're done here)
			rclcpp::shutdown();
		}
	};
}

// register the node with ROS2 components
RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::SpeakerActionClient)
