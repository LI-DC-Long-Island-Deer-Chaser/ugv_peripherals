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

// trying to merge lights and speaker action client into one
#include "ugv_interfaces/action/blink_lights.hpp"

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
		using BlinkLights = ugv_interfaces::action::BlinkLights;
		using GoalHandleBlinkLights = rclcpp_action::ClientGoalHandle<BlinkLights>;

		// constructor function
		explicit SpeakerActionClient(const rclcpp::NodeOptions & speaker_options = rclcpp::NodeOptions())
		: Node("speaker_action_client", speaker_options)
		{
			// creating the action client pointer
			this->speakers_client_ptr_ = rclcpp_action::create_client<PlaySpeakers>(
				this,
				"speaker");

			this->lights_client_ptr_ = rclcpp_action::create_client<BlinkLights>(
				this,
				"ugv_peripherals/blink_lights");

			// optional timer to send the goal after startup
			// wait like 500 ms or so
			this->timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
							       std::bind(&SpeakerActionClient::send_goal, this)
			);

			feedback_forwarding = false;
			waiting_for_next_speaker_goal_ = false;
		}

	private:
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;

		// pointer to speakers' action client
		rclcpp_action::Client<PlaySpeakers>::SharedPtr speakers_client_ptr_;

		// pointer to lights' action client
		rclcpp_action::Client<BlinkLights>::SharedPtr lights_client_ptr_;

		// timer for sending goal automatically
		// this timer will get used in a callback for sending the goal.
		rclcpp::TimerBase::SharedPtr timer_;

		// new timer for the 5-second wait
		rclcpp::TimerBase::SharedPtr wait_timer_;

		bool feedback_forwarding;
		bool waiting_for_next_speaker_goal_;

		// send goal function
		void send_goal()
		{
			using namespace std::placeholders;

			// cancel the timer so it only triggers once
			this->timer_->cancel();

			// wait for server to appear
			if (!this->speakers_client_ptr_->wait_for_action_server()) {
				RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
				rclcpp::shutdown();
				return;
			}

			// build goal message
			auto speakers_goal = PlaySpeakers::Goal();

			// prepare send goal options
			auto send_speakers_goal = rclcpp_action::Client<PlaySpeakers>::SendGoalOptions();

			// function and method stuff for the response, feedback and result
			send_speakers_goal.goal_response_callback = std::bind(&SpeakerActionClient::goal_response_callback, this, _1);
			send_speakers_goal.feedback_callback = std::bind(&SpeakerActionClient::feedback_callback, this, _1, _2);
			send_speakers_goal.result_callback = std::bind(&SpeakerActionClient::result_callback, this, _1);

			// async send goal
			this->speakers_client_ptr_->async_send_goal(speakers_goal, send_speakers_goal);
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
			if (!feedback_forwarding)
			{
				// set it to true
				feedback_forwarding = true;
				// read the feedback, and send it to action server: Lights

				if (!this->lights_client_ptr_->wait_for_action_server()) {
					RCLCPP_ERROR(this->get_logger(), "Lights action server not available");
					return;
				}

				auto lights_goal = BlinkLights::Goal();
				lights_goal.duration = feedback->time_remaining;
				lights_goal.color = "white";

				auto send_lights_goal = rclcpp_action::Client<BlinkLights>::SendGoalOptions();

				this->lights_client_ptr_->async_send_goal(lights_goal, send_lights_goal);
			}
			// display time remaining (get it from the action server)
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
					feedback_forwarding = false;

					// Acknowledge completion, and wait for 5 seconds
					waiting_for_next_speaker_goal_ = true;

					// Set a timer to send the next goal after 5 seconds
					wait_timer_ = this->create_wall_timer(
						std::chrono::seconds(5),
						std::bind(&SpeakerActionClient::send_goal, this)
					);
					break;

				case rclcpp_action::ResultCode::CANCELED:
					RCLCPP_ERROR(this->get_logger(), "Playback canceled");
					feedback_forwarding = false;
					break;

				case rclcpp_action::ResultCode::ABORTED:
					RCLCPP_ERROR(this->get_logger(), "Playback aborted");
					feedback_forwarding = false;
					break;

				default:
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
					feedback_forwarding = false;
					break;
			}

			// shutdown node after receiving result (we're done here)
			//
			// rclcpp::shutdown();
		}
	};
}

// register the node with ROS2 components
RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::SpeakerActionClient)
