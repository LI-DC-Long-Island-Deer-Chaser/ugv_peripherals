// C++ standard libraries and other builtin includes
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

// My custom defined actions
#include "ugv_interfaces/action/play_speakers.hpp"

// ROS2 make a service request
#include "ugv_interfaces/srv/strip_lights.hpp"

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

			// service client created once and reused
			this->strip_lights_client_ =
			this->create_client<ugv_interfaces::srv::StripLights>(
				"/ugv_peripherals/strip_lights");

			this->subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
				"/ap/battery",
				rclcpp::QoS(10).best_effort(),
				std::bind(&SpeakerActionClient::battery_callback, this, std::placeholders::_1)
			);

			// optional timer to send the goal after startup
			// wait like 500 ms or so
			this->timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
				std::bind(&SpeakerActionClient::send_goal, this)
			);

			feedback_forwarding = false;
			speaker_done = false;
			lights_done = false;
		}

	private:
		// subscription for battery state (voltage)
		rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;

		// pointer to speakers' action client
		rclcpp_action::Client<PlaySpeakers>::SharedPtr speakers_client_ptr_;

		// pointer to lights' action client
		rclcpp_action::Client<BlinkLights>::SharedPtr lights_client_ptr_;

		// service client
		rclcpp::Client<ugv_interfaces::srv::StripLights>::SharedPtr strip_lights_client_;

		// timer for sending goal automatically
		// this timer will get used in a callback for sending the goal.
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::TimerBase::SharedPtr restart_timer_;

		bool feedback_forwarding;
		bool speaker_done;
		bool lights_done;

		void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr b)
		{
			// if they are both done that is when we'll start doing some stuff
			if (speaker_done && lights_done)
			{
				// log voltage for now
				RCLCPP_INFO(this->get_logger(), "Voltage: %f", b->voltage);
				// TODO: MAKE IT SEND A SERVICE CALL to the service server

				// BEGIN FIX
				if (!strip_lights_client_->service_is_ready()) {
					RCLCPP_ERROR(this->get_logger(), "Service not available, unable to call strip_lights.");
					return;
				}

				auto request = std::make_shared<ugv_interfaces::srv::StripLights::Request>();
				request->all_off = false;


				// [11.1,11.1-((11.1-10.5)/(7))...10.5]
				if (b->voltage < 10.5857142857 )
				{
					request->color = "red2";
				}
				else if (b->voltage < 10.6714285714)
				{
					request->color = "red1";
				}
				else if (b->voltage < 10.7571428571)
				{
					request->color = "orange";
				}
				else if (b->voltage < 10.8428571429)
				{
					request->color = "yellow";
				}
				else if (b->voltage < 10.9285714286)
				{
					request->color = "green1";
				}
				else if (b->voltage < 11.0142857143)
				{
					request->color = "green2";
				}
				else
				{
					request->color = "green3";
				}

				request->led_num = 1;

				// Send the request asynchronously
				strip_lights_client_->async_send_request(request);

				// Spin until the future is completed using shared_from_this to get the shared pointer to the node
				// rclcpp::spin_until_future_complete(this->shared_from_this(), result_future);

				// Check if the future completed successfully
				// if (result_future.get()) {
				//	RCLCPP_INFO(this->get_logger(), "Service call succeeded. Response: %s", result_future.get()->debug_msg.c_str());
				// } else {
				//	RCLCPP_ERROR(this->get_logger(), "Failed to call the service");
				// }
				// END FIX
			}
		}

		// send goal function
		void send_goal()
		{
			using namespace std::placeholders;

			speaker_done = false;
			lights_done = false;

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
				return;
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

				send_lights_goal.result_callback =
				std::bind(&SpeakerActionClient::lights_result_callback, this, std::placeholders::_1);

				this->lights_client_ptr_->async_send_goal(lights_goal, send_lights_goal);
			}
			// display time remaining (get it from the action server)
			RCLCPP_INFO(this->get_logger(), "Time remaining: %.2f seconds", feedback->time_remaining);
		}

		void lights_result_callback(const GoalHandleBlinkLights::WrappedResult &)
		{
			lights_done = true;
			check_and_restart();
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
			speaker_done = true;
			feedback_forwarding = false;
			check_and_restart();

			// shutdown node after receiving result (we're done here)
			//
			// rclcpp::shutdown();
		}

		void check_and_restart()
		{
			if (speaker_done && lights_done)
			{
				this->restart_timer_ = this->create_wall_timer(
					std::chrono::milliseconds(5000),
					std::bind(&SpeakerActionClient::send_goal, this)
				);
			}
		}
	};
}
// register the node with ROS2 components
RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::SpeakerActionClient)
