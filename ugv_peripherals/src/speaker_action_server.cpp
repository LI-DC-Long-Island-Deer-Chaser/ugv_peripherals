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

// begin writing the server here.
namespace ugv_peripherals
{
	class SpeakerActionServer : public rclcpp::Node
	{
	public:
		// helpful but not necessary
		// NOTE: the thing that follows ::action::
		// 	 is going to be the name of the action file
		using PlaySpeakers = ugv_interfaces::action::PlaySpeakers;
		using GoalHandlePlaySpeakers = rclcpp_action::ServerGoalHandle<PlaySpeakers>;

		// IDK whether or not we need this and why
		// ACTION_TUTORIALS_CPP_PUBLIC

		explicit SpeakerActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
		: Node("speaker_action_server", options)
		{
			// for snowflakes in standard parameters
			// *, * -> __1, _2
			using namespace std::placeholders;

			this->action_server_ = rclcpp_action::create_server<PlaySpeakers>(
				this,
				"speaker",
				std::bind(&SpeakerActionServer::handle_goal, this, _1, _2),
																	std::bind(&SpeakerActionServer::handle_cancel, this, _1),
																	std::bind(&SpeakerActionServer::handle_accepted, this, _1)
			);
		}
	private:
		rclcpp_action::Server<PlaySpeakers>::SharedPtr action_server_;

		rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const PlaySpeakers::Goal> goal)
		{
			RCLCPP_INFO(this->get_logger(), "Test: I received type=%d", goal->type);
			(void)uuid;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandlePlaySpeakers> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
			(void)goal_handle;
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void execute(const std::shared_ptr<GoalHandlePlaySpeakers> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Executing goal");
			rclcpp::Rate loop_rate(1);
			const auto goal = goal_handle->get_goal();
			auto feedback = std::make_shared<PlaySpeakers::Feedback>();
			auto result = std::make_shared<PlaySpeakers::Result>();

			// Check if goal is done
			if (rclcpp::ok()) {
				result->finished = true;
				goal_handle->succeed(result);
				RCLCPP_INFO(this->get_logger(), "Goal succeeded");
			}
		}

		void handle_accepted(const std::shared_ptr<GoalHandlePlaySpeakers> goal_handle)
		{
			using namespace std::placeholders;
			// this needs to return quickly to avoid blocking the executor, so spin up a new thread
			std::thread(std::bind(&SpeakerActionServer::execute, this, _1), goal_handle).detach();
		}
	};
}

// all of these Windows Macros are probably not needed, but I am going to keep them just in case because why not.
RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::SpeakerActionServer)
