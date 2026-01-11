// C++ standard libraries and other builtin includes
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <filesystem>
#include <random>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <mutex>  // for single-speaker lock

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
		// get audio duration
		double get_audio_duration(const std::string & file)
		{
			// just a small buffer to hold
			char buffer[128];
			std::string cmd = "ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 \"" + file + "\"";
			FILE* pipe = popen(cmd.c_str(), "r");

			// looks like there is no open file capability
			if (!pipe){
				return 0.0;
			}

			// result gets taken from buffer to later return
			std::string result;
			while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
			{
				result += buffer;
			}
			pclose(pipe);

			try
			{
				return std::stod(result);
			}
			catch (...)
			{
				return 0.0;
			}
		}

		// used for picking random noise files
		std::string pick_random_wav(const std::string & dir)
		{
			std::vector<std::string> files;
			for (const auto & entry : std::filesystem::directory_iterator(dir)) {
				if (entry.path().extension() == ".wav") {
					files.push_back(entry.path().string());
				}
			}

			if (files.empty()) {
				throw std::runtime_error("No wav files found");
			}

			static std::mt19937 rng{std::random_device{}()};
			std::uniform_int_distribution<size_t> dist(0, files.size() - 1);
			return files[dist(rng)];
		}

		rclcpp_action::Server<PlaySpeakers>::SharedPtr action_server_;
		std::mutex speaker_mutex_; // ensures only one playback at a time

		// called when a goal request comes in
		rclcpp_action::GoalResponse handle_goal(
			const rclcpp_action::GoalUUID & uuid,
			std::shared_ptr<const PlaySpeakers::Goal> goal)
		{
			RCLCPP_INFO(this->get_logger(), "Test: I received type=%d", goal->type);
			(void)uuid;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		// called when a goal cancel request comes in
		rclcpp_action::CancelResponse handle_cancel(
			const std::shared_ptr<GoalHandlePlaySpeakers> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
			(void)goal_handle;
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		// main goal execution function
		void execute(const std::shared_ptr<GoalHandlePlaySpeakers> goal_handle)
		{
			RCLCPP_INFO(this->get_logger(), "Executing goal");

			// result, feedback, goal
			auto result = std::make_shared<PlaySpeakers::Result>();
			auto feedback = std::make_shared<PlaySpeakers::Feedback>();
			const auto goal = goal_handle->get_goal();

			// ensure only one playback at a time
			std::unique_lock<std::mutex> lock(speaker_mutex_, std::try_to_lock);
			if (!lock.owns_lock()) {
				result->finished = false;
				result->debug_msg = "Speaker busy";
				goal_handle->abort(result);
				return;
			}

			// wave file path
			std::string wav_file;

			// select correct file based on type
			try {
				// this is to align with the PlaySpeakers.action
				// 0 is for a random deterring roaring noise
				if (goal->type == 0)
				{
					wav_file = pick_random_wav("resource");
				}
				// this is for the sos.wav file for if the battery is low
				else if (goal->type == 1)
				{
					wav_file = "resource/sos.wav";
				}
				// Speaker type is not valid. 0, or 1 only (for now)
				else
				{
					result->finished = false;
					result->debug_msg = "Invalid speaker type";
					goal_handle->abort(result);
					return;
				}
			} catch (const std::exception & e) {
				result->finished = false;
				result->debug_msg = e.what();
				goal_handle->abort(result);
				return;
			}

			RCLCPP_INFO(this->get_logger(), "Playing %s", wav_file.c_str());

			// record start time for feedback
			auto start_time = this->now();

			// fork/exec ffplay
			pid_t pid = fork();
			if (pid == 0) {
				execlp("ffplay", "ffplay", "-nodisp", "-autoexit", wav_file.c_str(), nullptr);
				_exit(1);
			}

			if (pid < 0) {
				result->finished = false;
				result->debug_msg = "Failed to start ffplay";
				goal_handle->abort(result);
				return;
			}

			// monitor playback
			while (rclcpp::ok()) {
				// check cancel request
				if (goal_handle->is_canceling()) {
					kill(pid, SIGTERM);
					waitpid(pid, nullptr, 0);

					result->finished = false;
					result->debug_msg = "Playback canceled";
					goal_handle->canceled(result);
					RCLCPP_INFO(this->get_logger(), "Playback canceled");
					return;
				}

				// non-blocking wait to see if playback finished
				int status;
				pid_t ret = waitpid(pid, &status, WNOHANG);
				if (ret == pid) {
					break;
				}

				// publish feedback with time elapsed
				auto now = this->now();
				// elapsed duration = deltaT
				double elapsed = (now - start_time).seconds();

				// duration extracted from ffprobe subtracted from elapsed
				// this gives us time remaining = duration - deltaT
				feedback->time_remaining = get_audio_duration(wav_file) - elapsed;

				// feedback published
				goal_handle->publish_feedback(feedback);

				// sleep for 100ms wait.
				rclcpp::sleep_for(std::chrono::milliseconds(100));
			}

			// final result
			result->finished = true;
			result->debug_msg = wav_file;
			goal_handle->succeed(result);

			RCLCPP_INFO(this->get_logger(), "Playback complete");
		}

		// called when a goal is accepted by the server
		// spins off a thread so executor is not blocked
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
