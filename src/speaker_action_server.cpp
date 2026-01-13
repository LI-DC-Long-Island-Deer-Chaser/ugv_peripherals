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

// to get the list of files
#include "ament_index_cpp/get_package_share_directory.hpp"

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
			RCLCPP_INFO(this->get_logger(), "Attempting to pick random wav.");

			// Ensure directory exists
			if (!std::filesystem::exists(dir)) {
				RCLCPP_ERROR(this->get_logger(), "Why on Earth does the directory NOT exist?!");
				throw std::runtime_error("Directory does not exist");
			}

			if (!std::filesystem::is_directory(dir)) {
				RCLCPP_ERROR(this->get_logger(), "Audio path is not a directory: ");
				throw std::runtime_error("Path is not a directory");
			}

			std::vector<std::string> files;
			for (const auto & entry : std::filesystem::directory_iterator(dir)) {
				if (entry.path().extension() == ".wav") {
					files.push_back(entry.path().string());
					RCLCPP_INFO(this->get_logger(), "Found entry path: %s", entry.path().c_str());
				}
			}

			RCLCPP_INFO(this->get_logger(), "Finished loop.");

			if (files.empty()) {
				RCLCPP_ERROR(this->get_logger(), "No .wav files!");
				throw std::runtime_error("No wav files found");
			}

			for (const auto & file : files) {
				RCLCPP_INFO(this->get_logger(), "File: %s", file.c_str());
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
			std::unique_lock<std::mutex> lock(speaker_mutex_);
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
				// there will always be a detterrence going on
				// no SOS anymore
			// Get the installed package share directory path
			std::string package_share_dir = ament_index_cpp::get_package_share_directory("ugv_peripherals");
			std::string audio_dir = package_share_dir + "/resource/audio_lists_wav";			RCLCPP_INFO(this->get_logger(), "Looking for audio files in: %s", audio_dir.c_str());			wav_file = pick_random_wav(audio_dir);
			} catch (const std::exception & e) {
				result->finished = false;
				result->debug_msg = e.what();
				goal_handle->abort(result);
				RCLCPP_ERROR(this->get_logger(), "Exception called, there was an issue with the file. Filename: %s", wav_file.c_str());
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
				RCLCPP_ERROR(this->get_logger(), "ffplay failed for some reason");

				goal_handle->abort(result);
				return;
			}

			// monitor playback
			double duration = get_audio_duration(wav_file);
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
				feedback->time_remaining = duration - elapsed;

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
