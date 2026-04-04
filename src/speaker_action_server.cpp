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
#include <atomic>

// file parsing
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>

// audio
#include <alsa/asoundlib.h>

// thread
#include <thread>

// My custom defined actions
#include "ugv_interfaces/action/play_speakers.hpp"

// ROS2 standard interfaces, and other ros2-related includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// to get the list of files
#include "ament_index_cpp/get_package_share_directory.hpp"

// File reading tools prototypes
uint32_t read_uint32(std::ifstream& file);
uint16_t read_uint16(std::ifstream& file);
bool chunk_reader(std::ifstream& file, std::string identifier);
void chunk_reader(std::ifstream& file, std::string identifier, uint32_t* size, std::string err_message);

// Audio playback prototype
void playWavThreadFunc(std::ifstream file,
		       uint32_t data_size,
		       uint16_t num_channels,
		       uint16_t bits_per_sample,
		       uint32_t sample_rate,
		       std::atomic<bool>& stop_flag,
		       double* duration_remaning);
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
				"speaker_action_server",
				std::bind(&SpeakerActionServer::handle_goal, this),
				std::bind(&SpeakerActionServer::handle_cancel, this, _1),
				std::bind(&SpeakerActionServer::handle_accepted, this, _1)
			);
		}

	private:

		// used for picking random noise files
		std::string pick_random_wav(const std::string & dir)
		{
// 			RCLCPP_INFO(this->get_logger(), "Attempting to pick random wav.");

			// Ensure directory exists
			if (!std::filesystem::exists(dir)) {
				RCLCPP_ERROR(this->get_logger(), "Why on Earth does the directory NOT exist?!");
				throw std::runtime_error("Directory does not exist");
			}

			if (!std::filesystem::is_directory(dir)) {
// 				RCLCPP_ERROR(this->get_logger(), "Audio path is not a directory: ");
				throw std::runtime_error("Path is not a directory");
			}

			std::vector<std::string> files;
			for (const auto & entry : std::filesystem::directory_iterator(dir)) {
				if (entry.path().extension() == ".wav") {
					files.push_back(entry.path().string());
// 					RCLCPP_INFO(this->get_logger(), "Found entry path: %s", entry.path().c_str());
				}
			}

			if (files.empty()) {
				RCLCPP_ERROR(this->get_logger(), "No .wav files!");
				throw std::runtime_error("No wav files found");
			}

			static std::mt19937 rng{std::random_device{}()};
			std::uniform_int_distribution<size_t> dist(0, files.size() - 1);
			return files[dist(rng)];
		}

		rclcpp_action::Server<PlaySpeakers>::SharedPtr action_server_;

		std::atomic_bool speaker_busy_{false};

		// called when a goal request comes in
		rclcpp_action::GoalResponse handle_goal()
		{
			RCLCPP_INFO(this->get_logger(), "An action request came in.");

			if (speaker_busy_.load()) {  // <-- ADDED
				RCLCPP_WARN(this->get_logger(), "Speaker busy, rejecting goal");
				return rclcpp_action::GoalResponse::REJECT;
			}

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

			speaker_busy_.store(true);

			// wave file path
			std::string wav_file;

			// select correct file based on type
			try {
				// there will always be a detterrence going on
				// no SOS anymore
				// Get the installed package share directory path
				std::string package_share_dir = ament_index_cpp::get_package_share_directory("ugv_peripherals");
				std::string audio_dir = package_share_dir + "/resource/audio_lists_wav";
				wav_file = pick_random_wav(audio_dir);
			} catch (const std::exception & e) {
				speaker_busy_.store(false);
				result->finished = false;
				result->debug_msg = e.what();
				goal_handle->abort(result);
				RCLCPP_ERROR(this->get_logger(), "Exception called, there was an issue with the file. Filename: %s", wav_file.c_str());
				return;
			}

			// BEGIN WAVE FILE PARSING
			// From Wikipedia about WAV Files and its header.
			/*
			 *	 [Master RIFF chunk]
			 *	 FileTypeBlocID  (4 bytes) : Identifier <<RIFF>>  (0x52, 0x49, 0x46, 0x46)
			 *	 FileSize        (4 bytes) : Overall file size minus 8 bytes
			 *	 FileFormatID    (4 bytes) : Format = <<WAVE>>  (0x57, 0x41, 0x56, 0x45)
			 *
			 *	 [Chunk describing the data format]
			 *	 FormatBlocID    (4 bytes) : Identifier <<fmt >>  (0x66, 0x6D, 0x74, 0x20)
			 *	 BlocSize        (4 bytes) : Chunk size minus 8 bytes, which is 16 bytes here  (0x10)
			 *	 AudioFormat     (2 bytes) : Audio format (1: PCM integer, 3: IEEE 754 float)
			 *	 NbrChannels     (2 bytes) : Number of channels
			 *	 Frequency       (4 bytes) : Sample rate (in hertz)
			 *	 BytePerSec      (4 bytes) : Number of bytes to read per second (Frequency * BytePerBloc).
			 *	 BytePerBloc     (2 bytes) : Number of bytes per block (NbrChannels * BitsPerSample / 8).
			 *	 BitsPerSample   (2 bytes) : Number of bits per sample
			 *
			 *	 (Chunk containing metadata or before data format)
			 *	 ListBlocID      (4 bytes) : Identifier <<LIST>> (0x4C, 0x49, 0x53, 54)
			 *	 ListSize        (4 bytes) : Chunk size minus 8 bytes
			 *
			 *	 [Chunk containing the sampled data]
			 *	 DataBlocID      (4 bytes) : Identifier <<data>>  (0x64, 0x61, 0x74, 0x61)
			 *	 DataSize        (4 bytes) : SampledData size
			 *	 SampledData
			 */

			// https://cplusplus.com/doc/tutorial/files/
			RCLCPP_INFO(this->get_logger(), "Playing %s", wav_file.c_str());
			std::ifstream file;
			file.open(wav_file, std::ios::in | std::ios::binary);

			uint32_t file_size = 0;
			// WAV files are RIFF type files.
			chunk_reader(file, "RIFF", &file_size, "This is not a RIFF type file.");

			if (!chunk_reader(file, "WAVE"))
			{
				RCLCPP_ERROR(this->get_logger(), "THIS IS NOT A WAVE FILE");
				rclcpp::shutdown();
			}

			uint32_t fmt_size = 0;
			chunk_reader(file, "fmt ", &fmt_size, "[fmt ] expected here... anyways!");
			// Side note: IF it did, it wouldn't matter anyways!
			// when it is not found, it goes backwards 4 characters

			// Side note 2: IF I didn't find it, it's probably because
			// we've bumped into some annoying metadata.
			// Check the fmt_size to see if it was found.

			uint16_t audio_format;
			uint16_t channel_count;
			uint32_t frequency;
			uint32_t bytes_per_sec;
			uint16_t bytes_per_blk;
			uint16_t bits_per_sample;

			// could not find format size. Maybe we have metadata instead?
			if (!fmt_size)
			{
				uint32_t jumpahead = 0;
				// if so, jump ahead first
				chunk_reader(file, "LIST", &jumpahead, "[LIST] expected here... anyways!");
				// std::cout << "I see we have " << jumpahead << " bytes of garbage/metadata. Skipping ahead!" << std::endl;
				file.seekg((long)(file.tellg()) + jumpahead);

				// after jumping over metadata, reattempt to read fmt (last chance)
				chunk_reader(file, "fmt ", &fmt_size, "[fmt ] expected here.");
			}
			audio_format    = read_uint16(file);
			channel_count   = read_uint16(file);
			frequency       = read_uint32(file);
			bytes_per_sec   = read_uint32(file);	// ALSA doesn't care
			bytes_per_blk   = read_uint16(file);	// ALSA doesn't care
			bits_per_sample = read_uint16(file);	// ALSA doesn't care too much

			// now, re-attempt to read metadata one more time (just in case)
			uint32_t jumpahead = 0;
			// if so, jump ahead first
			chunk_reader(file, "LIST", &jumpahead, "[LIST] expected here... anyways!");
			// std::cout << "I see we have " << jumpahead << " bytes of garbage/metadata. Skipping ahead!" << std::endl;
			file.seekg((long)(file.tellg()) + jumpahead);

			// I'm only making it for PCM 16. (If found, good, else fail LOUDLY)
			if (fmt_size != 16)
			{
				RCLCPP_ERROR(this->get_logger(), "This is not a full fledged audio player. Support only for PCM 16 audio");
				rclcpp::shutdown();
			}



			RCLCPP_INFO(this->get_logger(), "Bytes per second: %d, Bytes per block: %d", bytes_per_sec, bytes_per_blk);

			if (audio_format != 1)
			{
				RCLCPP_ERROR(this->get_logger(), "This is not a full fledged audio player. Support only for PCM 16 audio");
				rclcpp::shutdown();
			}

			uint32_t data_size;
			chunk_reader(file, "data", &data_size, "[data] expected here.");

			double remaining_time = double(data_size) / double(channel_count * (bits_per_sample >> 3) * frequency);

			// END WAVE FILE PARSING

			// BEGIN AUDIO PLAYBACK
			std::atomic<bool> stop_playback(false);
			std::thread audioThread(playWavThreadFunc,
						std::move(file), // move the ifstream into the thread
						data_size,
						channel_count,
						bits_per_sample,
						frequency,
						std::ref(stop_playback),
						&remaining_time
			);
			// END AUDIO PLAYBACK

			// monitor playback
			// this way it doesn't print the same value twice (trick I used)
			double old_value = remaining_time;

			while (rclcpp::ok() && remaining_time != 0) {
				// check cancel request
				if (goal_handle->is_canceling()) {
					speaker_busy_.store(false);
					result->finished = false;
					result->debug_msg = "Playback canceled";
					goal_handle->canceled(result);
					RCLCPP_INFO(this->get_logger(), "Playback canceled");
					return;
				}

				// this way it doesn't print the same value twice *wav-reporting.cpp trick I used*
				if (old_value != remaining_time)
				{
					feedback->time_remaining = remaining_time;
					old_value = remaining_time;

					// feedback published
					goal_handle->publish_feedback(feedback);

					// For debugging, because for some reason
					// it originally published 0 as the first remaining time feedback.
					RCLCPP_INFO(this->get_logger(), "Remaining time %f", remaining_time);
				}

				// sleep for 30ms wait.
				rclcpp::sleep_for(std::chrono::milliseconds(30));
			}

			// one last time to really seal the deal.
			feedback->time_remaining = 0;
			goal_handle->publish_feedback(feedback);

			// final result
			result->finished = true;
			result->debug_msg = wav_file;
			goal_handle->succeed(result);

			// Stop playback if needed
			// stop_playback = true;
			if (audioThread.joinable()) {
				audioThread.join(); // blocks here only until playback thread finishes
				speaker_busy_.store(false);
			}


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


// File reading tools code
uint32_t read_uint32(std::ifstream& file)
{
	char buffer[4];
	file.read(buffer,4);
	return   uint32_t(uint8_t(buffer[0])) 		|
		(uint32_t(uint8_t(buffer[1])) << 8) 	|
		(uint32_t(uint8_t(buffer[2])) << 16) 	|
		(uint32_t(uint8_t(buffer[3])) << 24);
}

uint16_t read_uint16(std::ifstream& file)
{
	char buffer[2];
	file.read(buffer,2);
	return   uint16_t(uint8_t(buffer[0])) |
		(uint16_t(uint8_t(buffer[1])) << 8);
}

void chunk_reader(std::ifstream& file, std::string identifier, uint32_t* size, std::string err_message)
{
	// find out where the pointer is before reading
	auto starting = file.tellg();

	// actually read 4 bytes here
	char buffer[4];
	file.read(buffer,4);

	// check if it is not equal
	if (strncmp(buffer, identifier.c_str(), 4))
	{
		// print error, return pointer
		RCLCPP_WARN(rclcpp::get_logger("speaker_action_server"), "Read Identifier: 0x%x,%x,%x,%x\n%s\n", buffer[0], buffer[1], buffer[2], buffer[3], err_message.c_str());
		file.seekg(starting);
		*size = 0;
		return;
	}

	// read another 4 bytes to see the size of the chunk.
	// NOTE: This turtles all the way down. Remember the Discord Protobuf nonsense?
	*size = read_uint32(file);

	RCLCPP_INFO(rclcpp::get_logger("speaker_action_server"), "Found chunk [%s] size: %d\n", identifier.c_str(), *size);
}

bool chunk_reader(std::ifstream& file, std::string identifier)
{
	char buffer[4];
	file.read(buffer,4);
	// found
	if (!strncmp(buffer, identifier.c_str(), 4))
	{
		RCLCPP_INFO(rclcpp::get_logger("speaker_action_server"), "Found chunk [%s]\n", identifier.c_str());
		return true;
	}
	else
	{
		RCLCPP_ERROR(rclcpp::get_logger("speaker_action_server"), "[%s] not found.\n", identifier.c_str());
		return false;
	}
}

// Audio playback function code
void playWavThreadFunc(std::ifstream file,
		       uint32_t data_size,
		       uint16_t num_channels,
		       uint16_t bits_per_sample,
		       uint32_t sample_rate,
		       std::atomic<bool>& stop_flag,
		       double* duration_remaning)
{
	snd_pcm_t* pcm;
	if (snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
		std::cerr << "Failed to open ALSA device\n";
		return;
	}

	// Choose PCM format from bits_per_sample
	snd_pcm_format_t pcm_format = SND_PCM_FORMAT_UNKNOWN;
	if (bits_per_sample == 16) {
		pcm_format = SND_PCM_FORMAT_S16_LE;
	} else {
		std::cerr << "Unsupported bits_per_sample\n";
		snd_pcm_close(pcm);
		return;
	}

	snd_pcm_set_params(
		pcm,
		pcm_format,
		SND_PCM_ACCESS_RW_INTERLEAVED,
		num_channels,
		sample_rate,
		1,
		500000
	);

	const uint32_t bytes_per_frame =
	num_channels * (bits_per_sample >> 3);

	const uint32_t total_frames =
	data_size / bytes_per_frame;

	const uint32_t bufferFrames = 1024;
	std::vector<char> buffer(bufferFrames * bytes_per_frame);

	uint32_t frames_played = 0;

	while (!stop_flag && frames_played < total_frames) {
		uint32_t frames_to_read =
		(bufferFrames < (total_frames - frames_played))
		? bufferFrames
		: (total_frames - frames_played);

		file.read(buffer.data(), frames_to_read * bytes_per_frame);
		std::streamsize bytes_read = file.gcount();

		uint32_t frames_actual = bytes_read / bytes_per_frame;
		if (frames_actual == 0) break;

		snd_pcm_writei(pcm, buffer.data(), frames_actual);
		frames_played += frames_actual;

		// Update remaining duration (seconds)
		if (duration_remaning) {
			*duration_remaning =
			double(total_frames - frames_played) /
			double(sample_rate);
		}
	}

	snd_pcm_drain(pcm);
	snd_pcm_close(pcm);

	// if anything is left, say 0.
	if (duration_remaning) {
		*duration_remaning = 0.0;
	}
}
