// file parsing
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <vector>

// audio
#include <alsa/asoundlib.h>

// thread
#include <thread>
#include <atomic>

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
		std::cout << err_message << std::endl;
		file.seekg(starting);
		*size = 0;
		return;
	}

	// read another 4 bytes to see the size of the chunk.
	// NOTE: This turtles all the way down. Remember the Discord Protobuf nonsense?
	*size = read_uint32(file);

	std::cout << "Found chunk [" << identifier << "] size: " << *size << std::endl;
}

// potentially unused?
bool chunk_reader(std::ifstream& file, std::string identifier)
{
	char buffer[4];
	file.read(buffer,4);
	// found
	if (!strncmp(buffer, identifier.c_str(), 4))
	{
		std::cout << "Found chunk [" << identifier << "]" << std::endl;
		return true;
	}
	else
	{
		std::cout << "[" << identifier << "] not found.";
		return false;
	}
}

void playWavThreadFunc(std::ifstream file,
		       uint32_t data_size,
		       uint16_t num_channels,
		       uint16_t bits_per_sample,
		       uint32_t sample_rate,
		       std::atomic<bool>& stop_flag)
{
	// Open ALSA device
	snd_pcm_t* pcm;
	if (snd_pcm_open(&pcm, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0) {
		std::cerr << "Failed to open ALSA device\n";
		return;
	}

	// Configure PCM
	snd_pcm_set_params(
		pcm,
		SND_PCM_FORMAT_S16_LE,
		SND_PCM_ACCESS_RW_INTERLEAVED,
		num_channels,
		sample_rate,
		1,          // allow resampling
		500000      // 0.5 sec latency
	);

	const int bufferFrames = 1024;
	std::vector<short> buffer(bufferFrames * num_channels);

	uint32_t total_frames = data_size / (num_channels * bits_per_sample / 8);
	uint32_t frames_played = 0;

	while (!stop_flag && frames_played < total_frames) {
		uint32_t frames_to_read = (bufferFrames < (total_frames - frames_played)) ?
		bufferFrames : (total_frames - frames_played);

		file.read(reinterpret_cast<char*>(buffer.data()), frames_to_read * num_channels * bits_per_sample / 8);
		std::streamsize bytes_read = file.gcount();
		uint32_t frames_actual = bytes_read / (num_channels * bits_per_sample / 8);
		if (frames_actual == 0) break;

		snd_pcm_writei(pcm, buffer.data(), frames_actual);
		frames_played += frames_actual;
	}

	snd_pcm_drain(pcm);
	snd_pcm_close(pcm);
	std::cout << "Playback finished\n";
}

int main(void)
{
	// From Wikipedia about WAV Files and its header.
	/*
	 [Master RIFF chunk]
	 FileTypeBlocID  (4 bytes) : Identifier <<RIFF>>  (0x52, 0x49, 0x46, 0x46)
	 FileSize        (4 bytes) : Overall file size minus 8 bytes
	 FileFormatID    (4 bytes) : Format = <<WAVE>>  (0x57, 0x41, 0x56, 0x45)

	 [Chunk describing the data format]
	 FormatBlocID    (4 bytes) : Identifier <<fmt >>  (0x66, 0x6D, 0x74, 0x20)
	 BlocSize        (4 bytes) : Chunk size minus 8 bytes, which is 16 bytes here  (0x10)
	 AudioFormat     (2 bytes) : Audio format (1: PCM integer, 3: IEEE 754 float)
	 NbrChannels     (2 bytes) : Number of channels
	 Frequency       (4 bytes) : Sample rate (in hertz)
	 BytePerSec      (4 bytes) : Number of bytes to read per second (Frequency * BytePerBloc).
	 BytePerBloc     (2 bytes) : Number of bytes per block (NbrChannels * BitsPerSample / 8).
	 BitsPerSample   (2 bytes) : Number of bits per sample

	 (Chunk containing metadata or before data format)
	 ListBlocID      (4 bytes) : Identifier <<LIST>> (0x4C, 0x49, 0x53, 54)
	 ListSize        (4 bytes) : Chunk size minus 8 bytes

	 [Chunk containing the sampled data]
	 DataBlocID      (4 bytes) : Identifier <<data>>  (0x64, 0x61, 0x74, 0x61)
	 DataSize        (4 bytes) : SampledData size
	 SampledData
	*/

	// https://cplusplus.com/doc/tutorial/files/
	std::ifstream file;
	file.open("/home/penguin/audio2.wav", std::ios::in | std::ios::binary);

	uint32_t file_size = 0;
	// WAV files are RIFF type files.
	chunk_reader(file, "RIFF", &file_size, "This is not a RIFF type file.");

	if (!chunk_reader(file, "WAVE"))
	{
		return 1;
	}

	uint32_t fmt_size = 0;
	chunk_reader(file, "fmt ", &fmt_size, "[fmt ] expected here... anyways!");
	// Side note: IF it did, it wouldn't matter anyways!
	// when it is not found, it goes backwards 4 characters

	// Side note 2: IF I didn't find it, it's probably because
	// we've bumped into some annoying metadata.
	// Check the fmt_size to see if it was found.

	uint32_t jumpahead = 0;
	chunk_reader(file, "LIST", &jumpahead, "[LIST] expected here... anyways!");

	// std::cout << "I see we have " << jumpahead << " bytes of garbage/metadata. Skipping ahead!" << std::endl;
	file.seekg((long)(file.tellg()) + jumpahead);

	// now we ACTUALLY need format.
	if (!fmt_size)
	{
		chunk_reader(file, "fmt ", &fmt_size, "[fmt ] expected here.");
	}

	// I'm only making it for PCM 16.
	if (fmt_size != 16)
	{
		std::cout << "This is not a full fledged audio player. Support only for PCM 16 audio";
		return 1;
	}

	uint16_t audio_format = read_uint16(file);
	uint16_t channel_count = read_uint16(file);
	uint32_t frequency = read_uint32(file);
	uint32_t bytes_per_sec = read_uint32(file);
	uint16_t bytes_per_blk = read_uint16(file);	// ALSA doesn't care
	uint16_t bits_per_sample = read_uint16(file);	// ALSA doesn't care

	if (audio_format != 1)
	{
		std::cout << "This is not a full fledged audio player. Support only for PCM 16 audio";
		return 1;
	}

	uint32_t data_size;
	chunk_reader(file, "data", &data_size, "[data] expected here.");

	std::cout << "Audio thread started" << std::endl;
	std::atomic<bool> stop_playback(false);
	std::thread audioThread(playWavThreadFunc,
				std::move(file), // move the ifstream into the thread
				data_size,
				channel_count,
				bits_per_sample,
				frequency,
				std::ref(stop_playback));
	std::cout << "Audio thread playing" << std::endl;

	// Stop playback if needed
	// stop_playback = true;
	if (audioThread.joinable()) {
		audioThread.join(); // blocks here only until playback thread finishes
	}

	return 0;
}
