// SERIAL DEVICE NODE
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstring>
#include <chrono>
#include <atomic>

// Serial communication includes
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <filesystem>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Custom interfaces
#include "ugv_interfaces/srv/serial_write.hpp"
#include "ugv_interfaces/msg/serial_data.hpp"

using namespace std;

namespace ugv_peripherals {
using rcpputils::fs::exists;

class SerialDeviceNode : public rclcpp::Node {
public:
	explicit SerialDeviceNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
	: Node("serial_device_node", options) {
		using namespace placeholders;

		// Initialize serial device
		if (!this->init_serial_device()) {
			RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial device!");
			return;
		}

		// Create service server for serial write
		this->serial_write_service_ = this->create_service<ugv_interfaces::srv::SerialWrite>(
			"ugv_peripherals/serial_write",
			bind(&SerialDeviceNode::serial_write_callback, this, _1, _2));

		// Create publisher for serial data (received from device)
		this->serial_data_publisher_ = this->create_publisher<ugv_interfaces::msg::SerialData>(
			"ugv_peripherals/serial_data", 10);

		// Start continuous read thread
		this->read_thread_ = thread([this]() {
			this->continuous_read_loop();
		});

		RCLCPP_INFO(this->get_logger(), "SerialDeviceNode initialized successfully!");
	}

	~SerialDeviceNode() {
		// Signal thread to stop
		this->stop_reading_ = true;
		if (this->read_thread_.joinable()) {
			this->read_thread_.join();
		}
		// Close serial port
		if (this->fd_ > 0) {
			close(this->fd_);
		}
	}

private:
	int fd_ = -1;
	std::vector<uint8_t> read_buffer_;
	std::mutex serial_mutex_;
	std::atomic<bool> stop_reading_{false};
	std::thread read_thread_;

	rclcpp::Service<ugv_interfaces::srv::SerialWrite>::SharedPtr serial_write_service_;
	rclcpp::Publisher<ugv_interfaces::msg::SerialData>::SharedPtr serial_data_publisher_;

	/**
	 * @brief Initialize serial device by finding and opening the USB serial adapter
	 * @return true if successful, false otherwise
	 */
	bool init_serial_device() {
		std::string dir = "/dev/serial/by-id/";

		// Ensure directory exists
		if (!exists(dir)) {
			RCLCPP_ERROR(this->get_logger(), "Directory %s does not exist", dir.c_str());
			return false;
		}

		// Search for the Qinheng USB serial device
		const std::string beginning = "usb-1a86_USB_Single_Serial";
		std::string device_path;

		for (const auto &entry : filesystem::directory_iterator(dir)) {
			std::string unix_dev_name = entry.path().filename();

			if (unix_dev_name.substr(0, beginning.size()) == beginning) {
				device_path = unix_dev_name;
				RCLCPP_INFO(this->get_logger(), "Found serial device: %s", unix_dev_name.c_str());
				break;
			}
		}

		if (device_path.empty()) {
			RCLCPP_ERROR(this->get_logger(), "Could not find USB serial device with prefix: %s", 
				beginning.c_str());
			return false;
		}

		device_path = dir + device_path;

		// Open the device
		this->fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (this->fd_ <= 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_path.c_str());
			return false;
		}

		// Configure serial port
		struct termios tty;
		memset(&tty, 0, sizeof(tty));

		if (tcgetattr(this->fd_, &tty) != 0) {
			RCLCPP_ERROR(this->get_logger(), "Could not get serial port attributes");
			close(this->fd_);
			this->fd_ = -1;
			return false;
		}

		// Set baud rate to 9600
		cfsetospeed(&tty, B9600);
		cfsetispeed(&tty, B9600);

		// Set other serial port parameters
		tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 data bits
		tty.c_cflag &= ~PARENB;                       // No parity
		tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
		tty.c_cflag &= ~CRTSCTS;                      // No hardware flow control
		tty.c_cflag |= (CLOCAL | CREAD);              // Local connection, enable receiver

		tty.c_lflag = 0;                              // Non-canonical mode
		tty.c_oflag = 0;                              // No output processing
		tty.c_iflag = 0;                              // No input processing

		tty.c_cc[VMIN] = 1;                           // Wait for at least 1 character
		tty.c_cc[VTIME] = 5;                          // 500ms timeout

		if (tcsetattr(this->fd_, TCSANOW, &tty) != 0) {
			RCLCPP_ERROR(this->get_logger(), "Could not configure serial port");
			close(this->fd_);
			this->fd_ = -1;
			return false;
		}

		// Initialize read buffer
		this->read_buffer_.resize(256);

		RCLCPP_INFO(this->get_logger(), "Serial device initialized: %s", device_path.c_str());
		return true;
	}

	/**
	 * @brief Continuous thread function that reads from serial port and publishes data
	 */
	void continuous_read_loop() {
		while (!this->stop_reading_ && rclcpp::ok()) {
			if (this->fd_ <= 0) {
				rclcpp::sleep_for(chrono::milliseconds(100));
				continue;
			}

			std::lock_guard<std::mutex> lock(this->serial_mutex_);

			// Read from serial port
			ssize_t n = read(this->fd_, this->read_buffer_.data(), this->read_buffer_.size());

			if (n > 0) {
				// Create and publish serial data message
				auto msg = std::make_shared<ugv_interfaces::msg::SerialData>();
				msg->data.assign(this->read_buffer_.begin(), this->read_buffer_.begin() + n);
				msg->timestamp_ns = this->now().nanoseconds();

				this->serial_data_publisher_->publish(*msg);

				RCLCPP_DEBUG(this->get_logger(), "Received %ld bytes from serial device", n);
			} else if (n < 0) {
				RCLCPP_WARN(this->get_logger(), "Serial read error");
			}

			// Small sleep to prevent busy waiting
			rclcpp::sleep_for(chrono::milliseconds(10));
		}
	}

	/**
	 * @brief Service callback for serial write requests
	 */
	void serial_write_callback(
		const std::shared_ptr<ugv_interfaces::srv::SerialWrite::Request> request,
		std::shared_ptr<ugv_interfaces::srv::SerialWrite::Response> response) {
		
		if (this->fd_ <= 0) {
			response->success = false;
			response->error_msg = "Serial device not open";
			RCLCPP_ERROR(this->get_logger(), "Serial device not open");
			return;
		}

		if (request->data.empty()) {
			response->success = false;
			response->error_msg = "No data to write";
			RCLCPP_WARN(this->get_logger(), "Write request with no data");
			return;
		}

		std::lock_guard<std::mutex> lock(this->serial_mutex_);

		// Write data to serial port
		ssize_t n = write(this->fd_, request->data.data(), request->data.size());

		if (n < 0) {
			response->success = false;
			response->error_msg = "Write failed";
			RCLCPP_ERROR(this->get_logger(), "Serial write failed");
			return;
		}

		if (n != (ssize_t)request->data.size()) {
			response->success = false;
			response->error_msg = "Partial write";
			RCLCPP_WARN(this->get_logger(), "Partial write: %ld of %lu bytes", 
				n, request->data.size());
			return;
		}

		response->success = true;
		response->error_msg = "";

		// Log the write
		RCLCPP_INFO(this->get_logger(), "Successfully wrote %ld bytes", n);
	}
};

} // namespace ugv_peripherals

RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::SerialDeviceNode)
