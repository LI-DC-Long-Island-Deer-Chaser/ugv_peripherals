#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <string>

// needed to write to /dev/ttyACM2 (or whatever)
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string>
#include <iostream>


#include "ugv_interfaces/action/blink_lights.hpp"
#include "ugv_interfaces/srv/strip_lights.hpp"
#include "ugv_interfaces/srv/glow_lights.hpp"
#include "ugv_interfaces/srv/head_lights.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;

namespace ugv_peripherals
{
    class SerialDevice
    {
    public:
        // SerialDevice constructor function
        SerialDevice()
        {
            // code for the constructor function goes here
            // Hardcoded, known symlink path
            const std::string device_path = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_58CD119813-if00";

            // Open the device
            this->fd_ = open(device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (this->fd_ <= 0) {
                cout << "OPENING THE QINHENG DEVICE FAILED. IT IS NOT PLUGGED IN!" << endl;
                SerialDevice::~SerialDevice();
            }

            // Configure serial port
            // set tty so that later on we can use it
            struct termios tty;
            memset(&tty, 0, sizeof(tty));

            if (tcgetattr(this->fd_, &tty) != 0) {
                close(this->fd_);
                cout << "Could not configure the USB to UART adapter!" << endl;
            }

            // setting the baud rate to 115200
            cfsetospeed(&tty, B9600);
            cfsetispeed(&tty, B9600);

            // setting c flags
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_iflag = 0;
            tty.c_cc[VMIN]  = 1;
            tty.c_cc[VTIME] = 5;

            if (tcsetattr(this->fd_, TCSANOW, &tty) != 0) {
                cout << "Could not configure the USB to UART adapter!" << endl;
                close(this->fd_);
            }

            // start it empty
            this->internal_vector_.clear();
        }

        ~SerialDevice()
        {
            close(this->fd_);
            cout << "Destructor called";
        }

        int serial_write(uint8_t byte1, uint8_t byte2, std::string selected_color)
        {
            // Using a scoped lock for serial controller use.
            // 0:  Bright green
            // 1:  Dim green
            // 2:  Very dark green
            // 3:  Yellow
            // 4:  Orange
            // 5.  Bright red
            // 6:  Dark blue
            // 7:  Blue
            // 8:  Cyan
            // 9:  Magenta
            // 10: Dark Red
            // 11: Light Blue
            // 12: Purple
            // 13: Pink
            // 14: White
            // 15: Off
            std::unordered_map<std::string, uint8_t> colors;
            colors["green3"]  = 0;
            colors["green2"]  = 1;
            colors["green1"]  = 2;
            colors["yellow"]  = 3;
            colors["orange"]  = 4;
            colors["red2"]    = 5;
            colors["blue1"]   = 6;
            colors["blue2"]   = 7;
            colors["cyan"]    = 8;
            colors["magenta"] = 9;
            colors["red1"]    = 10;
            colors["blue3"]   = 11;
            colors["purple"]  = 12;
            colors["pink"]    = 13;
            colors["white"]   = 14;
            colors["off"]     = 15;

            // 010X to turn on all the headlights, where X is color
            // 011X to turn on all the strip lights, where X is color
            // 012X to turn on the glow lights, where X is brightness
            uint8_t bytes_to_be_sent[2] = {byte1, (uint8_t)((byte2 << 4) | colors[selected_color])};
            ssize_t n = write(this->fd_, bytes_to_be_sent, 2);

            if (n < 0) {
                // errors:
                cout << " n: " << n << " Failed."<< endl;
                return 1;
            }

            // no errors:
            cout << "Successfully sent payload [" << bytes_to_be_sent[1] << "] [" << bytes_to_be_sent[0] << "]" << endl;
            return 0;
        }

        uint8_t* serial_read()
        {
            ssize_t n = read(this->fd_, this->internal_vector_.data(), this->internal_vector_.size());

            if (n < 0) {
                perror("serial read failed");

                // returns NULL address if there is no read
                return NULL;
            }

            // returns the pure data of the internal vector
            // if there even is data to be returned. It should only
            // (ever) be two bytes that is ever sent from the panel
            return this->internal_vector_.data(); // number of bytes read
        }

    private:
        int fd_ = -1;
        std::vector<uint8_t> internal_vector_;
    };


    class LightsController : public rclcpp::Node
    {
        public:
            // Action server namespace.
            using BlinkLights = ugv_interfaces::action::BlinkLights;
            using HandleBlinkLights = rclcpp_action::ServerGoalHandle<BlinkLights>;

            // Service server namespace.
            using StripLights = ugv_interfaces::srv::StripLights;
            using HeadLights = ugv_interfaces::srv::HeadLights;
            using GlowLights = ugv_interfaces::srv::GlowLights;

            explicit LightsController(const rclcpp::NodeOptions &option = rclcpp::NodeOptions())
            : Node("lights_controller", option)
            {
                using namespace placeholders;

                this->action_server_ = rclcpp_action::create_server<BlinkLights>(
                    this,
                    "ugv_peripherals/blink_lights",
                    bind(&LightsController::action_handle_goal, this, _1, _2),
                    bind(&LightsController::action_handle_cancel, this, _1),
                    bind(&LightsController::action_handle_accepted, this, _1));

                this->head_lights_service_ = this->create_service<HeadLights>(
                    "ugv_peripherals/head_lights",
                    bind(&LightsController::head_lights_callback, this, _1, _2));

                this->glow_lights_service_ = this->create_service<GlowLights>(
                    "ugv_peripherals/glow_lights",
                    bind(&LightsController::glow_lights_callback, this, _1, _2));

                this->strip_lights_service_ = this->create_service<StripLights>(
                    "ugv_peripherals/strip_lights",
                    bind(&LightsController::strip_lights_callback, this, _1, _2));

                this->s_ = SerialDevice();

            }

        private:
            // The main limited resource here is the serial controller connected on board.
            // `serial_mutex_` is used for this.
            // All other mutexes are used to prevent service call overlap (although this
            // functionallity is handled in rclcpp and correct use of callback groups and QOS).
            mutex serial_mutex_;
            mutex head_mutex_;
            mutex strip_mutex_;
            mutex glow_mutex_;

            rclcpp_action::Server<BlinkLights>::SharedPtr action_server_;

            SerialDevice s_;

            // 3 Service servers for head/strip/glow lights control. corresponding topic must be used.
            rclcpp::Service<HeadLights>::SharedPtr head_lights_service_;
            rclcpp::Service<StripLights>::SharedPtr strip_lights_service_;
            rclcpp::Service<GlowLights>::SharedPtr glow_lights_service_;

            
            // Class Functions start here, we particularly have 6 different call back functions.
            // 3 call back functions corresponding to the action server.
            // 3 call back functions corresponding to head/glow/strip light service servers.
            rclcpp_action::GoalResponse action_handle_goal(const rclcpp_action::GoalUUID             &uuid, 
                                                                 shared_ptr<const BlinkLights::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Recieved Goal Request. LMAO WE ARE SO COOKED.");
                (void)uuid;
                (void)goal;

                if (serial_mutex_.try_lock())
                {
                    serial_mutex_.unlock();
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
                else 
                {
                    RCLCPP_INFO(this->get_logger(), "Lights busy, rejecting goal.");
                    return rclcpp_action::GoalResponse::REJECT;
                }

            }
            
            rclcpp_action::CancelResponse action_handle_cancel(const std::shared_ptr<HandleBlinkLights> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
                (void)goal_handle;

                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void action_handle_accepted(const std::shared_ptr<HandleBlinkLights> goal_handle)
            {
                thread([this, goal_handle]()
                {
                    execute_blink_lights(goal_handle);
                }).detach();
            }

            // Service server callback handling. 
            // Mutexes are used to insure serial controller is useable and no other servers are running.
            void strip_lights_callback(const std::shared_ptr<StripLights::Request>  request, 
                                             std::shared_ptr<StripLights::Response> response)
            {
                if (try_lock(serial_mutex_, strip_mutex_) == -1)
                {
                    if (request->on_off) 
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned on led_num:%d, with color: %s on the strip lights.", request->led_num, request->color.c_str());
                    }
                    else 
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned off led_num: %d on the strip lights.", request->led_num);
                    }

                    serial_mutex_.unlock();
                    strip_mutex_.unlock();
                }
                else
                {
                    response->result = false;
                    response->debug_msg = "In Use";
                    RCLCPP_INFO(this->get_logger(), "Strip lights are in use. responding with flase.");
                }
            }

            void head_lights_callback(const std::shared_ptr<HeadLights::Request>  request, 
                                            std::shared_ptr<HeadLights::Response> response)
            {
                if (try_lock(serial_mutex_, head_mutex_) == -1)
                {
                    if (request->on_off) 
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned on headlights with color: %s.", request->color.c_str());
                    }
                    else 
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned off head lights.");
                    }

                    serial_mutex_.unlock();
                    head_mutex_.unlock();
                }
                else
                {
                    response->result = false;
                    response->debug_msg = "In Use";
                    RCLCPP_INFO(this->get_logger(), "Head lights are in use. responding with flase.");
                }
            }
            void glow_lights_callback(const std::shared_ptr<GlowLights::Request>  request, 
                                            std::shared_ptr<GlowLights::Response> response)
            {
                if (try_lock(serial_mutex_, glow_mutex_) == -1)
                {
                    if (request->on_off)
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned on glowlights with brightness:%d", request->brightness);
                    }
                    else
                    {
                        // TODO: CODE GOES HERE

                        response->result = true;
                        response->debug_msg = "Success";

                        RCLCPP_INFO(this->get_logger(), "Turned off glowlights");
                    }

                    serial_mutex_.unlock();
                    glow_mutex_.unlock();
                }
                else
                {
                    response->result = false;
                    response->debug_msg = "In Use";
                    RCLCPP_INFO(this->get_logger(), "Glow lights are in use. responding with flase.");
                }
            }
    
            // Main thread of the action server.
            // Runs for as long as the lights need blinking.
            // Turns off all lights at the end.
            void execute_blink_lights(const std::shared_ptr<HandleBlinkLights> goal_handle)
            {
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<BlinkLights::Feedback>();
                auto result = std::make_shared<BlinkLights::Result>();

                auto &time_remaining = feedback->time_remaining;
                time_remaining = goal->duration;

                rclcpp::Rate loop_rate(goal->rate);

                RCLCPP_INFO(this->get_logger(), "And so it begins. Information about the requested action listed below:");
                RCLCPP_INFO(this->get_logger(), "Flashing rate: %f [Hz]", goal->rate);
                RCLCPP_INFO(this->get_logger(), "Duration: %d [s]", goal->duration);
                RCLCPP_INFO(this->get_logger(), "color: %s [RGB]", goal->color.c_str());
                RCLCPP_INFO(this->get_logger(), "what to flash? Strip: %d, Glow: %d, Head: %d", goal->striplights, goal->glowlights, goal->headlights);
                RCLCPP_INFO(this->get_logger(), "Ready to start, waiting for lights to become available");
    

                // using a scoped lock to use the lights on the rover making sure they are not taken.
                {
                    scoped_lock lock(serial_mutex_, head_mutex_, strip_mutex_, glow_mutex_);
                    
                    while(time_remaining > 0 && rclcpp::ok())
                    {
                        if (goal_handle->is_canceling())
                        {
                            result->finished = false;
                            result->debug_msg = "canceled";
                            goal_handle->canceled(result);
                            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                            return;
                        }

                        this->s_.serial_write(1, 0, "white");
                        // wait(0.25)
                        usleep(250000);
                        this->s_.serial_write(1, 1, "white");

                        // TODO because we have this manually added
                        // sleep so that the nuvoton can pick up both commands
                        // we need to fix the code's time remaining and rate.



                        time_remaining -= 1 / goal->rate;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "Published Feedback");
                        loop_rate.sleep();
                    }


                    this->s_.serial_write(1, 0, "off");
                    // wait(0.25)
                    usleep(250000);
                    this->s_.serial_write(1, 1, "off");

                    // wait(0.25)
                    usleep(250000);
                    // TODO because we have this manually added
                    // sleep so that the nuvoton can pick up both commands
                    // we need to fix the code's time remaining and rate.



                    if (rclcpp::ok())
                    {
                        result->finished = true;
                        result->debug_msg = "hell yea";
                        goal_handle->succeed(result);
                    }
                }
            }
    }; // Class: Lights Controller.
} // Namespace: UGV Peripherals.

RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::LightsController)
