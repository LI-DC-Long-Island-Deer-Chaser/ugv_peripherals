// Include cpp libraries
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <algorithm>

// Include Custom Interfaces that this node depends on
#include "ugv_interfaces/action/blink_lights.hpp"
#include "ugv_interfaces/srv/strip_lights.hpp"
#include "ugv_interfaces/srv/glow_lights.hpp"
#include "ugv_interfaces/srv/head_lights.hpp"

// Include all of the necessary ros client library things
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// Create a class for the LightsNode
class LightsNode : public rclcpp::Node
{
    public:
        // Namespace Translations
        using BlinkLights = ugv_interfaces::action::BlinkLights;
        using GoalHandleBlinkLights = rclcpp_action::ServerGoalHandle<BlinkLights>;
        using StripLights = ugv_interfaces::srv::StripLights;
        using HeadLights = ugv_interfaces::srv::HeadLights;
        using GlowLights = ugv_interfaces::srv::GlowLights;

        // Default Node Initialization
        explicit LightsNode (const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("lights_server", options)
        {
            // Action server for blinking the lights
            action_server_ = rclcpp_action::create_server<BlinkLights>(
                this, 
                "blinklights",
                // Handle Goal Callback
                [this](const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const BlinkLights::Goal> goal)
                {
                    return this->handle_goal(uuid, goal);
                },
                // Handle Cancel Callback
                [this](const std::shared_ptr<GoalHandleBlinkLights> goal_handle)
                {
                    return this->handle_cancel(goal_handle);
                },
                // Handle Accepted Callback
                [this](const std::shared_ptr<GoalHandleBlinkLights> goal_handle)
                {
                    this->handle_accepted(goal_handle);
                }
            );
            // Successful action server init
            RCLCPP_INFO(this->get_logger(), "blinklights Action server inited sucessfully");
            
            // Service to control the StripLights
            strip_lights_service_ = this->create_service<ugv_interfaces::srv::StripLights>
            ("strip_lights_service", 
            [this]
            (const std::shared_ptr<StripLights::Request> request, std::shared_ptr<StripLights::Response> response)
            {this->strip_lights_callback(request, response);});
            RCLCPP_INFO(this->get_logger(), "strip_lights_service sucessfully created");

            // Service to control the HeadLights
            head_lights_service_ = this->create_service<ugv_interfaces::srv::HeadLights>
            ("head_lights_service", 
            [this]
            (const std::shared_ptr<HeadLights::Request> request, std::shared_ptr<HeadLights::Response> response)
            {this->head_lights_callback(request, response);});
            RCLCPP_INFO(this->get_logger(), "head_lights_service sucessfully created");

            // Service to control the GlowLights
            glow_lights_service_ = this->create_service<ugv_interfaces::srv::GlowLights>
            ("glow_lights_service", 
            [this]
            (const std::shared_ptr<GlowLights::Request> request, std::shared_ptr<GlowLights::Response> response)
            {this->glow_lights_callback(request, response);});
            RCLCPP_INFO(this->get_logger(), "glow_lights_service sucessfully created");

            // Start ONE worker thread that will execute goals for the action server
            worker_ = std::thread([this]() { this->worker_loop(); });
        }

    // Destructor: stop and join the worker thread cleanly
    ~LightsNode() override
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            stop_worker_ = true;   // tell worker_loop() to exit
        }
        cv_.notify_one();        // wake worker if it is waiting

        if (worker_.joinable()) {
            worker_.join();        // wait for thread to finish
        }
    }

    private:
        // Variables to ensure that when a new goal arrives, the current goal is cancelled and the new goal is started
        std::mutex m_; // Mutex Variable
        std::condition_variable cv_; 
        std::thread worker_; // Single worker thread for the long running action server
        bool stop_worker_{false}; // Tells the worker thread to exit in the destructor

        // Variables to store the goals
        std::shared_ptr<GoalHandleBlinkLights> pending_goal_; // New Goal
        std::shared_ptr<GoalHandleBlinkLights> active_goal_; // Current Goal

        // Create a shared pointer that keeps the action server alive
        rclcpp_action::Server<BlinkLights>::SharedPtr action_server_;

        // Shared pointers for Service Servers
        rclcpp::Service<StripLights>::SharedPtr strip_lights_service_;
        rclcpp::Service<HeadLights>::SharedPtr head_lights_service_;
        rclcpp::Service<GlowLights>::SharedPtr glow_lights_service_;

        // Function to handle the case when a goal is sent to the action server
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const BlinkLights::Goal> goal
        )
        {
            // Debug message
            RCLCPP_INFO(this->get_logger(), 
            "Recieved goal request with duration: %d, rate: %lf, color: %s, striplights: %d, headlights: %d, glowlights: %d", 
            goal->duration, goal->rate, goal->color.c_str(), goal->striplights, goal->headlights, goal->glowlights
            );
            // Void the unused parameter to ensure the compiler doesnt complain
            (void)uuid;
            // Accept and execute the goal, reject if the request is invalid
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; 
        }

        // Function to handle the case when a cancel request is sent (ctrl c in the server terminal etc)
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleBlinkLights> goal_handle
        )
        {
            // Debug message when there is a cancel request
            RCLCPP_INFO(this->get_logger(), "Received request to cancel blink lights");
            // Avoid unused parameter warning
            (void)goal_handle;
            // Return an accepted cancel response (this will stop the current goal)
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        // Function to handle the case when the goal is accepted and needs to be executed
        void handle_accepted(const std::shared_ptr<GoalHandleBlinkLights> goal_handle) {
            {
                // Lock the mutex
                std::lock_guard<std::mutex> lk(m_);
                pending_goal_ = goal_handle; // Overwrite the older goal
            }
            cv_.notify_one(); // Wake the worker thread
        }

        // Execute the worker thread
        void worker_loop()
        {
            while(rclcpp::ok()) {
                // Create a local copy of the goal handle
                std::shared_ptr<GoalHandleBlinkLights> goal;
                // Wait for a pending goal (or shutdown)
                {
                    // Lock the mutex before doing the work
                    std::unique_lock<std::mutex> lk(m_);
                    
                    // Wait until there is a new pending goal or a stop command has been received
                    cv_.wait(lk, [this](){return stop_worker_ || pending_goal_ != nullptr;});

                    // Get out of the worker loop 
                    if (stop_worker_) {
                    return; // node is shutting down
                    }

                    // Promote pending -> active (this is the "preempt point")
                    goal = pending_goal_;
                    pending_goal_.reset();
                    active_goal_ = goal;
                } // Unlock before doing long work

                // Do the long running work (action)
                execute(goal);

                // Clear active if we're still active (not strictly required, but tidy)
                {
                    // Lock the mutex
                    std::lock_guard<std::mutex> lk(m_);
                    // Clear the poiter to let the state machine know that active_goal 
                    if (active_goal_ == goal) {
                        active_goal_.reset();
                    }
                }
            }
        }

        void execute(const std::shared_ptr<GoalHandleBlinkLights> & goal_handle)
        {
            // Logger
            RCLCPP_INFO(get_logger(), "Executing goal");
            
            // Created shared pointers to store the goal, feedback and results
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<BlinkLights::Feedback>();
            auto result   = std::make_shared<BlinkLights::Result>();
                
            // Variables to hold start and end time
            const auto start_time = this->now();
            const auto end_time   = start_time + rclcpp::Duration::from_seconds(
                static_cast<double>(goal->duration));
            
            // Get the rate from the request (Convert float 64 to double)
            const double hz = static_cast<double>(goal->rate);
            // Should not perform action with this type of request
            if (hz <= 0.0) {
                result->finished = false;
                result->debug_msg = "Invalid rate <= 0";
                goal_handle->abort(result);
                return;
            }

            // Create a rate object
            rclcpp::Rate rate(hz);

            // While ROS is ok and there is still time remaining
            while (rclcpp::ok() && this->now() < end_time) {

                // Cancel check
                if (goal_handle->is_canceling()) {
                result->finished = false;
                result->debug_msg = "Goal cancelled";
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Goal canceled");
                return;
                }

                // Preempt check (with your worker-thread model, use pending_goal_)
                {
                    std::lock_guard<std::mutex> lk(m_);
                    if (pending_goal_) {
                        result->finished = false;
                        result->debug_msg = "Preempted by newer goal";
                        goal_handle->abort(result);
                        RCLCPP_WARN(get_logger(), "Preempting current goal");
                        return;
                    }
                }

                // --- blink step ---
                // send ON opcode
                // ...
                rate.sleep();
                // send OFF opcode
                // ...

                // Feedback (example: seconds remaining as double)
                auto remaining = end_time - this->now();
                feedback->time_remaining = std::max(0.0, remaining.seconds());
                goal_handle->publish_feedback(feedback);
            }

            // If ROS is not okay
            if (!rclcpp::ok()) {
                result->finished = false;
                result->debug_msg = "ROS shutting down";
                goal_handle->abort(result);
                return;
            }
            
            // Successful execution of the node
            result->finished = true;
            result->debug_msg = "Done";
            goal_handle->succeed(result);
        }


        // Service servers callbacks
        // Strip Lights Callback
        void strip_lights_callback(const std::shared_ptr<StripLights::Request> request, std::shared_ptr<StripLights::Response> response)
        {
            // Check if we need to turn on or off the light
            if (request->on_off) {
                // Package and send the hex code
                request->color;
                request->led_num;

                // TODO: OPCODE PACKAGING AND SENDING

                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned on led_num:%d, with color:%s on the strip lights", request->led_num, request->color.c_str());
            }
            else {
                // Send the hex code to turn off said LED

                // TODO: OPCODE PACKAGING AND SENDING
    
                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned off led_num:%d on the strip lights", request->led_num);
            }
        }

        // Head Lights Callback
        void head_lights_callback(const std::shared_ptr<HeadLights::Request> request, std::shared_ptr<HeadLights::Response> response)
        {
            // Check if we need to turn on or off the light
            if (request->on_off) {
                // Package and send the hex code
                request->color;
                
                // TODO: OPCODE PACKAGING AND SENDING


                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned on headlights with color:%s", request->color.c_str());
            }
            else {
                // Send the hex code to turn off the headlights
                
                // TODO: OPCODE PACKAGING AND SENDING

                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned off head lights");
            }
            
        }

        // Glow Lights Callback
        void glow_lights_callback(const std::shared_ptr<GlowLights::Request> request, std::shared_ptr<GlowLights::Response> response)
        {
            // Check if we need to turn on or off the light
            if (request->on_off) {
                // Package and send the hex code
                request->brightness;

                // TODO: OPCODE PACKAGING AND SENDING

                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned on glowlights with brightness:%d", request->brightness);
            }
            else {
                // Send the hex code to turn off the headlights

                // TODO: OPCODE PACKAGING AND SENDING
    
                // Response
                response->result = true;
                response->debug_msg = "Success";

                // Logger
                RCLCPP_INFO(this->get_logger(), "Turned off glowlights");
            }
            
        }
};

// Main function (No Composability added yet)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightsNode>());
  rclcpp::shutdown();
  return 0;
}