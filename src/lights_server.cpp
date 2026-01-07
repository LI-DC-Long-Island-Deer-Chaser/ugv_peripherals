// Include cpp libraries
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

// Include rclcpp and the custom action server interfaces
#include "ugv_interfaces/action/blink_lights.hpp"
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

        // Default Node Initialization
        explicit LightsNode (const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("lights_server", options)
        {
            this->action_server_ = rclcpp_action::create_server<BlinkLights>(
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

            // Start ONE worker thread that will execute goals sequentially
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

        // Function to handle the case when a goal is sent to the action server
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid, 
            std::shared_ptr<const BlinkLights::Goal> goal
        )
        {
            // Debug message
            RCLCPP_INFO(this->get_logger(), 
            "Recieved goal request with period: %d, duration: %d, color: %s, striplights: %d, headlights: %d, glowlights: %d", 
            goal->period, goal->duration, goal->color, goal->striplights, goal->headlights, goal->glowlights
            )
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

        void execute()
        {
            // Print Log Message
            RCLCPP_INFO(this->get_logger(), "Executing goal (placeholder)");
           
            // auto feedback = std::make_shared<BlinkLights::Feedback>();
            // auto result   = std::make_shared<BlinkLights::Result>();

            // Skeleton loop: keep it responsive to cancel + preempt
            while (rclcpp::ok()) {

            // 1) Cancel check (client requested cancel)
            if (goal_handle->is_canceling()) {
                // TODO:
                // result->... = ...
                // goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled (placeholder)");
                return;
            }

            // 2) Preempt check: if a newer goal became active, stop this one
            {
                std::lock_guard<std::mutex> lk(m_);
                if (active_goal_ != goal_handle) {
                // TODO:
                // goal_handle->abort(result);
                RCLCPP_WARN(this->get_logger(), "Goal preempted by newer goal (placeholder)");
                return;
                }
            }

            // 3) ---- YOUR ACTION WORK GOES HERE ----
            // - blink lights / sleep / publish feedback / check completion
            //
            // Example structure:
            //   do_one_step();
            //   goal_handle->publish_feedback(feedback);
            //   if (done) { goal_handle->succeed(result); return; }

            // Placeholder so this doesn't busy-spin if you test it accidentally
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            // If ROS is shutting down, you can abort/return.
        }
};