#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <string>

#include "ugv_interfaces/action/blink_lights.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;

namespace ugv_peripherals
{
    class LightsController : public rclcpp::Node
    {
        public:
            using BlinkLights = ugv_interfaces::action::BlinkLights;
            using HandleBlinkLights = rclcpp_action::ServerGoalHandle<BlinkLights>;

            explicit LightsController(const rclcpp::NodeOptions &option = rclcpp::NodeOptions()) : Node("lights_controller", option)
            {
                using namespace placeholders;

                this->action_server_ = 
                    rclcpp_action::create_server<BlinkLights>
                    (
                        this,
                        "blink_lights",
                        bind(&LightsController::handle_goal, this, _1, _2),
                        bind(&LightsController::handle_cancel, this, _1),
                        bind(&LightsController::handle_accepted, this, _1)
                    );
            }

        private:
            mutex light_mutex_;
            rclcpp_action::Server<BlinkLights>::SharedPtr action_server_;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, shared_ptr<const BlinkLights::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Recieved Goal Request. LMAO WE ARE SO COOKED.");
                (void)uuid;
                (void)goal;

                if (light_mutex_.try_lock())
                {
                    light_mutex_.unlock();
                    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
                }
                else 
                {
                    RCLCPP_INFO(this->get_logger(), "Lights busy, rejecting goal.");
                    return rclcpp_action::GoalResponse::REJECT;
                }

            }
            
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<HandleBlinkLights> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
                (void)goal_handle;

                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_accepted(const std::shared_ptr<HandleBlinkLights> goal_handle)
            {
                thread([this, goal_handle]()
                {
                    execute_blink_lights(goal_handle);
                }).detach();
            }

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
                    scoped_lock lock(light_mutex_);
                    
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

                        // Flash them titties.
                        
                        time_remaining -= 1 / goal->rate;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "Published Feedback");
                        loop_rate.sleep();
                    }

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
