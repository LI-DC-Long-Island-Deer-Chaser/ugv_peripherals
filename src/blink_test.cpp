#include <functional>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>

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
    class LightsTestClient : public rclcpp::Node
    {
        public:
            // Action server namespace.
            using BlinkLights = ugv_interfaces::action::BlinkLights;
            using HandleBlinkLights = rclcpp_action::ClientGoalHandle<BlinkLights>;

            using StripLights = ugv_interfaces::srv::StripLights;
            using HeadLights = ugv_interfaces::srv::HeadLights;
            using GlowLights = ugv_interfaces::srv::GlowLights;

            explicit LightsTestClient(const rclcpp::NodeOptions &option = rclcpp::NodeOptions())
            : Node("lights_test_client", option)
            {
                this->client_ptr_ = rclcpp_action::create_client<BlinkLights>(
                    this, 
                    "ugv_peripherals/blink_lights");

                this->timer_ = this->create_wall_timer(
                    chrono::milliseconds(500),
                    bind(&LightsTestClient::send_goal, this));
            }

            void send_goal()
            {
                using namespace placeholders;

                this->timer_->cancel();

                if (!this->client_ptr_->wait_for_action_server())
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
                    rclcpp::shutdown();
                }

                auto goal_msg = BlinkLights::Goal();
                goal_msg.rate = 0.5;   // Hz
                goal_msg.duration = 9; // Seconds
                goal_msg.color = "white";
                goal_msg.striplights = 1;
                goal_msg.headlights = 1;
                goal_msg.glowlights = 0;

                RCLCPP_INFO(this->get_logger(), "Sending goal.");

                auto send_goal_options = rclcpp_action::Client<BlinkLights>::SendGoalOptions();

                send_goal_options.goal_response_callback = bind(&LightsTestClient::goal_response_callback, this, _1);
                send_goal_options.feedback_callback = bind(&LightsTestClient::feedback_callback, this, _1, _2);
                send_goal_options.result_callback = bind(&LightsTestClient::result_callback, this, _1);

                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            }

        private:
            rclcpp_action::Client<BlinkLights>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::Client<HeadLights>::SharedPtr head_lights_client_;
            rclcpp::Client<StripLights>::SharedPtr strip_lights_client_;
            rclcpp::Client<GlowLights>::SharedPtr glow_lights_client_;

            void goal_response_callback(const HandleBlinkLights::SharedPtr &goal_handle)
            {
                if (!goal_handle) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } 
                else 
                {
                  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            }

            void feedback_callback(HandleBlinkLights::SharedPtr, 
                                   const std::shared_ptr<const BlinkLights::Feedback> feedback)
            {
                std::stringstream ss;
                ss << "Time remaining until blinking will finish: " << feedback->time_remaining;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            }

            void result_callback(const HandleBlinkLights::WrappedResult &result)
            {
                switch (result.code) 
                {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        break;

                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        return;

                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                        return;

                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        return;
                }
                std::stringstream ss;
                ss << "Result received: finished: " << result.result->finished << "with debug message: " << result.result->debug_msg;

                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                RCLCPP_INFO(this->get_logger(), "TEST PASSED");
                rclcpp::shutdown();
            }
    }; // Class: Lights Controller.
} // Namespace: UGV Peripherals.

RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::LightsTestClient)
