// C++ standard libraries and other builtin include
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

// ROS2 standard interfaces
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// custom messages
#include "ugv_interfaces/msg/yolo_status.hpp"

// custom actions
#include "ugv_interfaces/action/play_speakers.hpp"
#include "ugv_interfaces/action/blink_lights.hpp"

/*
*
*       What: Deterrence system that synchronizes speakers and lights, triggering them on 
*                 detection of a deer.
*
*       How: Node subscribes to /yolo_anomaly_angle topic and sends a goal to start the process for deterring when a threshold is crossed.
*                The first goal sent is to the speaker action server. That goal handle returns the length of the audio.
*                The next and last goal is to the blink lights action server, which uses the length of the audio to determine duration of blinking.
*                
*       Relevant Concepts: 
*                 * topic: /yolo_anomaly_angle (YoloStatus.msg)
*                 * action servers: speaker_action_server, blink_lights_action_server
*
*/

namespace ugv_peripherals
{
        class DeterrenceActionClient : public rclcpp::Node
        {
        public:
                using PlaySpeakers = ugv_interfaces::action::PlaySpeakers;
                using GoalHandlePlaySpeakers = rclcpp_action::ClientGoalHandle<PlaySpeakers>;
                using BlinkLights = ugv_interfaces::action::BlinkLights;
                using GoalHandleBlinkLights = rclcpp_action::ClientGoalHandle<BlinkLights>;
                using YoloStatus = ugv_interfaces::msg::YoloStatus; 

                explicit DeterrenceActionClient(const rclcpp::NodeOptions &deterrence_options = rclcpp::NodeOptions())
                : Node("deterrence_action_client", deterrence_options)
                {

                        this->speakers_client_ptr_ = rclcpp_action::create_client<PlaySpeakers>(
                                this,
                                "speaker_action_server");

                        this->blink_lights_client_ptr_ = rclcpp_action::create_client<BlinkLights>(
                                this,
                                "blink_lights_action_server");

                        // safety check confirming action servers are set up 
                        if (!this->speakers_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
                        { 
                                RCLCPP_ERROR(this->get_logger(), "Speakers action server not available after waiting."); 
                                rclcpp::shutdown(); 
                                return; 
                        }
                        if (!this->blink_lights_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
                        { 
                                RCLCPP_ERROR(this->get_logger(), "Lights action server not available after waiting."); 
                                rclcpp::shutdown(); 
                                return; 
                        }

                        detection_subscription_ = this->create_subscription<YoloStatus>(
                        "yolo_anomaly_angle",                           // topic name
                        10,                                             // QoS (queue size)
                        std::bind(&DeterrenceActionClient::anomaly_callback, this, std::placeholders::_1)
                        );

                        threshold = 0.5; 
                        feedback_forwarding = false;
                        speakers_done = true;
                }

        private:
                rclcpp_action::Client<PlaySpeakers>::SharedPtr speakers_client_ptr_;
                rclcpp_action::Client<BlinkLights>::SharedPtr blink_lights_client_ptr_;
                rclcpp::Subscription<YoloStatus>::SharedPtr detection_subscription_;

                float threshold;                                                                        // confidence level to exceed before deterrence system activates
                bool feedback_forwarding;                                                               // synchronizes audio length and blinking duration
                bool speakers_done;

                void anomaly_callback(const YoloStatus msg)
                {
                if(msg.confidence[0] > threshold && speakers_done)                                      // if confident that there is an anomaly and no deterrence system playing
                {
                        send_goal();
                }
                        return; 
                }

                void send_goal()
                {
                        using namespace std::placeholders;

                        speakers_done = false;

                        auto speakers_goal = PlaySpeakers::Goal();                                                                                              // build goal message

                        auto speakers_goal_options = rclcpp_action::Client<PlaySpeakers>::SendGoalOptions();    // options object controls lifecycle of the goal 

                        speakers_goal_options.goal_response_callback = std::bind(&DeterrenceActionClient::goal_response_callback, this, _1);
                        speakers_goal_options.feedback_callback = std::bind(&DeterrenceActionClient::feedback_callback, this, _1, _2);
                        speakers_goal_options.result_callback = std::bind(&DeterrenceActionClient::result_callback, this, _1);

                        this->speakers_client_ptr_->async_send_goal(speakers_goal, speakers_goal_options);              // this actually sends the goal
                }

                void goal_response_callback(const GoalHandlePlaySpeakers::SharedPtr &goal_handle)
                {
                        if (!goal_handle)                                                                                       // the goal was rejected by the server
                        {
                                RCLCPP_ERROR(this->get_logger(), "Goal rejected.");
                                return;
                        }
                        else                                                                                                            // the goal was accepted by the server 
                        {
                                RCLCPP_INFO(this->get_logger(), "Goal accepted");
                        }
                }

                void feedback_callback(
                        GoalHandlePlaySpeakers::SharedPtr,
                        const std::shared_ptr<const PlaySpeakers::Feedback> feedback)
                {
                        // feedback from speakers goal used to synchronize with time duration of blinking lights
                        if (!feedback_forwarding)
                        {
                                feedback_forwarding = true;

                                auto blinking_lights_goal = BlinkLights::Goal();
                                blinking_lights_goal.duration = feedback->time_remaining;
                                blinking_lights_goal.color = "white";

                                this->blink_lights_client_ptr_->async_send_goal(blinking_lights_goal);
                        }

                        RCLCPP_INFO(this->get_logger(), "Time remaining: %.2f seconds", feedback->time_remaining);
                }

                void result_callback(const GoalHandlePlaySpeakers::WrappedResult &result)
                {
                        switch (result.code) {
                                case rclcpp_action::ResultCode::SUCCEEDED:
                                        RCLCPP_INFO(this->get_logger(), "Playback finished successfully: %s.",
                                                    result.result->debug_msg.c_str());
                                        break;

                                case rclcpp_action::ResultCode::CANCELED:
                                        RCLCPP_ERROR(this->get_logger(), "Playback canceled.");
                                        break;

                                case rclcpp_action::ResultCode::ABORTED:
                                        RCLCPP_ERROR(this->get_logger(), "Playback aborted.");
                                        break;

                                default:
                                        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                                        break;
                        }

                        speakers_done = true;
                        feedback_forwarding = false;
                }

        };
}
// register the node with ROS2 components
RCLCPP_COMPONENTS_REGISTER_NODE(ugv_peripherals::DeterrenceActionClient)
