#include <memory>
#include <vector>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include <crazyflie_interfaces/srv/notify_setpoints_stop.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"


#include <Eigen/Geometry>

using std::placeholders::_1;

using std_srvs::srv::Empty;
using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::NotifySetpointsStop;
using crazyflie_interfaces::msg::FullState;

using namespace std::chrono_literals;
using namespace Eigen;

namespace Xbox360Buttons {

    enum { Green = 0,
           Red = 1,
           Blue = 2,
           Yellow = 3,
           LB = 4,
           RB = 5,
           Back = 6,
           Start = 7,
           COUNT = 8,
    };
}

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
        : Node("teleop")
    {
        
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 1, std::bind(&TeleopNode::joyChanged, this, _1));

        pub_cmd_vel_legacy_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_legacy", 10);
        pub_cmd_full_state_ = this->create_publisher<crazyflie_interfaces::msg::FullState>("cmd_full_state", 10);

        this->declare_parameter("frequency", 0);
        this->get_parameter<int>("frequency", frequency_);
        this->declare_parameter("mode", "default");
        this->get_parameter<std::string>("mode", mode_);
        this->declare_parameter("auto_yaw_rate", 0.0);
        this->get_parameter<double>("auto_yaw_rate", auto_yaw_rate_);

        this->declare_parameter<float>("initial_position.x");
        this->get_parameter<float>("initial_position.x", state_.x);
        this->declare_parameter<float>("initial_position.y");
        this->get_parameter<float>("initial_position.y", state_.y);
        this->declare_parameter<float>("initial_position.z");
        this->get_parameter<float>("initial_position.z", state_.z);

        // declare cmd_rpy params
        declareAxis("cmd_rpy.roll");
        declareAxis("cmd_rpy.pitch");
        declareAxis("cmd_rpy.yawrate");
        declareAxis("cmd_rpy.thrust");

        // declare cmd_vel_world params
        declareAxis("cmd_vel_world.x_velocity");
        declareAxis("cmd_vel_world.y_velocity");
        declareAxis("cmd_vel_world.z_velocity");
        declareAxis("cmd_vel_world.yaw_velocity");
        this->declare_parameter("cmd_vel_world.x_limit", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("cmd_vel_world.y_limit", rclcpp::PARAMETER_DOUBLE_ARRAY);
        this->declare_parameter("cmd_vel_world.z_limit", rclcpp::PARAMETER_DOUBLE_ARRAY);

        on_mode_switched();

        dt_ = 1.0f/frequency_;
        is_low_level_flight_active_ = false;

        // Create a parameter subscriber that can be used to monitor parameter changes
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        cb_handle_ = param_subscriber_->add_parameter_event_callback(std::bind(&TeleopNode::on_parameter_event, this, _1));

        if (frequency_ > 0) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), std::bind(&TeleopNode::publish, this));
        }

        client_emergency_ = this->create_client<Empty>("emergency");
        client_takeoff_ = this->create_client<Takeoff>("takeoff");
        client_land_ = this->create_client<Land>("land");
        client_notify_setpoints_stop_ = this->create_client<NotifySetpointsStop>("notify_setpoints_stop");
    }

private:
    struct 
    {
        float x;
        float y;
        float z;
        float yaw;
    }state_;

    struct Axis
    { 
        int axis;
        float max;
        float deadband;
    };
    struct
    {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    float angle_normalize(float a){ 
        a = fmod(a, 2*M_PI);
        a = fmod((a + 2*M_PI),2*M_PI);
        if (a > M_PI){
            a -= 2*M_PI;
        }
        return a;
    }

    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event)
    {
        if (event.node == "/teleop") {
            auto params = param_subscriber_->get_parameters_from_event(event);
            for (auto &p : params) {
                if (p.get_name() == "mode") {
                    mode_ = p.as_string();
                    on_mode_switched();
                }
            }
        }
    }

    void publish() 
    {
        if (!is_low_level_flight_active_) {
            return;
        }

        if (mode_ == "cmd_rpy") { 
            pub_cmd_vel_legacy_->publish(twist_);
        }
        if (mode_ == "cmd_vel_world") {   

            float prev_x = state_.x;
            float prev_y = state_.y;
            float prev_z = state_.z;
            state_.x = std::min<float>(std::max<float>(state_.x + twist_.linear.x*dt_, x_limit_[0]), x_limit_[1]);
            state_.y = std::min<float>(std::max<float>(state_.y + twist_.linear.y*dt_, y_limit_[0]), y_limit_[1]);
            state_.z = std::min<float>(std::max<float>(state_.z + twist_.linear.z*dt_, z_limit_[0]), z_limit_[1]);
            state_.yaw = angle_normalize(state_.yaw + twist_.angular.z*dt_);

            Quaternionf q;
            q = AngleAxisf(0, Vector3f::UnitX())
                    * AngleAxisf(0, Vector3f::UnitY())
                    * AngleAxisf(state_.yaw, Vector3f::UnitZ());

            fullstate_.pose.position.x = state_.x; 
            fullstate_.pose.position.y = state_.y;
            fullstate_.pose.position.z = state_.z;
            fullstate_.twist.linear.x = (state_.x-prev_x)/dt_;
            fullstate_.twist.linear.y = (state_.y-prev_y)/dt_;
            fullstate_.twist.linear.z = (state_.z-prev_z)/dt_;
            fullstate_.acc.x = 0;
            fullstate_.acc.y = 0;
            fullstate_.acc.z = 0;
            fullstate_.pose.orientation.x = q.x(); 
            fullstate_.pose.orientation.y = q.y();
            fullstate_.pose.orientation.z = q.z();
            fullstate_.pose.orientation.w = q.w();
            fullstate_.twist.angular.x = 0;       
            fullstate_.twist.angular.y = 0;       
            fullstate_.twist.angular.z = twist_.angular.z; // yaw rate 
            
            pub_cmd_full_state_->publish(fullstate_);
        }

    }

    void joyChanged(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        static std::vector<int> lastButtonState(Xbox360Buttons::COUNT);

        if (msg->buttons.size() >= Xbox360Buttons::COUNT && lastButtonState.size() >= Xbox360Buttons::COUNT) {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Start] == 1 && lastButtonState[Xbox360Buttons::Start] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Back] == 1 && lastButtonState[Xbox360Buttons::Back] == 0) {
                land();
            }
        }

        lastButtonState = msg->buttons;
        
        twist_.linear.x = getAxis(msg, axes_.x);
        twist_.linear.y = getAxis(msg, axes_.y);
        twist_.linear.z = getAxis(msg, axes_.z);
        if(auto_yaw_rate_ == 0.0){
            twist_.angular.z = getAxis(msg, axes_.yaw);
        }
        else {
            twist_.angular.z = auto_yaw_rate_; 
        }
        
    }

    sensor_msgs::msg::Joy::_axes_type::value_type getAxis(const sensor_msgs::msg::Joy::SharedPtr &msg, Axis a)
    {
        if (a.axis == 0) {
            return 0;
        }

        sensor_msgs::msg::Joy::_axes_type::value_type sign = 1.0;
        if (a.axis < 0) {
            sign = -1.0;
            a.axis = -a.axis;
        }
        if ((size_t) a.axis > msg->axes.size()) {
            return 0;
        }
        auto result = sign * msg->axes[a.axis - 1]*a.max;
        if (fabs(result) > a.deadband) {
            return result;
        } else {
            return 0;
        }
    }
    
    void emergency()
    {
        if (!client_emergency_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Emergency service not ready!");
            return;
        }
        auto request = std::make_shared<Empty::Request>();
        client_emergency_->async_send_request(request);
    }

    void takeoff()
    {
        if (!client_takeoff_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Takeoff service not ready!");
            return;
        }
        auto request = std::make_shared<Takeoff::Request>();
        request->group_mask = 0;
        request->height = 0.5;
        request->duration = rclcpp::Duration::from_seconds(2);
        client_takeoff_->async_send_request(request);

        timer_takeoff_ = this->create_wall_timer(2s, [this]() {
            state_.z = 0.5;
            is_low_level_flight_active_ = true;
            this->timer_takeoff_->cancel();
        });
    }

    void land()
    {
        if (!client_land_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "Land service not ready!");
            return;
        }

        is_low_level_flight_active_ = false;

        // If we are in manual flight mode, first switch back to high-level mode
        if (!client_notify_setpoints_stop_->service_is_ready()) {
            RCLCPP_ERROR(get_logger(), "NotifySetpointStop service not ready!");
        } else {
            auto request1 = std::make_shared<NotifySetpointsStop::Request>();
            request1->remain_valid_millisecs = 100;
            request1->group_mask = 0;
            client_notify_setpoints_stop_->async_send_request(request1);
        }

        // Now we should be able to land!
        auto request2 = std::make_shared<Land::Request>();
        request2->group_mask = 0;
        request2->height = 0.0;
        request2->duration = rclcpp::Duration::from_seconds(3.5);
        client_land_->async_send_request(request2);
    }

    void declareAxis(const std::string& name)
    {
        this->declare_parameter<int>(name + ".axis");
        this->declare_parameter<float>(name + ".max");
        this->declare_parameter<float>(name + ".deadband");
    }

    void getAxis(const std::string& name, Axis& axis)
    {
        this->get_parameter<int>(name + ".axis", axis.axis);
        this->get_parameter<float>(name + ".max", axis.max);
        this->get_parameter<float>(name + ".deadband", axis.deadband);
    }

    void on_mode_switched()
    {
        if (mode_ == "high_level") {
            // nothing to do
        } else if (mode_ == "cmd_rpy") {
            getAxis(mode_ + ".roll", axes_.x);
            getAxis(mode_ + ".pitch", axes_.y);
            getAxis(mode_ + ".yawrate", axes_.yaw);
            getAxis(mode_ + ".thrust", axes_.z);
        } else if (mode_ == "cmd_vel_world"){
            getAxis(mode_ + ".x_velocity", axes_.x);
            getAxis(mode_ + ".y_velocity", axes_.y);
            getAxis(mode_ + ".z_velocity", axes_.z);
            getAxis(mode_ + ".yaw_velocity", axes_.yaw);

            this->get_parameter(mode_ + ".x_limit", x_param);
            x_limit_ = x_param.as_double_array();
            this->get_parameter(mode_ + ".y_limit", y_param);
            y_limit_ = y_param.as_double_array();
            this->get_parameter(mode_ + ".z_limit", z_param);
            z_limit_ = z_param.as_double_array();
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown mode %s", mode_.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Mode changed to %s", mode_.c_str());
    }

    // monitor parameter changes
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_emergency_;
    rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
    rclcpp::Client<Land>::SharedPtr client_land_;
    rclcpp::Client<NotifySetpointsStop>::SharedPtr client_notify_setpoints_stop_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_legacy_;
    rclcpp::Publisher<crazyflie_interfaces::msg::FullState>::SharedPtr pub_cmd_full_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_takeoff_;
    geometry_msgs::msg::Twist twist_;
    crazyflie_interfaces::msg::FullState fullstate_;
    std::vector<double> x_limit_;
    std::vector<double> y_limit_;
    std::vector<double> z_limit_;
    std::string mode_;
    rclcpp::Parameter x_param;
    rclcpp::Parameter y_param;
    rclcpp::Parameter z_param;
    int frequency_;
    float dt_;
    bool is_low_level_flight_active_;
    double auto_yaw_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
