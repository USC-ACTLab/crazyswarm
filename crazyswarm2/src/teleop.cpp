#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "crazyswarm2_interfaces/srv/takeoff.hpp"
#include "crazyswarm2_interfaces/srv/land.hpp"
#include "geometry_msgs/msg/twist.hpp"


using std::placeholders::_1;

using std_srvs::srv::Empty;
using crazyswarm2_interfaces::srv::Takeoff;
using crazyswarm2_interfaces::srv::Land;

using namespace std::chrono_literals;


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

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        this->declare_parameter<int>("frequency", 100);
        frequency_ = this->get_parameter("frequency").as_int();

        this->declare_parameter<int>("x_axis", 5);
        axes_.x.axis = this->get_parameter("x_axis").as_int();
        this->declare_parameter<int>("y_axis", 4);
        axes_.y.axis = this->get_parameter("y_axis").as_int();
        this->declare_parameter<int>("z_axis", 2);
        axes_.z.axis = this->get_parameter("z_axis").as_int();
        this->declare_parameter<int>("yaw_axis", 1);
        axes_.yaw.axis = this->get_parameter("yaw_axis").as_int();


        this->declare_parameter<double>("x_velocity_max", 30.0);
        axes_.x.max = this->get_parameter("x_velocity_max").as_double();
        this->declare_parameter<double>("y_velocity_max", -30.0);
        axes_.y.max = this->get_parameter("y_velocity_max").as_double();
        this->declare_parameter<double>("z_velocity_max", 60000.0);
        axes_.z.max = this->get_parameter("z_velocity_max").as_double();

        // this->declare_parameter<double>("yaw_velocity_max", 90.0 * M_PI / 180.0);
        this->declare_parameter<double>("yaw_velocity_max", -200.0);
        axes_.yaw.max = this->get_parameter("yaw_velocity_max").as_double();

        if (frequency_ > 0) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), std::bind(&TeleopNode::publish, this));
        }

        client_emergency_ = this->create_client<Empty>("emergency");
        client_emergency_->wait_for_service();

        client_takeoff_ = this->create_client<Takeoff>("takeoff");
        client_takeoff_->wait_for_service();

        client_land_ = this->create_client<Land>("land");
        client_land_->wait_for_service();
        
    }

private:
    
    struct Axis
    { 
        int axis;
        double max;
    };
    struct
    {
        Axis x;
        Axis y;
        Axis z;
        Axis yaw;
    } axes_;

    void publish() 
    {
        publisher_->publish(twist_);
            
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
        twist_.angular.z = getAxis(msg, axes_.yaw);
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
        return sign * msg->axes[a.axis - 1]*a.max;
    }

    
    void emergency()
    {
        auto request = std::make_shared<Empty::Request>();
        client_emergency_->async_send_request(request);
    }

    void takeoff()
    {
        auto request = std::make_shared<Takeoff::Request>();
        request->group_mask = 0;
        request->height = 0.5;
        request->duration = rclcpp::Duration::from_seconds(2);
        client_takeoff_->async_send_request(request);
    }

    void land()
    {
        auto request = std::make_shared<Land::Request>();
        request->group_mask = 0;
        request->height = 0.0;
        request->duration = rclcpp::Duration::from_seconds(3.5);
        client_land_->async_send_request(request);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_emergency_;
    rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
    rclcpp::Client<Land>::SharedPtr client_land_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_;
    int frequency_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
