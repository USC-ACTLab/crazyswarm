#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "crazyswarm2_interfaces/srv/takeoff.hpp"
#include "crazyswarm2_interfaces/srv/land.hpp"

using std::placeholders::_1;

using std_srvs::srv::Empty;
using crazyswarm2_interfaces::srv::Takeoff;
using crazyswarm2_interfaces::srv::Land;

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

        client_emergency_ = this->create_client<Empty>("emergency");
        client_emergency_->wait_for_service();

        client_takeoff_ = this->create_client<Takeoff>("takeoff");
        client_takeoff_->wait_for_service();

        client_land_ = this->create_client<Land>("land");
        client_land_->wait_for_service();
    }

private:
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
    }

    void emergency()
    {
        RCLCPP_INFO(this->get_logger(), "emergency requested...");
        auto request = std::make_shared<Empty::Request>();
        auto result = client_emergency_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
        RCLCPP_INFO(this->get_logger(), "Done.");
    }

    void takeoff()
    {
        auto request = std::make_shared<Takeoff::Request>();
        request->group_mask = 0;
        request->height = 0.5;
        request->duration = rclcpp::Duration::from_seconds(2);
        auto result = client_takeoff_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    }

    void land()
    {
        auto request = std::make_shared<Land::Request>();
        request->group_mask = 0;
        request->height = 0.0;
        request->duration = rclcpp::Duration::from_seconds(3.5);
        auto result = client_land_->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_emergency_;
    rclcpp::Client<Takeoff>::SharedPtr client_takeoff_;
    rclcpp::Client<Land>::SharedPtr client_land_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
