#include <memory>
#include <vector>
#include <regex>

#include <crazyflie_cpp/Crazyflie.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "std_srvs/srv/empty.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using crazyflie_interfaces::srv::StartTrajectory;
using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::UploadTrajectory;
using crazyflie_interfaces::srv::NotifySetpointsStop;
using std_srvs::srv::Empty;

using motion_capture_tracking_interfaces::msg::NamedPoseArray;
using crazyflie_interfaces::msg::FullState;

// Helper class to convert crazyflie_cpp logging messages to ROS logging messages
class CrazyflieLogger : public Logger
{
public:
  CrazyflieLogger(rclcpp::Logger logger)
      : Logger()
      , logger_(logger)
  {
  }

  virtual ~CrazyflieLogger() {}

  virtual void info(const std::string &msg)
  {
    RCLCPP_INFO(logger_, "%s", msg.c_str());
  }

  virtual void warning(const std::string &msg)
  {
    RCLCPP_WARN(logger_, "%s", msg.c_str());
  }

  virtual void error(const std::string &msg)
  {
    RCLCPP_ERROR(logger_, "%s", msg.c_str());
  }
private:
  rclcpp::Logger logger_;
};

std::set<std::string> extract_names(
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
    const std::string &pattern)
{
  std::set<std::string> result;
  for (const auto &i : parameter_overrides)
  {
    if (i.first.find(pattern) == 0)
    {
      size_t start = pattern.size() + 1;
      size_t end = i.first.find(".", start);
      result.insert(i.first.substr(start, end - start));
    }
  }
  return result;
}

// ROS wrapper for a single Crazyflie object
class CrazyflieROS
{
private:
  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

  struct logScan {
    uint16_t front;
    uint16_t left;
    uint16_t back;
    uint16_t right;
  } __attribute__((packed));

public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& cf_type,
    const std::string& name,
    rclcpp::Node* node,
    bool enable_parameters = true)
    : logger_(rclcpp::get_logger(name))
    , cf_logger_(logger_)
    , cf_(
      link_uri,
      cf_logger_,
      std::bind(&CrazyflieROS::on_console, this, std::placeholders::_1))
    , name_(name)
    , node_(node)
    , tf_broadcaster_(node)
  {
    service_emergency_ = node->create_service<Empty>(name + "/emergency", std::bind(&CrazyflieROS::emergency, this, _1, _2));
    service_start_trajectory_ = node->create_service<StartTrajectory>(name + "/start_trajectory", std::bind(&CrazyflieROS::start_trajectory, this, _1, _2));
    service_takeoff_ = node->create_service<Takeoff>(name + "/takeoff", std::bind(&CrazyflieROS::takeoff, this, _1, _2));
    service_land_ = node->create_service<Land>(name + "/land", std::bind(&CrazyflieROS::land, this, _1, _2));
    service_go_to_ = node->create_service<GoTo>(name + "/go_to", std::bind(&CrazyflieROS::go_to, this, _1, _2));
    service_upload_trajectory_ = node->create_service<UploadTrajectory>(name + "/upload_trajectory", std::bind(&CrazyflieROS::upload_trajectory, this, _1, _2));
    service_notify_setpoints_stop_ = node->create_service<NotifySetpointsStop>(name + "/notify_setpoints_stop", std::bind(&CrazyflieROS::notify_setpoints_stop, this, _1, _2));

    subscription_cmd_vel_legacy_ = node->create_subscription<geometry_msgs::msg::Twist>(name + "/cmd_vel_legacy", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_vel_legacy_changed, this, _1));
    subscription_cmd_full_state_ = node->create_subscription<crazyflie_interfaces::msg::FullState>(name + "/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_full_state_changed, this, _1));
    subscription_cmd_position_ = node->create_subscription<crazyflie_interfaces::msg::Position>(name + "/cmd_position", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieROS::cmd_position_changed, this, _1));

    auto start = std::chrono::system_clock::now();

    cf_.logReset();

    auto node_parameters_iface = node->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    // declares lambda, to be used as local function, which re-declares specified parameters for other nodes to query
    auto declare_param = [&parameter_overrides, node](const std::string& param)
    {
      // rclcpp::ParameterValue value(parameter_overridesparam]);
      node->declare_parameter(param, parameter_overrides.at(param));
    };
    declare_param("robots." + name + ".uri");
    declare_param("robots." + name + ".initial_position");

    // declares a lambda, to be used as local function
    auto update_map = [&parameter_overrides](std::map<std::string, rclcpp::ParameterValue>& map, const std::string& pattern)
    {
      for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
          size_t start = pattern.size() + 1;
          const auto group_and_name = i.first.substr(start);
          map[group_and_name] = i.second;
        }
      }
    };

    if (enable_parameters) {
      bool query_all_values_on_connect = node->get_parameter("firmware_params.query_all_values_on_connect").get_parameter_value().get<bool>();

      int numParams = 0;
      RCLCPP_INFO(logger_, "Requesting parameters...");
      cf_.requestParamToc(/*forceNoCache*/false, /*requestValues*/query_all_values_on_connect);
      for (auto iter = cf_.paramsBegin(); iter != cf_.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = name + ".params." + entry.group + "." + entry.name;
        switch (entry.type)
        {
        case Crazyflie::ParamTypeUint8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt8:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int8_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<uint16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt16:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int16_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeUint32:
          if (query_all_values_on_connect) {
            node->declare_parameter<int64_t>(paramName, cf_.getParam<uint32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeInt32:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<int32_t>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
          }
          break;
        case Crazyflie::ParamTypeFloat:
          if (query_all_values_on_connect) {
            node->declare_parameter(paramName, cf_.getParam<float>(entry.id));
          } else {
            node->declare_parameter(paramName, rclcpp::PARAMETER_DOUBLE);
          }
          break;
        default:
          RCLCPP_WARN(logger_, "Unknown param type for %s/%s", entry.group.c_str(), entry.name.c_str());
          break;
        }
        // If there is no such parameter in all, add it
        std::string allParamName = "all.params." + entry.group + "." + entry.name;
        if (!node->has_parameter(allParamName)) {
          if (entry.type == Crazyflie::ParamTypeFloat) {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_DOUBLE);
          } else {
            node->declare_parameter(allParamName, rclcpp::PARAMETER_INTEGER);
          }
        }
        ++numParams;
      }
      auto end1 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds1 = end1 - start;
      RCLCPP_INFO(logger_, "reqParamTOC: %f s (%d params)", elapsedSeconds1.count(), numParams);
      
      // Set parameters as specified in the configuration files, as in the following order
      // 1.) check all/firmware_params
      // 2.) check robot_types/<type_name>/firmware_params
      // 3.) check robots/<robot_name>/firmware_params
      // where the higher order is used if defined on multiple levels.

      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> set_param_map;

      // check global settings/firmware_params
      update_map(set_param_map, "all.firmware_params");
      // check robot_types/<type_name>/firmware_params
      update_map(set_param_map, "robot_types." + cf_type + ".firmware_params");
      // check robots/<robot_name>/firmware_params
      update_map(set_param_map, "robots." + name_ + ".firmware_params");

      // Update parameters
      for (const auto&i : set_param_map) {
        std::string paramName = name + ".params." + std::regex_replace(i.first, std::regex("\\."), ".");
        change_parameter(rclcpp::Parameter(paramName, i.second));
      }
    }

    // Logging
    {
      // <group>.<name> -> value map
      std::map<std::string, rclcpp::ParameterValue> log_config_map;

      // Get logging configuration as specified in the configuration files, as in the following order
      // 1.) check all/firmware_logging
      // 2.) check robot_types/<type_name>/firmware_logging
      // 3.) check robots/<robot_name>/firmware_logging
      // where the higher order is used if defined on multiple levels.
      update_map(log_config_map, "all.firmware_logging");
      // check robot_types/<type_name>/firmware_logging
      update_map(log_config_map, "robot_types." + cf_type + ".firmware_logging");
      // check robots/<robot_name>/firmware_logging
      update_map(log_config_map, "robots." + name_ + ".firmware_logging");

      // check if logging is enabled for this drone
      bool logging_enabled = log_config_map["enabled"].get<bool>();
      if (logging_enabled) {
        cf_.requestLogToc(/*forceNoCache*/);

        for (const auto&i : log_config_map) {
          // check if any of the default topics are enabled
          if (i.first.find("default_topics.pose") == 0) {
            int freq = log_config_map["default_topics.pose.frequency"].get<int>();
            RCLCPP_INFO(logger_, "Logging to /pose at %d Hz", freq);

            publisher_pose_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name + "/pose", 10);

            std::function<void(uint32_t, const logPose*)> cb = std::bind(&CrazyflieROS::on_logging_pose, this, std::placeholders::_1, std::placeholders::_2);

            log_block_pose_.reset(new LogBlock<logPose>(
              &cf_,{
                {"stateEstimate", "x"},
                {"stateEstimate", "y"},
                {"stateEstimate", "z"},
                {"stateEstimateZ", "quat"}
              }, cb));
            log_block_pose_->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
          else if (i.first.find("default_topics.scan") == 0) {
            int freq = log_config_map["default_topics.scan.frequency"].get<int>();
            RCLCPP_INFO(logger_, "Logging to /scan at %d Hz", freq);

            publisher_scan_ = node->create_publisher<sensor_msgs::msg::LaserScan>(name + "/scan", 10);

            std::function<void(uint32_t, const logScan*)> cb = std::bind(&CrazyflieROS::on_logging_scan, this, std::placeholders::_1, std::placeholders::_2);

            log_block_scan_.reset(new LogBlock<logScan>(
              &cf_,{
                {"range", "front"},
                {"range", "left"},
                {"range", "back"},
                {"range", "right"}
              }, cb));
            log_block_scan_->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
          else if (i.first.find("custom_topics") == 0
                   && i.first.rfind(".vars") != std::string::npos) {
            std::string topic_name = i.first.substr(14, i.first.size() - 14 - 5);

            int freq = log_config_map["custom_topics." + topic_name + ".frequency"].get<int>();
            auto vars = log_config_map["custom_topics." + topic_name + ".vars"].get<std::vector<std::string>>();
            
            RCLCPP_INFO(logger_, "Logging to %s at %d Hz", topic_name.c_str(), freq);

            publishers_generic_.emplace_back(node->create_publisher<crazyflie_interfaces::msg::LogDataGeneric>(name + "/" + topic_name, 10));

            std::function<void(uint32_t, const std::vector<float>*, void* userData)> cb = std::bind(
              &CrazyflieROS::on_logging_custom,
              this,
              std::placeholders::_1,
              std::placeholders::_2,
              std::placeholders::_3);

            log_blocks_generic_.emplace_back(new LogBlockGeneric(
              &cf_,
              vars,
              (void*)&publishers_generic_.back(),
              cb));
            log_blocks_generic_.back()->start(uint8_t(100.0f / (float)freq)); // this is in tens of milliseconds
          }
        }
      }
    }

    RCLCPP_INFO(logger_, "Requesting memories...");
    cf_.requestMemoryToc();
  }

  void spin_some()
  {
    cf_.sendPing();
  }

  std::string broadcastUri() const
  {
    return cf_.broadcastUri();
  }

  uint8_t id() const
  {
    return cf_.address() & 0xFF;
  }

  const Crazyflie::ParamTocEntry* paramTocEntry(const std::string& group, const std::string& name)
  {
    return cf_.getParamTocEntry(group, name);
  }

  const std::string& name() const
  {
    return name_;
  }

  void change_parameter(const rclcpp::Parameter& p)
  {
    std::string prefix = name_ + ".params.";
    if (p.get_name().find(prefix) != 0) {
      RCLCPP_ERROR(
              logger_,
              "Incorrect parameter update request for param \"%s\"", p.get_name().c_str());
      return;
    }
    size_t pos = p.get_name().find(".", prefix.size());
    std::string group(p.get_name().begin() + prefix.size(), p.get_name().begin() + pos);
    std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

    RCLCPP_INFO(
        logger_,
        "Update parameter \"%s.%s\" to %s",
        group.c_str(),
        name.c_str(),
        p.value_to_string().c_str());

    auto entry = cf_.getParamTocEntry(group, name);
    if (entry) {
      switch (entry->type)
      {
      case Crazyflie::ParamTypeUint8:
        cf_.setParam<uint8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt8:
        cf_.setParam<int8_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint16:
        cf_.setParam<uint16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt16:
        cf_.setParam<int16_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeUint32:
        cf_.setParam<uint32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeInt32:
        cf_.setParam<int32_t>(entry->id, p.as_int());
        break;
      case Crazyflie::ParamTypeFloat:
        if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
          cf_.setParam<float>(entry->id, (float)p.as_int());
        } else {
          cf_.setParam<float>(entry->id, p.as_double());
        }

        break;
      }
    } else {
      RCLCPP_ERROR(logger_, "Could not find param %s/%s", group.c_str(), name.c_str());
    }
  }

private:

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  { 
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;
    cf_.sendFullStateSetpoint(
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    qx, qy, qz, qw,
    rollRate, pitchRate, yawRate);

  }

  void cmd_position_changed(const crazyflie_interfaces::msg::Position::SharedPtr msg) {
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;
    float yaw = msg->yaw;
    cf_.sendPositionSetpoint(x, y, z, yaw);
  }

  void cmd_vel_legacy_changed(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    float roll = msg->linear.y;
    float pitch = - (msg->linear.x);
    float yawrate = msg->angular.z;
    uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);
    // RCLCPP_INFO(logger_, "roll: %f, pitch: %f, yaw: %f, thrust: %u", roll, pitch, yawrate, (unsigned int)thrust);
    cf_.sendSetpoint(roll, pitch, yawrate, thrust);
  }

  void on_console(const char *msg)
  {
    message_buffer_ += msg;
    size_t pos = message_buffer_.find('\n');
    if (pos != std::string::npos)
    {
      message_buffer_[pos] = 0;
      RCLCPP_INFO(logger_, "%s", message_buffer_.c_str());
      message_buffer_.erase(0, pos + 1);
    }
  }

  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "emergency()");
    cf_.emergencyStop();
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                        std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "start_trajectory(id=%d, timescale=%f, reversed=%d, relative=%d, group_mask=%d)",
      request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
    cf_.startTrajectory(request->trajectory_id,
      request->timescale,
      request->reversed,
      request->relative,
      request->group_mask);
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
               std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "takeoff(height=%f m, duration=%f s, group_mask=%d)", 
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void land(const std::shared_ptr<Land::Request> request,
            std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "land(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    cf_.land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
             std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, relative=%d, group_mask=%d)",
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->relative,
                request->group_mask);
    cf_.goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw, 
              rclcpp::Duration(request->duration).seconds(),
              request->relative, request->group_mask);
  }

  void upload_trajectory(const std::shared_ptr<UploadTrajectory::Request> request,
                        std::shared_ptr<UploadTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "upload_trajectory(id=%d, offset=%d)",
                request->trajectory_id,
                request->piece_offset);

    std::vector<Crazyflie::poly4d> pieces(request->pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i)
    {
      if (   request->pieces[i].poly_x.size() != 8 
          || request->pieces[i].poly_y.size() != 8
          || request->pieces[i].poly_z.size() != 8
          || request->pieces[i].poly_yaw.size() != 8)
      {
        RCLCPP_FATAL(logger_, "Wrong number of pieces!");
        return;
      }
      pieces[i].duration = rclcpp::Duration(request->pieces[i].duration).seconds();
      for (size_t j = 0; j < 8; ++j)
      {
        pieces[i].p[0][j] = request->pieces[i].poly_x[j];
        pieces[i].p[1][j] = request->pieces[i].poly_y[j];
        pieces[i].p[2][j] = request->pieces[i].poly_z[j];
        pieces[i].p[3][j] = request->pieces[i].poly_yaw[j];
      }
    }
    cf_.uploadTrajectory(request->trajectory_id, request->piece_offset, pieces);
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                request->remain_valid_millisecs,
                request->group_mask);

    cf_.notifySetpointsStop(request->remain_valid_millisecs);
  }

  void on_logging_pose(uint32_t time_in_ms, const logPose* data) {
    if (publisher_pose_) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = "world";

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      publisher_pose_->publish(msg);

      // send a transform for this pose
      geometry_msgs::msg::TransformStamped msg2;
      msg2.header = msg.header;
      msg2.child_frame_id = name_;
      msg2.transform.translation.x = data->x;
      msg2.transform.translation.y = data->y;
      msg2.transform.translation.z = data->z;
      msg2.transform.rotation.x = q[0];
      msg2.transform.rotation.y = q[1];
      msg2.transform.rotation.z = q[2];
      msg2.transform.rotation.w = q[3];
      tf_broadcaster_.sendTransform(msg2);
    }
  }

  void on_logging_scan(uint32_t time_in_ms, const logScan* data) {
    if (publisher_scan_) {
      
      const float max_range = 3.49;
      float front_range = data->front / 1000.0f;
      if (front_range > max_range) front_range = std::numeric_limits<float>::infinity();
      float left_range = data->left / 1000.0f;
      if (left_range > max_range) left_range = std::numeric_limits<float>::infinity();
      float back_range = data->back / 1000.0f;
      if (back_range > max_range) back_range = std::numeric_limits<float>::infinity();
      float right_range = data->right / 1000.0f;
      if (right_range > max_range) right_range = std::numeric_limits<float>::infinity();

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = node_->get_clock()->now();
      msg.header.frame_id = name_;
      msg.range_min = 0.01;
      msg.range_max = max_range;
      msg.ranges.push_back(back_range);
      msg.ranges.push_back(right_range);
      msg.ranges.push_back(front_range);
      msg.ranges.push_back(left_range);
      msg.angle_min = -0.5 * 2 * M_PI;
      msg.angle_max = 0.25 * 2 * M_PI;
      msg.angle_increment = 1.0 * M_PI / 2;

      publisher_scan_->publish(msg);
    }
  }

  void on_logging_custom(uint32_t time_in_ms, const std::vector<float>* values, void* userData) {

    auto pub = reinterpret_cast<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr*>(userData);

    crazyflie_interfaces::msg::LogDataGeneric msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "world";
    msg.timestamp = time_in_ms;
    msg.values = *values;

    (*pub)->publish(msg);
  }

private:
  rclcpp::Logger logger_;
  CrazyflieLogger cf_logger_;

  Crazyflie cf_;
  std::string message_buffer_;
  std::string name_;

  rclcpp::Node* node_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Service<Empty>::SharedPtr service_emergency_;
  rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
  rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
  rclcpp::Service<Land>::SharedPtr service_land_;
  rclcpp::Service<GoTo>::SharedPtr service_go_to_;
  rclcpp::Service<UploadTrajectory>::SharedPtr service_upload_trajectory_;
  rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_legacy_;
  rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
  rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr subscription_cmd_position_;

  // logging
  std::unique_ptr<LogBlock<logPose>> log_block_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

  std::unique_ptr<LogBlock<logScan>> log_block_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;

  std::list<std::unique_ptr<LogBlockGeneric>> log_blocks_generic_;
  std::list<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr> publishers_generic_;
};

class CrazyflieServer : public rclcpp::Node
{
public:
  CrazyflieServer()
      : Node("crazyflie_server")
      , logger_(rclcpp::get_logger("all"))
  {
    // topics for "all"

    subscription_cmd_full_state_ = this->create_subscription<crazyflie_interfaces::msg::FullState>("all/cmd_full_state", rclcpp::SystemDefaultsQoS(), std::bind(&CrazyflieServer::cmd_full_state_changed, this, _1));

    // services for "all"
    service_emergency_ = this->create_service<Empty>("all/emergency", std::bind(&CrazyflieServer::emergency, this, _1, _2));
    service_start_trajectory_ = this->create_service<StartTrajectory>("all/start_trajectory", std::bind(&CrazyflieServer::start_trajectory, this, _1, _2));
    service_takeoff_ = this->create_service<Takeoff>("all/takeoff", std::bind(&CrazyflieServer::takeoff, this, _1, _2));
    service_land_ = this->create_service<Land>("all/land", std::bind(&CrazyflieServer::land, this, _1, _2));
    service_go_to_ = this->create_service<GoTo>("all/go_to", std::bind(&CrazyflieServer::go_to, this, _1, _2));
    service_notify_setpoints_stop_ = this->create_service<NotifySetpointsStop>("all/notify_setpoints_stop", std::bind(&CrazyflieServer::notify_setpoints_stop, this, _1, _2));

    // declare global params
    this->declare_parameter("all.broadcasts.num_repeats", 15);
    this->declare_parameter("all.broadcasts.delay_between_repeats_ms", 1);
    this->declare_parameter("firmware_params.query_all_values_on_connect", false);

    broadcasts_num_repeats_ = this->get_parameter("all.broadcasts.num_repeats").get_parameter_value().get<int>();
    broadcasts_delay_between_repeats_ms_ = this->get_parameter("all.broadcasts.delay_between_repeats_ms").get_parameter_value().get<int>();

    // load crazyflies from params
    auto node_parameters_iface = this->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    auto cf_names = extract_names(parameter_overrides, "robots");
    for (const auto &name : cf_names) {
      bool enabled = parameter_overrides.at("robots." + name + ".enabled").get<bool>();
      if (enabled) {
        // Lookup type
        std::string cf_type = parameter_overrides.at("robots." + name + ".type").get<std::string>();
        // Find the connection setting for the given type
        const auto con = parameter_overrides.find("robot_types." + cf_type + ".connection");
        std::string constr = "crazyflie";
        if (con != parameter_overrides.end()) {
          constr = con->second.get<std::string>();
        }

        // if it is a Crazyflie, try to connect
        if (constr == "crazyflie") {
          std::string uri = parameter_overrides.at("robots." + name + ".uri").get<std::string>();
          crazyflies_.emplace(name, std::make_unique<CrazyflieROS>(uri, cf_type, name, this));

          auto broadcastUri = crazyflies_[name]->broadcastUri();
          RCLCPP_INFO(logger_, "%s", broadcastUri.c_str());
          if (broadcaster_.count(broadcastUri) == 0) {
            broadcaster_.emplace(broadcastUri, std::make_unique<CrazyflieBroadcaster>(broadcastUri));
          }

          update_name_to_id_map(name, crazyflies_[name]->id());
        }
        else if (constr == "none") {
          // we still might want to track this object, so update our map
          uint8_t id = parameter_overrides.at("robots." + name + ".id").get<uint8_t>();
          update_name_to_id_map(name, id);
        } else {
          RCLCPP_INFO(logger_, "Unknown connection type %s", constr.c_str());
        }
      }
    }

    sub_poses_ = this->create_subscription<NamedPoseArray>(
        "poses", 1, std::bind(&CrazyflieServer::posesChanged, this, _1));

    // support for all.params

    // Create a parameter subscriber that can be used to monitor parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handle_ = param_subscriber_->add_parameter_event_callback(std::bind(&CrazyflieServer::on_parameter_event, this, _1));

  }

  void spin_some()
  {
    for (auto& cf : crazyflies_) {
      cf.second->spin_some();
    }
  }

private:
  void emergency(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
  {
    RCLCPP_INFO(logger_, "emergency()");
    for (int i = 0; i < broadcasts_num_repeats_; ++i)
    {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->emergencyStop();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
            std::shared_ptr<StartTrajectory::Response> response)
  {
    RCLCPP_INFO(logger_, "start_trajectory(id=%d, timescale=%f, reversed=%d, group_mask=%d)",
                request->trajectory_id,
                request->timescale,
                request->reversed,
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->startTrajectory(request->trajectory_id,
                            request->timescale,
                            request->reversed,
                            request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void takeoff(const std::shared_ptr<Takeoff::Request> request,
                        std::shared_ptr<Takeoff::Response> response)
  {
    RCLCPP_INFO(logger_, "takeoff(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void land(const std::shared_ptr<Land::Request> request,
           std::shared_ptr<Land::Response> response)
  {
    RCLCPP_INFO(logger_, "land(height=%f m, duration=%f s, group_mask=%d)",
                request->height,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto& bc : broadcaster_) {
        auto& cfbc = bc.second;
        cfbc->land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void go_to(const std::shared_ptr<GoTo::Request> request,
            std::shared_ptr<GoTo::Response> response)
  {
    RCLCPP_INFO(logger_, "go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, group_mask=%d)",
                request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw,
                rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                         std::shared_ptr<NotifySetpointsStop::Response> response)
  {
    RCLCPP_INFO(logger_, "notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                request->remain_valid_millisecs,
                request->group_mask);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->notifySetpointsStop(request->remain_valid_millisecs);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg)
  { 
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendFullStateSetpoint(
          x, y, z,
          vx, vy, vz,
          ax, ay, az,
          qx, qy, qz, qw,
          rollRate, pitchRate, yawRate);
    }

  }

  void posesChanged(const NamedPoseArray::SharedPtr msg)
  {
    // Here, we send all the poses to all CFs
    // In Crazyswarm1, we only sent the poses of the same group (i.e. channel)


    // split the message into parts that require position update and pose update
    std::vector<CrazyflieBroadcaster::externalPosition> data_position;
    std::vector<CrazyflieBroadcaster::externalPose> data_pose;

    for (const auto& pose : msg->poses) {
      const auto iter = name_to_id_.find(pose.name);
      if (iter != name_to_id_.end()) {
        uint8_t id = iter->second;
        if (isnan(pose.pose.orientation.w)) {
          data_position.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z});
        } else {
          data_pose.push_back({id, 
            (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
            (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w});
        }
      }
    }

    // send position only updates to the swarm
    if (data_position.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPositions(data_position);
      }
    }

    // send pose only updates to the swarm
    if (data_pose.size() > 0) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendExternalPoses(data_pose);
      }
    }
  }

  void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event)
  {
    if (event.node == "/crazyflie_server") {
      auto params = param_subscriber_->get_parameters_from_event(event);
      for (auto &p : params) {
        size_t params_pos = p.get_name().find(".params.");
        if (params_pos == std::string::npos) {
          continue;
        }
        std::string cfname(p.get_name().begin(), p.get_name().begin() + params_pos);
        size_t prefixsize = params_pos + 8;
        if (cfname == "all") {
          size_t pos = p.get_name().find(".", prefixsize);
          std::string group(p.get_name().begin() + prefixsize, p.get_name().begin() + pos);
          std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

          RCLCPP_INFO(
              logger_,
              "Update parameter \"%s.%s\" to %s",
              group.c_str(),
              name.c_str(),
              p.value_to_string().c_str());

          Crazyflie::ParamType paramType;
          for (auto& cf : crazyflies_) {
            const auto entry = cf.second->paramTocEntry(group, name);
            if (entry) {
              switch (entry->type)
              {
              case Crazyflie::ParamTypeUint8:
                broadcast_set_param<uint8_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt8:
                broadcast_set_param<int8_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeUint16:
                broadcast_set_param<uint16_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt16:
                broadcast_set_param<int16_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeUint32:
                broadcast_set_param<uint32_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeInt32:
                broadcast_set_param<int32_t>(group, name, p.as_int());
                break;
              case Crazyflie::ParamTypeFloat:
                if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                  broadcast_set_param<float>(group, name, (float)p.as_int());
                } else {
                  broadcast_set_param<float>(group, name, p.as_double());
                }
                break;
              }
              break;
            }
          }
        } else {
          auto iter = crazyflies_.find(cfname);
          if (iter != crazyflies_.end()) {
            iter->second->change_parameter(p);
          }
        }
      }
    }
  }

  template<class T>
  void broadcast_set_param(
    const std::string& group,
    const std::string& name,
    const T& value)
  {
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
      for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->setParam<T>(group.c_str(), name.c_str(), value);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
  }

  void update_name_to_id_map(const std::string& name, uint8_t id)
  {
    const auto iter = name_to_id_.find(name);
    if (iter != name_to_id_.end()) {
      RCLCPP_WARN(logger_, "At least two objects with the same id (%d, %s, %s)", id, name.c_str(), iter->first.c_str());
    } else {
      name_to_id_.insert(std::make_pair(name, id));
    }
  }

  private:
    rclcpp::Logger logger_;

    // subscribers
    rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
    rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;

    // services
    rclcpp::Service<Empty>::SharedPtr service_emergency_;
    rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
    rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
    rclcpp::Service<Land>::SharedPtr service_land_;
    rclcpp::Service<GoTo>::SharedPtr service_go_to_;
    rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;

    std::map<std::string, std::unique_ptr<CrazyflieROS>> crazyflies_;

    // broadcastUri -> broadcast object
    std::map<std::string, std::unique_ptr<CrazyflieBroadcaster>> broadcaster_;

    // maps CF name -> CF id
    std::map<std::string, uint8_t> name_to_id_;

    // global params
    int broadcasts_num_repeats_;
    int broadcasts_delay_between_repeats_ms_;

    // parameter updates
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;
  };

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // rclcpp::spin(std::make_shared<CrazyflieServer>());
  auto node = std::make_shared<CrazyflieServer>();
  while (true)
  {
    node->spin_some();
    rclcpp::spin_some(node);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  rclcpp::shutdown();
  return 0;
}
