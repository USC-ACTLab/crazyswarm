#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

#include <crazyflie_driver/UploadTrajectory.h>
#include <crazyflie_driver/Takeoff.h>
#include <crazyflie_driver/Land.h>
#include <crazyflie_driver/StartTrajectory.h>
#include <crazyflie_driver/StartEllipse.h>
#include <crazyflie_driver/GoHome.h>



namespace Xbox360Buttons {

    enum {
        Green  = 0,
        Red    = 1,
        Blue   = 2,
        Yellow = 3,
        LB     = 4,
        RB     = 5,
        Back   = 6,
        Start  = 7,
        COUNT  = 8,
    };

}

class Manager
{
public:

    Manager()
        : m_subscribeJoy()
        , m_serviceEmergency()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_serviceStartTrajectory()
        , m_serviceEllipse()
        , m_serviceGoHome()
    {
        ros::NodeHandle nh;
        m_subscribeJoy = nh.subscribe("/joy", 1, &Manager::joyChanged, this);

        ROS_INFO("Wait for services...");

        ros::service::waitForService("/emergency");
        m_serviceEmergency = nh.serviceClient<std_srvs::Empty>("/emergency");
        ros::service::waitForService("/takeoff");
        m_serviceTakeoff = nh.serviceClient<crazyflie_driver::Takeoff>("/takeoff");
        ros::service::waitForService("/land");
        m_serviceLand = nh.serviceClient<crazyflie_driver::Land>("/land");
        ros::service::waitForService("/start_trajectory");
        m_serviceStartTrajectory = nh.serviceClient<std_srvs::Empty>("/start_trajectory");
        ros::service::waitForService("/start_ellipse");
        m_serviceEllipse = nh.serviceClient<std_srvs::Empty>("/start_ellipse");
        ros::service::waitForService("/go_home");
        m_serviceGoHome = nh.serviceClient<std_srvs::Empty>("/go_home");

        ROS_INFO("Manager ready.");
    }

    ~Manager()
    {
    }

private:
    void joyChanged(
        const sensor_msgs::Joy::ConstPtr& msg)
    {
        static std::vector<int> lastButtonState(Xbox360Buttons::COUNT);

        if (msg->buttons.size() >= Xbox360Buttons::COUNT
            && lastButtonState.size() >= Xbox360Buttons::COUNT)
        {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Start] == 1 && lastButtonState[Xbox360Buttons::Start] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Back] == 1 && lastButtonState[Xbox360Buttons::Back] == 0) {
                land();
            }
            #if 0
            if (msg->buttons[Xbox360Buttons::Yellow] == 1 && lastButtonState[Xbox360Buttons::Yellow] == 0) {
                startTrajectory();
            }
            if (msg->buttons[Xbox360Buttons::Blue] == 1 && lastButtonState[Xbox360Buttons::Blue] == 0) {
                uploadTrajectory();
            }
            if (msg->buttons[Xbox360Buttons::Green] == 1 && lastButtonState[Xbox360Buttons::Green] == 0) {
                ellipse();
            }
            if (msg->buttons[Xbox360Buttons::LB] == 1 && lastButtonState[Xbox360Buttons::LB] == 0) {
                goHome();
            }
            #endif
        }

        lastButtonState = msg->buttons;
    }

    void emergency()
    {
        ROS_INFO("emergency requested...");
        std_srvs::Empty srv;
        m_serviceEmergency.call(srv);
        ROS_INFO("Done.");
    }

    void takeoff()
    {
        crazyflie_driver::Takeoff srv;
        srv.request.group = 0;
        srv.request.height = 0.5;
        srv.request.time_from_start = ros::Duration(2.0);
        m_serviceTakeoff.call(srv);
    }

    void land()
    {
        crazyflie_driver::Land srv;
        srv.request.group = 0;
        srv.request.height = 0.05;
        srv.request.time_from_start = ros::Duration(3.5);
        m_serviceLand.call(srv);
    }

    void startTrajectory()
    {
        crazyflie_driver::StartTrajectory srv;
        srv.request.group = 0;
        m_serviceStartTrajectory.call(srv);
    }

    void ellipse()
    {
        crazyflie_driver::StartEllipse srv;
        srv.request.group = 0;
        m_serviceEllipse.call(srv);
    }

    void goHome()
    {
        crazyflie_driver::GoHome srv;
        srv.request.group = 0;
        m_serviceGoHome.call(srv);
    }

    void uploadTrajectory()
    {
        // read trajectory
        ros::NodeHandle n("~");
        std::string csvFile;
        double timescale; // 2 means it will take 2x longer
        n.getParam("csv_file", csvFile);
        n.getParam("timescale", timescale);

        crazyflie_driver::UploadTrajectory srv;
        std::ifstream stream(csvFile);
        bool firstLine = true;
        for (std::string line; std::getline(stream, line); ) {
            if (!firstLine) {
                std::stringstream sstr;
                char dummy;
                float duration, accX, accY, accZ;
                crazyflie_driver::QuadcopterTrajectoryPoly poly;
                sstr << line;
                sstr >> duration >> dummy;
                poly.poly_x.resize(8);
                for (size_t i = 0; i < 8; ++i) {
                    sstr >> poly.poly_x[i] >> dummy;
                }
                poly.poly_y.resize(8);
                for (size_t i = 0; i < 8; ++i) {
                    sstr >> poly.poly_y[i] >> dummy;
                }
                poly.poly_z.resize(8);
                for (size_t i = 0; i < 8; ++i) {
                    sstr >> poly.poly_z[i] >> dummy;
                }
                poly.poly_yaw.resize(8);
                for (size_t i = 0; i < 8; ++i) {
                    sstr >> poly.poly_yaw[i] >> dummy;
                }
                // poly.poly_x[0] += x_offset;
                // poly.poly_y[0] += y_offset;
                // poly.poly_z[0] += z_offset;

                // // e.g. if s==2 the new polynomial will be stretched to take 2x longer
                float recip = 1.0f / timescale;
                float scale = recip;
                for (int i = 1; i < 8; ++i) {
                    poly.poly_x[i] *= scale;
                    poly.poly_y[i] *= scale;
                    poly.poly_z[i] *= scale;
                    poly.poly_yaw[i] *= scale;
                    scale *= recip;
                }
                poly.duration = ros::Duration(duration * timescale);

              // point.position.x += x_offset;
                // point.position.y += y_offset;
                srv.request.polygons.push_back(poly);
            }
            firstLine = false;
        }

        // upload for each CF
        ros::NodeHandle nGlobal;

        XmlRpc::XmlRpcValue crazyflies;
        nGlobal.getParam("crazyflies", crazyflies);
        ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for (int32_t i = 0; i < crazyflies.size(); ++i)
        {
            ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
            int id = crazyflie["id"];

            ros::service::call("cf" + std::to_string(id) + "/upload_trajectory", srv);
        }

        ROS_INFO("All trajectories uploaded.");
    }

    // void callEmptyService(const std::string&)
    // {
    //     ros::NodeHandle n;
    //     ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("add_two_ints");
    //     std_srvs::Empty srv;
    //     if (!client.call(srv)) {
    //         ROS_ERROR("Failed to call emergency service");
    //     }
    // }

private:
    ros::Subscriber m_subscribeJoy;

    ros::ServiceClient m_serviceEmergency;
    ros::ServiceClient m_serviceTakeoff;
    ros::ServiceClient m_serviceLand;
    ros::ServiceClient m_serviceStartTrajectory;
    ros::ServiceClient m_serviceEllipse;
    ros::ServiceClient m_serviceGoHome;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  Manager manager;
  ros::spin();

  return 0;
}
