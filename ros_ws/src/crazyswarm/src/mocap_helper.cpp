#include <iostream>

#include <ros/ros.h>

// Motion Capture
#ifdef ENABLE_VICON
#include <libmotioncapture/vicon.h>
#endif
#ifdef ENABLE_OPTITRACK
#include <libmotioncapture/optitrack.h>
#endif
#ifdef ENABLE_PHASESPACE
#include <libmotioncapture/phasespace.h>
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_helper");

  ros::NodeHandle nl("~");

  std::string motionCaptureType;
  nl.param<std::string>("motion_capture_type", motionCaptureType, "vicon");

  // Make a new client
  libmotioncapture::MotionCapture* mocap = nullptr;
  if (false)
  {
  }
#ifdef ENABLE_VICON
  else if (motionCaptureType == "vicon")
  {
    std::string hostName;
    nl.getParam("vicon_host_name", hostName);
    mocap = new libmotioncapture::MotionCaptureVicon(hostName,
      /*enableObjects*/ false,
      /*enablePointcloud*/ true);
  }
#endif
#ifdef ENABLE_OPTITRACK
  else if (motionCaptureType == "optitrack")
  {
    std::string localIP;
    std::string serverIP;
    nl.getParam("optitrack_local_ip", localIP);
    nl.getParam("optitrack_server_ip", serverIP);
    mocap = new libmotioncapture::MotionCaptureOptitrack(localIP, serverIP);
  }
#endif
#ifdef ENABLE_PHASESPACE
  else if (motionCaptureType == "phasespace")
  {
    std::string ip;
    int numMarkers;
    nl.getParam("phasespace_ip", ip);
    nl.getParam("phasespace_num_markers", numMarkers);
    std::map<size_t, std::pair<int, int> > cfs;
    cfs[231] = std::make_pair<int, int>(10, 11);
    mocap = new libmotioncapture::MotionCapturePhasespace(ip, numMarkers, cfs);
  }
#endif
  else {
    throw std::runtime_error("Unknown motion capture type!");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t frameId = 0; ros::ok(); ++frameId) {
    std::cout << "frame " << frameId << ":" << std::endl;
    // Get a frame
    mocap->waitForNextFrame();
    mocap->getPointCloud(markers);

    std::cout << "    points:" << std::endl;

    for (size_t i = 0; i < markers->size(); ++i) {
      const pcl::PointXYZ& point = markers->at(i);
      std::cout << "      \"" << i << "\": [" << point.x << "," << point.y << "," << point.z << "]" << std::endl;
    }


    ros::spinOnce();
  }

  return 0;
}
