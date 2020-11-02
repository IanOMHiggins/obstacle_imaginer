#ifndef _OBSTACLE_IMAGINER_H_
#define _OBSTACLE_IMAGINER_H_

// #include <sensor_msgs/Image.h>
#include <string>

// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Range.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_ros/transforms.h>
// #include <core_trajectory_msgs/Odometry.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <vector>
// #include <list>
// #include <unordered_map>
// #include <fstream>
// #include <streambuf>
// #include <CL/cl.hpp>
// #include <ros/package.h>
// #include <vislib/vislib.h>
// #include <tflib/tflib.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Empty.h>
// #include <std_srvs/SetBool.h>
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <cv_bridge/cv_bridge.h>
// #include <templib/templib.h>



#include <ros/ros.h>
// #include <ros/spinner.h>
// #include <ros/callback_queue.h>
//#include <mutex>
#include <base/BaseNode.h>
#include <distance_map_local_planner/chunk_manager.h>
#include <distance_map_local_planner/waypoint_manager.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <boost/bind.hpp>
#include <functional>
#include <map>

class ObstacleImaginer : public BaseNode {
private:

  
  // params
  double uav_radius;
  double ugv_radius;
  
  // bool debug;
  
  // variables
  
  // robot code:
  // 11 = R1, 12 = R2, 13 = R3, 51 = DS1, 52 = DS2, 53 = DS3, 54 = DS4
  std::map<short, std::string> robot_codes;
  const short ugv_lt_uav = 50;


  // services
  // ros::ServiceServer clear_map_server;
  
  // publishers
  // TODO: Add waypoint_point_vis to communication_manager
  ros::Publisher virtual_obstacle_pub;

  // subscribers
  std::vector<ros::Subscriber> odom_subs;
  std::vector<ros::Subscriber> way_point_subs;
  
  // callbacks
  void odom_callback(const nav_msgs::Odometry& msg, short robot_code);
  void way_point_callback(const geometry_msgs::PointStamped& msg, short robot_code);
  
public:
  ObstacleImaginer(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ObstacleImaginer();

};


#endif
