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
  // ChunkManager* chunk_manager;
  // bool got_tracking_point, got_look_ahead;
  // core_trajectory_msgs::Odometry tracking_point, look_ahead;
  // templib::TimeOutMonitor* look_ahead_time_monitor;
  // templib::TimeOutMonitor* all_in_collision_monitor;
  // templib::TimeOutMonitor* stuck_monitor;
  // templib::TemporalThresholdMonitor<tf::Vector3, double>* stationary_monitor;
  // const int SMOOTH_YAW = 1;
  // const int WAYPOINT_YAW = 2;
  // int yaw_mode;
  // WaypointManager* waypoint_manager;
  
  // robot code:
  // 11 = R1, 12 = R2, 13 = R3, 51 = DS1, 52 = DS2, 53 = DS3, 54 = DS4
  std::map<short, std::string> robot_codes;


  // services
  // ros::ServiceServer clear_map_server;
  
  // publishers
  // TODO: Add way_point to communication_manager
  ros::Publisher virtual_obstacle_pub;

  // subscribers
  // ros::CallbackQueue cloud_callback_queue;
  // ros::AsyncSpinner* spinner;
  // ros::Subscriber cloud_sub;
  // ros::Subscriber odom_sub;
  // ros::Subscriber tracking_point_sub;
  // ros::Subscriber look_ahead_sub;
  // ros::Subscriber virtual_obstacle_sub;
  // ros::Subscriber reset_stuck_sub;
  // ros::Timer health_timer;
  // tf::TransformListener* listener;
  // std::vector<message_filters::Subscriber<sensor_msgs::Image>* > depth_image_subs;
  // std::vector<message_filters::Subscriber<sensor_msgs::CameraInfo>* > camera_info_subs;
  // std::vector<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>* > depth_image_syncs;
  std::vector<ros::Subscriber> odom_subs;
  std::vector<ros::Subscriber> way_point_subs;
  
  // callbacks
  void odom_callback(const nav_msgs::Odometry& msg, short robot_code);
  void way_point_callback(const geometry_msgs::PointStamped& msg, short robot_code);
  // void cloud_callback(const sensor_msgs::PointCloud2& msg);
  // void odom_callback(nav_msgs::Odometry msg);
  // void tracking_point_callback(core_trajectory_msgs::Odometry msg);
  // void look_ahead_callback(core_trajectory_msgs::Odometry msg);
  // void timer_callback(const ros::TimerEvent& event);
  // void depth_image_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
  // void virtual_obstacle_callback(const sensor_msgs::PointCloud2& msg);
  // void reset_stuck_callback(const std_msgs::Empty& msg);
  // bool clear_map_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  
  // void set_yaw(core_trajectory_msgs::TrajectoryXYZVYaw& trajectory, const geometry_msgs::PoseStamped& waypoint);
  
public:
  ObstacleImaginer(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ObstacleImaginer();

};


#endif
