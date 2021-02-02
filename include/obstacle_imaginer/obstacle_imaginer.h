#ifndef _OBSTACLE_IMAGINER_H_
#define _OBSTACLE_IMAGINER_H_

#include <string>
#include <vector>
#include <iostream>


#include <ros/ros.h>
#include <base/BaseNode.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/bind.hpp>
#include <functional>
#include <map>
#include <cmath>

class ObstacleImaginer : public BaseNode {
private:

  
  // params
  double uav_radius;
  double ugv_radius;
  std::string global_frame;
  int vanish_timeout;
  int num_obs_per_odom;
  double planning_horizon_seconds;
  
  // bool debug;
  
  // variables
  
  // robot code:
  // 11 = R1, 12 = R2, 13 = R3, 51 = DS1, 52 = DS2, 53 = DS3, 54 = DS4
  std::map<const int, std::string> robot_codes;
  int ugv_lt_uav;
  sensor_msgs::PointCloud2 obstacles_msg;
  pcl::PointCloud<pcl::PointXYZI> obstacles;
  geometry_msgs::Point my_position;
  
  //ros

  //tf listener
  tf::TransformListener* listener;
  
  // publishers
  // TODO: Add waypoint_point_vis to communication_manager
  ros::Publisher virtual_obstacle_pub;

  // subscribers
  ros::Subscriber my_odom_sub;
  std::vector<ros::Subscriber> odom_subs;
  
  // callbacks
  void other_odom_callback(const nav_msgs::Odometry& msg);
  void way_point_callback(const geometry_msgs::PointStamped& msg);
  void my_way_point_callback(const geometry_msgs::PointStamped& msg);
  void my_odom_callback(const nav_msgs::Odometry& msg);
  
public:
  ObstacleImaginer(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~ObstacleImaginer();

};


#endif
