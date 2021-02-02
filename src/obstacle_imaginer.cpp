
#include <obstacle_imaginer/obstacle_imaginer.h>

ObstacleImaginer::ObstacleImaginer(std::string node_name)
  : BaseNode(node_name){
}

bool ObstacleImaginer::initialize(){

  robot_codes[11] = "ugv1";
  robot_codes[12] = "ugv2";
  robot_codes[13] = "ugv3";
  robot_codes[51] = "uav1";
  robot_codes[52] = "uav2";
  robot_codes[53] = "uav3";
  robot_codes[54] = "uav4";
  ugv_lt_uav = 50;

  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  uav_radius = pnh->param("uav_radius", 2.);
  ugv_radius = pnh->param("ugv_radius", 2.);
  global_frame = pnh->param("global_frame", std::string("map"));
  vanish_timeout = pnh->param("vanish_timeout_in_executions", 6);
  num_obs_per_odom = pnh->param("num_obs_per_odom", 6);
  planning_horizon_seconds = pnh->param("planning_horizon_seconds", 2.0);

  // init publishers
  virtual_obstacle_pub = nh->advertise<sensor_msgs::PointCloud2>("virtual_obstacles", 1);

  // init subscribers
  my_odom_sub = nh->subscribe("integrated_to_map", 1, &ObstacleImaginer::my_odom_callback, this);
  for (std::pair<const int, std::string>& r : robot_codes){
    odom_subs.push_back(nh->subscribe(r.second + "/integrated_to_map", 1, &ObstacleImaginer::other_odom_callback, this));
  }

  listener = new tf::TransformListener();
  
  return true;
}

bool ObstacleImaginer::execute(){

  static int vanish_timeout_counter = 0;

  pcl::toROSMsg(obstacles, obstacles_msg);
  obstacles_msg.header.stamp = ros::Time::now();
  obstacles_msg.header.frame_id = global_frame;

  // don't send empty message for a few cycles in case of laggy comms
  if (obstacles_msg.data.size() == 0) {
    vanish_timeout_counter += 1;
    if (vanish_timeout_counter >= vanish_timeout) {
      vanish_timeout_counter = 0;
      virtual_obstacle_pub.publish(obstacles_msg);
    }
  }
  else virtual_obstacle_pub.publish(obstacles_msg);

  obstacles.clear();

  return true;
}


  // callbacks
void ObstacleImaginer::other_odom_callback(const nav_msgs::Odometry& msg){
  
  pcl::PointXYZI point;

  // location
  point.x = msg.pose.pose.position.x;
  point.y = msg.pose.pose.position.y;
  point.z = msg.pose.pose.position.z;

  point.intensity = pow(pow(point.x - my_position.x, 2) + pow(point.y - my_position.y, 2), 0.5);
  if (point.intensity > uav_radius)
    point.intensity = uav_radius;

  obstacles.points.push_back(point);

  // velocity
  for (int i = 0; i < num_obs_per_odom; i++){
    point.x += msg.twist.twist.linear.x * i / num_obs_per_odom * planning_horizon_seconds;
    point.y += msg.twist.twist.linear.y * i / num_obs_per_odom * planning_horizon_seconds;
    point.z += msg.twist.twist.linear.z * i / num_obs_per_odom * planning_horizon_seconds;
    point.intensity = pow(pow(point.x - my_position.x, 2) + pow(point.y - my_position.y, 2), 0.5);
    if (point.intensity > uav_radius)
      point.intensity = uav_radius;
    obstacles.points.push_back(point);
  }

}

  // callbacks
void ObstacleImaginer::my_odom_callback(const nav_msgs::Odometry& msg){

  my_position.x = msg.pose.pose.position.x;
  my_position.y = msg.pose.pose.position.y;
  my_position.z = msg.pose.pose.position.z;
  
}

ObstacleImaginer::~ObstacleImaginer(){
  ;
}

BaseNode* BaseNode::get(){
  ObstacleImaginer* obstacle_imaginer = new ObstacleImaginer("ObstacleImaginer");
  return obstacle_imaginer;
}
