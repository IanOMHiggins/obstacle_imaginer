
#include <obstacle_imaginer/obstacle_imaginer.h>

ObstacleImaginer::ObstacleImaginer(std::string node_name)
  : BaseNode(node_name){
}

bool ObstacleImaginer::initialize(){


  robot_codes[(short)11] = "ugv1";
  robot_codes[(short)12] = "ugv2";
  robot_codes[(short)13] = "ugv3";
  robot_codes[(short)51] = "uav1";
  robot_codes[(short)52] = "uav2";
  robot_codes[(short)53] = "uav3";
  robot_codes[(short)54] = "uav4";


  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  uav_radius = pnh->param("uav_radius", 2.);
  ugv_radius = pnh->param("ugv_radius", 2.);

  // init publishers
  virtual_obstacle_pub = nh->advertise<sensor_msgs::PointCloud2>("virtual_obstacles", 1);

  // debug_markers_pub = nh->advertise<visualization_msgs::MarkerArray>("debug_markers2", 1);
  // health_pub = nh->advertise<std_msgs::Bool>("opencl_pointcloud_map_health2", 1);
  // traj_pub = nh->advertise<core_trajectory_msgs::TrajectoryXYZVYaw>("trajectory", 1);
  // stuck_pub = nh->advertise<std_msgs::Bool>("stuck", 1);
  // waypoint_pose_pub = nh->advertise<geometry_msgs::PoseStamped>("waypoint_pose_vis", 1);
  // waypoint_point_pub = nh->advertise<geometry_msgs::PointStamped>("waypoint_point_vis", 1);
  // map_clearing_point_pub = nh->advertise<geometry_msgs::PoseStamped>("map_clearing_point", 1);
  
  // init subscribers
  // cloud_sub = nh->subscribe("pointcloud", 1, &ObstacleImaginer::cloud_callback, this);
  // odom_sub = nh->subscribe("odometry", 1, &ObstacleImaginer::odom_callback, this);
  // tracking_point_sub = nh->subscribe("tracking_point", 1, &ObstacleImaginer::tracking_point_callback, this);
  // look_ahead_sub = nh->subscribe("look_ahead", 1, &ObstacleImaginer::look_ahead_callback, this);
  // virtual_obstacle_sub = nh->subscribe("virtual_obstacles", 1, &ObstacleImaginer::virtual_obstacle_callback, this);
  // reset_stuck_sub = nh->subscribe("reset_stuck", 1, &ObstacleImaginer::reset_stuck_callback, this);
  // health_timer = nh->createTimer(ros::Duration(0.1), &ObstacleImaginer::timer_callback, this);
  // listener = new tf::TransformListener();
  // std::vector<ros::Subscriber> odom_subs;
  // std::vector<ros::Subscriber> way_point_subs;
  for (std::pair<short, std::string>& r : robot_codes){
    odom_subs.push_back(nh.subscribe(r.second + "/integrated_to_map", 1, 
      boost::bind(&ObstacleImaginer::odom_callback, _1, r.first), this));
    way_point_subs.push_back(nh.subscribe(r.second + "/way_point", 1, 
      boost::bind(&ObstacleImaginer::way_point_callback, _1, r.first), this));
  }
  

  listener = new tf::TransformListener();
  
  

  return true;
}

bool ObstacleImaginer::execute(){

  return true;
}


  // callbacks
void odom_callback(const nav_msgs::Odometry& msg, short robot_code){
  // robot code:
  // 11 = R1, 12 = R2, 13 = R3, 51 = DS1, 52 = DS2, 53 = DS3, 54 = DS4
  


}


  // callbacks
void way_point_callback(const geometry_msgs::PointStamped& msg, short robot_code){
  // robot code:
  // 11 = R1, 12 = R2, 13 = R3, 51 = DS1, 52 = DS2, 53 = DS3, 54 = DS4



}

ObstacleImaginer::~ObstacleImaginer(){
  ;
}

BaseNode* BaseNode::get(){
  ObstacleImaginer* obstacle_imaginer = new ObstacleImaginer("ObstacleImaginer");
  return obstacle_imaginer;
}
