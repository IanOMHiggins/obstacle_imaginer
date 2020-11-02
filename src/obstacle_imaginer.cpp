
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

  
  // init subscribers
  for (std::pair<short, std::string>& r : robot_codes){
    odom_subs.push_back(nh.subscribe(r.second + "/integrated_to_map", 1, 
      boost::bind(&ObstacleImaginer::odom_callback, _1, r.first), this));
    way_point_subs.push_back(nh.subscribe(r.second + "/waypoint_point_vis", 1, 
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

  // if code < 50, we know it's a ugv.
  if (robot_code < ugv_lt_uav){
    virtual_obstacle_pub.publish()
  }
  

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
