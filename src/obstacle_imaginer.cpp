
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
  ugv_lt_uav = 50;

  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  uav_radius = pnh->param("uav_radius", 2.);
  ugv_radius = pnh->param("ugv_radius", 2.);
  global_frame = pnh->param("global_frame", std::string("darpa_map"));

  // init publishers
  my_way_point_pub = nh->advertise<geometry_msgs::PointStamped>("trajectory_waypoint", 1);
  virtual_obstacle_pub = nh->advertise<sensor_msgs::PointCloud2>("virtual_obstacles", 1);

  // init subscribers
  my_way_point_sub = nh->subscribe("waypoint_point_vis", 1, &ObstacleImaginer::my_way_point_callback, this);
  for (std::pair<const short, std::string>& r : robot_codes){
    odom_subs.push_back(nh->subscribe(r.second + "/integrated_to_map", 1, &ObstacleImaginer::odom_callback, this));
    way_point_subs.push_back(nh->subscribe(r.second + "/trajectory_waypoint", 1, 
      &ObstacleImaginer::way_point_callback, this));
  }

  listener = new tf::TransformListener();
  
  return true;
}

bool ObstacleImaginer::execute(){

  pcl::toROSMsg(obstacles, obstacles_msg);
  obstacles_msg.header.stamp = ros::Time::now();
  obstacles_msg.header.frame_id = global_frame;
  virtual_obstacle_pub.publish(obstacles_msg);

  obstacles.clear();

  return true;
}


  // callbacks
void ObstacleImaginer::odom_callback(const nav_msgs::Odometry& msg){
  
  pcl::PointXYZI point;
  point.x = msg.pose.pose.position.x;
  point.y = msg.pose.pose.position.y;
  point.z = msg.pose.pose.position.z;
  point.intensity = uav_radius;
  obstacles.points.push_back(point);

}

  // callbacks
void ObstacleImaginer::way_point_callback(const geometry_msgs::PointStamped& msg){
  
  pcl::PointXYZI point;
  point.x = msg.point.x;
  point.y = msg.point.y;
  point.z = msg.point.z;
  point.intensity = uav_radius;
  obstacles.points.push_back(point);
  
}
  // callbacks
void ObstacleImaginer::my_way_point_callback(const geometry_msgs::PointStamped& msg){

    try{
      tf::StampedTransform transform;
      listener->waitForTransform(msg.header.frame_id, global_frame, msg.header.stamp, ros::Duration(0.1));
      listener->lookupTransform(msg.header.frame_id, global_frame, msg.header.stamp, transform);
      geometry_msgs::PointStamped p;
      listener->transformPoint(global_frame, msg.header.stamp, msg, msg.header.frame_id, p);
      p.header.frame_id = global_frame;
      p.header.stamp = msg.header.stamp;
      my_way_point_pub.publish(p);
    }
    catch(const tf::TransformException& ex){
      ROS_ERROR_STREAM("TransformException while transforming my waypoint_point_vis: " << ex.what());
    }

}

ObstacleImaginer::~ObstacleImaginer(){
  ;
}

BaseNode* BaseNode::get(){
  ObstacleImaginer* obstacle_imaginer = new ObstacleImaginer("ObstacleImaginer");
  return obstacle_imaginer;
}
