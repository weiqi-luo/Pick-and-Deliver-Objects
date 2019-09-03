/*********************************************************************
* STD INCLUDES
********************************************************************/
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <math.h>
#include <string>
/*********************************************************************
* ROS INCLUDES
********************************************************************/
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>

/******************
***************************************************
 * PCL specific includes
 * ******************************************************************/
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

/*********************************************************************
 * Octomap
 * ******************************************************************/
# include <octomap_server/OctomapServerConfig.h>
# include <octomap/octomap.h>
# include <octomap_msgs/Octomap.h>
# include <octomap/Pointcloud.h>
# include <octomap_msgs/conversions.h>

/*********************************************************************
 * Moveit
 * ******************************************************************/
#include <moveit/pointcloud_octomap_updater/pointcloud_octomap_updater.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_srvs/Empty.h>

//moveit::planning_interface::PlanningSceneInterface PSI;
//PSI.addCollisionObjects();

ros::Publisher pub_octomap;
ros::ServiceServer shutdown_pub;
ros::Subscriber sub_collisionobj;

void octomap_cb(const octomap_msgs::Octomap& msg){

  moveit_msgs::PlanningSceneWorld planning_scene_world;
  ROS_INFO_STREAM(__LINE__);

  planning_scene_world.octomap.octomap.header = msg.header;

  planning_scene_world.octomap.octomap.binary = msg.binary;

  planning_scene_world.octomap.octomap.id     = msg.id;

  planning_scene_world.octomap.octomap.resolution = msg.resolution;

  planning_scene_world.octomap.octomap.data = msg.data;


  //planning_scene_world.

  pub_octomap.publish(planning_scene_world);
}

bool process_shutdown(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_WARN_STREAM(__LINE__);
    pub_octomap.shutdown();
    return 1;
}


int main(int argc, char** argv){

  ros::init(argc, argv, "manip_oct_service");

  ros::NodeHandle node;

  ROS_INFO_STREAM("Start octomap progress...");

  ros::Subscriber sub = node.subscribe("octomap_binary", 10, octomap_cb);

  pub_octomap = node.advertise<moveit_msgs::PlanningSceneWorld>("/planning_scene_world", 2);

//  sub_collisionobj = node.subscribe("");


  shutdown_pub = node.advertiseService("shutdown_octopub", process_shutdown);


  ros::spin();

  return 0;
}

