/*
 * distance_tracker_node.cpp
 *
 *  Created on: Jul 11, 2019
 *      Author: jat
 */


#include <octomap_distance_tracker.h>
#include <ros/ros.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_distance_tracker;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_distance_tracker");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  OctomapDistanceTracker Tracker(private_nh, ros::Duration(0.1));

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
