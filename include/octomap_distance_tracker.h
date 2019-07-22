/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#ifndef OCTOMAP_DISTANCE_TRACKER_H
#define OCTOMAP_DISTANCE_TRACKER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_msgs/PointIndices.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <message_filters/subscriber.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>


namespace octomap_distance_tracker
{

class OctomapDistanceTracker
{
public:
  OctomapDistanceTracker(ros::NodeHandle private_nh_, ros::Duration update_period);
  ~OctomapDistanceTracker();

protected:
  void subscribe();
  void unsubscribe();

  void incomingMapCallback(const octomap_msgs::OctomapConstPtr& msg);
  void incomingPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

  void publishMap(octomap::OcTree* oc_tree);

  void clear();

  message_filters::Subscriber<octomap_msgs::Octomap> map_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;

  boost::recursive_mutex mutex_;

  // point buffer
  bool new_points_received_;
  bool new_map_update_received_;
  bool dist_map_created_;

  // Ogre-rviz point clouds
  std::vector<double> box_size_;
  std_msgs::Header map_header_;
  std_msgs::Header pointcloud_header_;

  std::string octomap_topic_property_;
  std::string tree_depth_property_;
  std::string base_frame_;
  std::string fixed_frame_;

  // Transform Buffer
  boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  boost::shared_ptr<tf2_ros::TransformListener> listener_;

  DynamicEDTOctomap* distmap_ = nullptr;

  ros::Publisher matching_pointcloud_pub_;

  u_int32_t queue_size_;
  uint32_t maps_received_;
  uint32_t map_updates_received_;

  octomap::OcTree* oc_tree_ = nullptr;
};

}


#endif //OCCUPANCY_DISTANCE_TRACKER_H
