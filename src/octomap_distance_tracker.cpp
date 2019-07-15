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
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/TransformStamped.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>
#include <octomap_msgs/conversions.h>

#include <octomap_distance_tracker.h>

#include <sstream>

namespace octomap_distance_tracker
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

OctomapDistanceTracker::OctomapDistanceTracker(ros::NodeHandle private_nh_, ros::Duration update_period) :
  new_points_received_(false),
  new_map_update_received_(false),
  maps_received_(0),
  map_updates_received_(0),
  queue_size_(5),
  octomap_topic_property_("/octomap/octomap_binary_updates"),
  base_frame_("map"),
  fixed_frame_("odom")
{
  ros::NodeHandle private_nh(private_nh_);
  private_nh.param("tracked_octomap_topic", octomap_topic_property_, octomap_topic_property_);
  update_sub_->subscribe(private_nh, octomap_topic_property_, queue_size_);
  update_sub_->registerCallback(boost::bind(&OctomapDistanceTracker::incomingUpdateMessageCallback, this, _1));

  tf_buffer_.reset(new tf2_ros::Buffer);
  listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  update_timer_ = private_nh.createTimer(update_period, boost::bind(&OctomapDistanceTracker::timerCallback, this, _1));
}

OctomapDistanceTracker::~OctomapDistanceTracker()
{
  delete oc_tree_;
}

void OctomapDistanceTracker::incomingUpdateMessageCallback(const octomap_msgs::OctomapUpdateConstPtr& msg)
{
  static bool dist_map_created = false;
  // creating octree
  boost::recursive_mutex::scoped_lock lock(mutex_);
  if (map_updates_received_ == 0) {
    delete oc_tree_;
    oc_tree_ = nullptr;
  }
  map_updates_received_++;

//  if(!checkType(msg->octomap_bounds.id)){
//    return;
//  }

  header_ = msg->header;
//  if (!updateFromTF()) {
//      std::stringstream ss;
//      return;
//  }

  // Get update data
  octomap::OcTree* update_bounds = (octomap::OcTree*)octomap_msgs::msgToMap(msg->octomap_bounds);
  octomap::OcTree* update_values = (octomap::OcTree*)octomap_msgs::msgToMap(msg->octomap_update);
  if (!(update_bounds && update_values)){
    ROS_ERROR("Failed to deserialize octree message.");
    // Delete memory before this exit point
    delete update_bounds;
    delete update_values;
    return;
  }

  // Merge new tree into internal tree
  if(oc_tree_)
  {
    oc_tree_->setTreeValues(update_values, update_bounds, false, true);
    // Since we've stored the values, delete this copy
    delete update_values;
  }
  // If no tree exists, just fill it with our new values
  // Do not delete update_values in this case because it becomes the internal tree
  else
  {
    oc_tree_ = update_values;
  }

  double x,y,z;
  oc_tree_->getMetricMin(x,y,z);
  octomap::point3d min(x,y,z);
  //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
  oc_tree_->getMetricMax(x,y,z);
  octomap::point3d max(x,y,z);
  //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;
  float maxDist = 1.0;

  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  // Static makes this distmap only be created once
  if(!distmap_){
	  distmap_ = boost::shared_ptr<DynamicEDTOctomap>(new DynamicEDTOctomap(maxDist, oc_tree_, min, max, false));
  }
  //distmap_.boundingBoxMaxKey = update_bounds->bbx_max_key;
  //distmap_.boundingBoxMinKey = update_bounds->bbx_min_key;
  distmap_->update();

  // No reason to preserve bounds
  delete update_bounds;
  new_map_update_received_ = true;
  ROS_DEBUG("Message received and processed");
  //updateNewPoints();
}

void OctomapDistanceTracker::timerCallback(const ros::TimerEvent&){
	geometry_msgs::TransformStamped point_to_base_tf;

    // Lookup depth_frame at time it was acquired to base_frame at current time (odom frame fixed in time)
    try {
    	point_to_base_tf = tf_buffer_->lookupTransform("odom", "base", ros::Time(0));

    } catch (...) {
        // Transform lookup failed
    	ROS_WARN("Transform failed!");
        return;
    }

	octomap::point3d observer(
		point_to_base_tf.transform.translation.x,
		point_to_base_tf.transform.translation.y,
		0.5f
		);

    float dist = distmap_->getDistance(observer);

    ROS_INFO_STREAM("Closest point: " << dist << std::endl);

}


} // namespace


