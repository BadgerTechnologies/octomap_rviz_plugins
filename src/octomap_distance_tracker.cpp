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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
//#include <pcl/io/io.h>

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
  octomap_topic_property_("/octomap/front_lidar/octomap_binary_updates"),
  base_frame_("base_link"),
  fixed_frame_("odom"),
  dist_map_created_(false)
{
  ros::NodeHandle private_nh(private_nh_);
  private_nh.param("tracked_octomap_topic", octomap_topic_property_, octomap_topic_property_);
  map_sub_.subscribe(private_nh, octomap_topic_property_, queue_size_);
  map_sub_.registerCallback(boost::bind(&OctomapDistanceTracker::incomingMapCallback, this, _1));
  pointcloud_sub_.subscribe(private_nh, "tophat/pointcloud", queue_size_);
  pointcloud_sub_.registerCallback(boost::bind(&OctomapDistanceTracker::incomingPointcloudCallback, this, _1));

  tf_buffer_.reset(new tf2_ros::Buffer);
  listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  dist_octree_pub_ = private_nh.advertise<octomap_msgs::Octomap>("dist_octomap", 1);


}

OctomapDistanceTracker::~OctomapDistanceTracker()
{
  delete oc_tree_;
  delete distmap_;
}

void OctomapDistanceTracker::incomingMapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock lock(mutex_);

  map_header_ = msg->header;

  delete oc_tree_;
  oc_tree_ = nullptr;

  // Deserialize map data
  octomap::OcTree* update_bounds = (octomap::OcTree*)octomap_msgs::msgToMap(*msg);
  if (!update_bounds){
    ROS_ERROR("Failed to deserialize octree message.");
    // Delete memory before this exit point
    delete update_bounds;
    return;
  }

  // Update internal tree
  oc_tree_ = update_bounds;

  double x,y,z;
  oc_tree_->getMetricMin(x,y,z);
  octomap::point3d min(x,y,z);
  std::cout<<std::endl<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
  oc_tree_->getMetricMax(x,y,z);
  octomap::point3d max(x,y,z);
  std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;
  float maxDist = 5.0;

  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  delete distmap_;
  distmap_ = new DynamicEDTOctomap(maxDist, oc_tree_, min, max, false);
  distmap_->update();
  ROS_INFO("Message received and processed");
}

void OctomapDistanceTracker::incomingPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
	boost::recursive_mutex::scoped_lock lock(mutex_);
	geometry_msgs::TransformStamped fixed_to_base_tf;

	pointcloud_header_ = msg->header;
	pcl::PCLPointCloud2 pointcloud2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl_conversions::toPCL(*msg, pointcloud2);
    pcl::fromPCLPointCloud2(pointcloud2, *pointcloud);

    // Lookup depth_frame at time it was acquired to base_frame at current time (odom frame fixed in time)
    try {
    	fixed_to_base_tf = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, ros::Time(0));

    } catch (tf2::TransformException &ex) {
        // Transform lookup failed
    	ROS_WARN("%s", ex.what());
        return;
    }

	tf::StampedTransform fixed_to_base_tf2;
	tf::transformStampedMsgToTF(fixed_to_base_tf, fixed_to_base_tf2);

    pcl_ros::transformPointCloud(*pointcloud, *transformed_pointcloud, fixed_to_base_tf2);

	octomap::point3d observer(
		//0,0,0.5f
		fixed_to_base_tf.transform.translation.x,
		fixed_to_base_tf.transform.translation.y,
		0.5f
		);
	octomap::point3d observed;

	if(dist_map_created_) {
		//distmap_->
		float dist(0);
		distmap_->getDistanceAndClosestObstacle(observer, dist, observed);
		ROS_INFO("x: %f, y: %f, z: %f" , observer.x(), observer.y(), observer.z());
		ROS_INFO("x: %f, y: %f, z: %f" , observed.x(), observed.y(), observed.z());
    	ROS_INFO_STREAM("Closest point: " << dist);
    	publishMap(oc_tree_);
	} else {
		ROS_WARN_THROTTLE(10, "Distmap does not yet exist");
	}


}

void OctomapDistanceTracker::publishMap(octomap::OcTree* oc_tree)
{
//  octomap_msgs::Octomap map;
//  map.header.frame_id = "odom";
//  map.header.stamp = ros::Time::now();
//
//  if (octomap_msgs::binaryMapToMsg(*oc_tree, map))
//	  dist_octree_pub_.publish(map);
//  else
//    ROS_ERROR("Error serializing OctoMap");
}


} // namespace


