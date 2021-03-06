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

#ifndef RVIZ_OCCUPANCY_GRID_DISPLAY_H
#define RVIZ_OCCUPANCY_GRID_DISPLAY_H

#ifndef Q_MOC_RUN 
#include <ros/ros.h>

#include <memory>
#include <mutex>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>

#include <rviz/display.h>
#include "rviz/ogre_helpers/point_cloud.h"

#endif

namespace rviz {
class RosTopicProperty;
class IntProperty;
class EnumProperty;
class FloatProperty;
}

namespace octomap_rviz_plugin
{

class OccupancyGridDisplay : public rviz::Display
{
Q_OBJECT
public:
  OccupancyGridDisplay();
  virtual ~OccupancyGridDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

private Q_SLOTS:
  void updateQueueSize();
  void updateTopic();
  void updateTreeDepth();
  void updateOctreeRenderMode();
  void updateOctreeColorMode();
  void updateAlpha();
  void updateMaxHeight();
  void updateMinHeight();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  void subscribe();
  void subscribeUpdates();
  void resubscribeUpdates();
  void unsubscribe();
  void scheduleResubscribeUpdates();

  virtual void incomingMapMessageCallback(const octomap_msgs::OctomapConstPtr& msg) = 0;
  virtual void incomingUpdateMessageCallback(const octomap_msgs::OctomapUpdateConstPtr& msg) = 0;

  void setColor( double z_pos, double min_z, double max_z, double color_factor, rviz::PointCloud::Point& point);

  void clear();

  virtual bool updateFromTF();

  typedef std::vector<rviz::PointCloud::Point> VPoint;
  typedef std::vector<VPoint> VVPoint;

  std::shared_ptr<message_filters::Subscriber<octomap_msgs::Octomap> > map_sub_;
  std::shared_ptr<message_filters::Subscriber<octomap_msgs::OctomapUpdate> > update_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<octomap_msgs::Octomap>> tf_map_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<octomap_msgs::OctomapUpdate>> tf_update_sub_;
  ros::Timer resub_timer_;

  std::recursive_mutex mutex_;

  // point buffer
  VVPoint new_points_;
  VVPoint point_buf_;
  bool new_points_received_;
  bool new_map_update_received_;
  bool using_updates_;
  bool first_full_map_update_received_;
  unsigned int update_last_seq_;

  // Ogre-rviz point clouds
  std::vector<rviz::PointCloud*> cloud_;
  std::vector<double> box_size_;
  std_msgs::Header header_;

  // Plugin properties
  rviz::IntProperty* queue_size_property_;
  rviz::RosTopicProperty* octomap_topic_property_;
  rviz::EnumProperty* octree_render_property_;
  rviz::EnumProperty* octree_coloring_property_;
  rviz::IntProperty* tree_depth_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* max_height_property_;
  rviz::FloatProperty* min_height_property_;

  u_int32_t queue_size_;
  uint32_t maps_received_;
  uint32_t map_updates_received_;
  double color_factor_;
  double min_probability_;
  double max_probability_;
};

template <typename OcTreeType>
class TemplatedOccupancyGridDisplay: public OccupancyGridDisplay {
protected:
  std::shared_ptr<OcTreeType> oc_tree_;
  ~TemplatedOccupancyGridDisplay();
  void incomingMapMessageCallback(const octomap_msgs::OctomapConstPtr& msg);
  void incomingUpdateMessageCallback(const octomap_msgs::OctomapUpdateConstPtr& msg);
  void updateNewPoints();
  void setVoxelColor(rviz::PointCloud::Point& newPoint, typename OcTreeType::NodeType& node, double minZ, double maxZ);
  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

} // namespace octomap_rviz_plugin

#endif //RVIZ_OCCUPANCY_GRID_DISPLAY_H
