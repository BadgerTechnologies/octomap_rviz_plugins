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
#include <type_traits>

#include <ros/ros.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/ColorOcTree.h>

#include <rviz/display.h>
#include <rviz/display_context.h>
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

// Everything in rviz_tf_api_fixup can be removed once there is no longer any
// need to be backwards compatible
namespace rviz_tf_api_fixup
{

template<typename> struct type_sink { typedef void type; }; // consumes a type, and makes it `void`
template<typename T> using type_sink_t = typename type_sink<T>::type;

template<typename T, typename=void> struct has_get_tf2_buffer_ptr : std::false_type {};
template<typename T> struct has_get_tf2_buffer_ptr<
  T,
  type_sink_t< decltype( std::declval<T>().getTF2BufferPtr() ) >
> : std::true_type {};

template <typename M, typename Context, typename Enabler = void>
class DisplayContextTFMessageFilterWrapper
{
};

template <typename M, typename Context>
class DisplayContextTFMessageFilterWrapper<M, Context, typename std::enable_if<has_get_tf2_buffer_ptr<Context>::value>::type>
{
  using FilterType = tf2_ros::MessageFilter<M>;
  using FilterPtr = std::shared_ptr<FilterType>;
public:
  void reset(Context* context, std::string frame_id, uint32_t queue_size, ros::NodeHandle nh)
  {
    filter_ptr_ = std::make_shared<FilterType>(
        *context->getTF2BufferPtr(),
	frame_id,
	queue_size,
	nh);
  }

  void reset()
  {
    filter_ptr_.reset();
  }

  FilterPtr getFilter() { return filter_ptr_; }

  void setTargetFrame(std::string frame_id) { if (filter_ptr_) filter_ptr_->setTargetFrame(frame_id); }

private:
  FilterPtr filter_ptr_;
};

template <typename M, typename Context>
class DisplayContextTFMessageFilterWrapper<M, Context, typename std::enable_if<!has_get_tf2_buffer_ptr<Context>::value>::type>
{
  using FilterType = tf::MessageFilter<M>;
  using FilterPtr = std::shared_ptr<FilterType>;
public:
  void reset(Context* context, std::string frame_id, uint32_t queue_size, ros::NodeHandle nh)
  {
    filter_ptr_ = std::make_shared<FilterType>(
        *context->getTFClient(),
	frame_id,
	queue_size,
	nh);
  }

  void reset()
  {
    filter_ptr_.reset();
  }

  FilterPtr getFilter() { return filter_ptr_; }

  void setTargetFrame(std::string frame_id) { if (filter_ptr_) filter_ptr_->setTargetFrame(frame_id); }

private:
  FilterPtr filter_ptr_;
};

}  // namespace rviz_tf_api_fixup

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

  boost::shared_ptr<message_filters::Subscriber<octomap_msgs::Octomap> > map_sub_;
  boost::shared_ptr<message_filters::Subscriber<octomap_msgs::OctomapUpdate> > update_sub_;
  rviz_tf_api_fixup::DisplayContextTFMessageFilterWrapper<octomap_msgs::Octomap, rviz::DisplayContext> tf_map_sub_wrapper_;
  rviz_tf_api_fixup::DisplayContextTFMessageFilterWrapper<octomap_msgs::OctomapUpdate, rviz::DisplayContext> tf_update_sub_wrapper_;
  ros::Timer resub_timer_;

  boost::recursive_mutex mutex_;

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
  boost::shared_ptr<OcTreeType> oc_tree_;
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
