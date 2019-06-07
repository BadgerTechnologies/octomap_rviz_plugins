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
#include <QObject>

#include "octomap_rviz_plugins/occupancy_grid_display.h"

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "rviz/visualization_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/OctomapUpdate.h>
#include <octomap_msgs/conversions.h>


#include <sstream>


using namespace rviz;

namespace octomap_rviz_plugin
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2
};

enum OctreeVoxelColorMode
{
  OCTOMAP_CELL_COLOR,
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
};

OccupancyGridDisplay::OccupancyGridDisplay() :
    rviz::Display(),
    new_points_received_(false),
    new_map_update_received_(false),
    using_updates(false),
    maps_received_(0),
    map_updates_received_(0),
    queue_size_(5),
    color_factor_(0.8)
{

  octomap_topic_property_ = new RosTopicProperty( "Octomap Topic",
                                                  "",
                                                  QString::fromStdString(ros::message_traits::datatype<octomap_msgs::Octomap>()),
                                                  "octomap_msgs::Octomap topic to subscribe to (binary or full probability map)",
                                                  this,
                                                  SLOT( updateTopic() ));

  queue_size_property_ = new IntProperty( "Queue Size",
                                          queue_size_,
                                          "Advanced: set the size of the incoming message queue.  Increasing this "
                                          "is useful if your incoming TF data is delayed significantly from your"
                                          " image data, but it can greatly increase memory usage if the messages are big.",
                                          this,
                                          SLOT( updateQueueSize() ));
  queue_size_property_->setMin(1);

  octree_render_property_ = new rviz::EnumProperty( "Voxel Rendering", "Occupied Voxels",
                                                    "Select voxel type.",
                                                     this,
                                                     SLOT( updateOctreeRenderMode() ) );

  octree_render_property_->addOption( "Occupied Voxels",  OCTOMAP_OCCUPIED_VOXELS );
  octree_render_property_->addOption( "Free Voxels",  OCTOMAP_FREE_VOXELS );
  octree_render_property_->addOption( "All Voxels",  OCTOMAP_FREE_VOXELS | OCTOMAP_OCCUPIED_VOXELS);

  octree_coloring_property_ = new rviz::EnumProperty( "Voxel Coloring", "Z-Axis",
                                                "Select voxel coloring mode",
                                                this,
                                                SLOT( updateOctreeColorMode() ) );

  octree_coloring_property_->addOption( "Cell Color",  OCTOMAP_CELL_COLOR );
  octree_coloring_property_->addOption( "Z-Axis",  OCTOMAP_Z_AXIS_COLOR );
  octree_coloring_property_->addOption( "Cell Probability",  OCTOMAP_PROBABLILTY_COLOR );
  alpha_property_ = new rviz::FloatProperty( "Voxel Alpha", 1.0, "Set voxel transparency alpha",
                                             this, 
                                             SLOT( updateAlpha() ) );
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  tree_depth_property_ = new IntProperty("Max. Octree Depth",
                                         max_octree_depth_,
                                         "Defines the maximum tree depth",
                                         this,
                                         SLOT (updateTreeDepth() ));
  tree_depth_property_->setMin(0);

  max_height_property_ = new FloatProperty("Max. Height Display",
                                           std::numeric_limits<double>::infinity(),
                                           "Defines the maximum height to display",
                                           this,
                                           SLOT (updateMaxHeight() ));

  min_height_property_ = new FloatProperty("Min. Height Display",
                                           -std::numeric_limits<double>::infinity(),
                                           "Defines the minimum height to display",
                                           this,
                                           SLOT (updateMinHeight() ));
}

void OccupancyGridDisplay::onInitialize()
{
  boost::mutex::scoped_lock lock(mutex_);

  box_size_.resize(max_octree_depth_);
  cloud_.resize(max_octree_depth_);
  point_buf_.resize(max_octree_depth_);
  new_points_.resize(max_octree_depth_);

  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    std::stringstream sname;
    sname << "PointCloud Nr." << i;
    cloud_[i] = new rviz::PointCloud();
    cloud_[i]->setName(sname.str());
    cloud_[i]->setRenderMode(rviz::PointCloud::RM_BOXES);
    scene_node_->attachObject(cloud_[i]);
  }
}

OccupancyGridDisplay::~OccupancyGridDisplay()
{
  std::size_t i;

  unsubscribe();

  for (std::vector<rviz::PointCloud*>::iterator it = cloud_.begin(); it != cloud_.end(); ++it) {
    delete *(it);
  }

  if (scene_node_)
    scene_node_->detachAllObjects();
}

void OccupancyGridDisplay::updateQueueSize()
{
  queue_size_ = queue_size_property_->getInt();

  subscribe();
}

void OccupancyGridDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void OccupancyGridDisplay::onDisable()
{
  scene_node_->setVisible(false);
  unsubscribe();

  clear();
}

void OccupancyGridDisplay::subscribe()
{
  if (!isEnabled())
  {
    return;
  }

  try
  {
    unsubscribe();

    // Subscribe to map topic
    const std::string& mapTopicStr = octomap_topic_property_->getStdString();

    if (!mapTopicStr.empty())
    {

      map_sub_.reset(new message_filters::Subscriber<octomap_msgs::Octomap>());

      map_sub_->subscribe(threaded_nh_, mapTopicStr, queue_size_);
      map_sub_->registerCallback(boost::bind(&OccupancyGridDisplay::incomingMapMessageCallback, this, _1));

    }

    // Subscribe to update topic
    const std::string& updateTopicStr = octomap_topic_property_->getStdString() + "_updates";

    if (!updateTopicStr.empty())
    {

      update_sub_.reset(new message_filters::Subscriber<octomap_msgs::OctomapUpdate>());

      update_sub_->subscribe(threaded_nh_, updateTopicStr, queue_size_);
      update_sub_->registerCallback(boost::bind(&OccupancyGridDisplay::incomingUpdateMessageCallback, this, _1));

    }
  }
  catch (ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Topic", (std::string("Error subscribing: ") + e.what()).c_str());
  }

}

void OccupancyGridDisplay::unsubscribe()
{
  boost::mutex::scoped_lock lock(mutex_);
  clear();

  try
  {
    // reset filters
    map_sub_.reset();
    update_sub_.reset();
    using_updates = false;
  }
  catch (ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Topic", (std::string("Error unsubscribing: ") + e.what()).c_str());
  }

}

// method taken from octomap_server package
void OccupancyGridDisplay::setColor(double z_pos, double min_z, double max_z, double color_factor,
                                    rviz::PointCloud::Point& point)
{
  int i;
  double m, n, f;

  double s = 1.0;
  double v = 1.0;

  double h = (1.0 - std::min(std::max((z_pos - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor;

  h -= floor(h);
  h *= 6;
  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      point.setColor(v, n, m);
      break;
    case 1:
      point.setColor(n, v, m);
      break;
    case 2:
      point.setColor(m, v, n);
      break;
    case 3:
      point.setColor(m, n, v);
      break;
    case 4:
      point.setColor(n, m, v);
      break;
    case 5:
      point.setColor(v, m, n);
      break;
    default:
      point.setColor(1, 0.5, 0.5);
      break;
  }
}

void OccupancyGridDisplay::updateTreeDepth()
{
  updateTopic();
}

void OccupancyGridDisplay::updateOctreeRenderMode()
{
  updateTopic();
}

void OccupancyGridDisplay::updateOctreeColorMode()
{
  updateTopic();
}

void OccupancyGridDisplay::updateAlpha()
{
  updateTopic();
}

void OccupancyGridDisplay::updateMaxHeight()
{
  updateTopic();
}

void OccupancyGridDisplay::updateMinHeight()
{
  updateTopic();
}

void OccupancyGridDisplay::clear()
{

  boost::mutex::scoped_lock lock(mutex_);

  // reset rviz pointcloud boxes
  for (size_t i = 0; i < cloud_.size(); ++i)
  {
    cloud_[i]->clear();
  }
}

void OccupancyGridDisplay::update(float wall_dt, float ros_dt)
{
  if (new_points_received_)
  {
    boost::mutex::scoped_lock lock(mutex_);

    for (size_t i = 0; i < max_octree_depth_; ++i)
    {
      double size = box_size_[i];

      cloud_[i]->clear();
      cloud_[i]->setDimensions(size, size, size);
      cloud_[i]->addPoints(&new_points_[i].front(), new_points_[i].size());

      new_points_[i].clear();
      cloud_[i]->setAlpha(alpha_property_->getFloat());
    }
    new_points_received_ = false;
    new_map_update_received_ = false;
  }
  updateFromTF();
  context_->queueRender();
}

void OccupancyGridDisplay::reset()
{
  clear();
  maps_received_ = 0;
  setStatus(StatusProperty::Ok, "Messages", QString("0 binary octomap messages received"));
}

void OccupancyGridDisplay::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

template <typename OcTreeType>
bool TemplatedOccupancyGridDisplay<OcTreeType>::checkType(std::string type_id)
{
  //General case: Need to be specialized for every used case
  setStatus(StatusProperty::Warn, "Messages", QString("Cannot verify octomap type"));
  return true; //Try deserialization, might crash though
}
  
template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTreeStamped>::checkType(std::string type_id)
{
  if(type_id == "OcTreeStamped") return true;
  else return false;
}
template <>
bool TemplatedOccupancyGridDisplay<octomap::OcTree>::checkType(std::string type_id)
{
  if(type_id == "OcTree") return true;
  else return false;
}

template <>
bool TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::checkType(std::string type_id)
{
  if(type_id == "ColorOcTree") return true;
  else return false;
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::setVoxelColor(PointCloud::Point& newPoint, 
                                                              typename OcTreeType::NodeType& node,
                                                              double minZ, double maxZ)
{
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  float cell_probability;
  switch (octree_color_mode)
  {
    case OCTOMAP_CELL_COLOR:
      setStatus(StatusProperty::Error, "Messages", QString("Cannot extract color"));
      //Intentional fall-through for else-case
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}

//Specialization for ColorOcTreeNode, which can set the voxel color from the node itself
template <>
void TemplatedOccupancyGridDisplay<octomap::ColorOcTree>::setVoxelColor(PointCloud::Point& newPoint, 
                                                                      octomap::ColorOcTree::NodeType& node,
                                                                      double minZ, double maxZ)
{
  float cell_probability;
  OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
  switch (octree_color_mode)
  {
    case OCTOMAP_CELL_COLOR:
    {
      const float b2f = 1./256.; 
      octomap::ColorOcTreeNode::Color& color = node.getColor();
      newPoint.setColor(b2f*color.r, b2f*color.g, b2f*color.b, node.getOccupancy());
      break;
    }
    case OCTOMAP_Z_AXIS_COLOR:
      setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
      break;
    case OCTOMAP_PROBABLILTY_COLOR:
      cell_probability = node.getOccupancy();
      newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
      break;
    default:
      break;
  }
}


bool OccupancyGridDisplay::updateFromTF()
{
    // get tf transform
    Ogre::Vector3 pos;
    Ogre::Quaternion orient;
    if (!context_->getFrameManager()->getTransform(header_, pos, orient)) {
      return false;
    }

    scene_node_->setOrientation(orient);
    scene_node_->setPosition(pos);
    return true;
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::incomingUpdateMessageCallback(const octomap_msgs::OctomapUpdateConstPtr& msg)
{
  if(oc_tree_)
  {
    // creating octree
    map_updates_received_++;
    setStatus(StatusProperty::Ok, "Messages", QString::number(map_updates_received_) + " octomap updates received");
    setStatusStd(StatusProperty::Ok, "Type", msg->octomap_bounds.id.c_str());
    if(!checkType(msg->octomap_bounds.id)){
      setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
      return;
    }

    header_ = msg->header;
    if (!updateFromTF()) {
        std::stringstream ss;
        ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
            << context_->getFrameManager()->getFixedFrame() << "]";
        setStatusStd(StatusProperty::Error, "Message", ss.str());
        return;
    }

    // Get update data
    OcTreeType* update_bounds = NULL;
    OcTreeType* update_values = NULL;
    octomap::AbstractOcTree* bounds_tree = octomap_msgs::msgToMap(msg->octomap_bounds);
    octomap::AbstractOcTree* value_tree = octomap_msgs::msgToMap(msg->octomap_update);
    if (bounds_tree && value_tree){
      update_bounds = dynamic_cast<OcTreeType*>(bounds_tree);
      update_values = dynamic_cast<OcTreeType*>(value_tree);
      if(!update_bounds || !update_values){
        setStatusStd(StatusProperty::Error, "Message", "Wrong octomap_update type. Use a different display type.");
      }
    }
    else
    {
      setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize octree message.");
      return;
    }

    // Merge new tree into internal tree
    {
      boost::mutex::scoped_lock lock(mutex_);
      using_updates = true;
      oc_tree_->setTreeValues(update_values, update_bounds, false, true);
    }
    delete update_bounds;
    delete update_values;
    new_map_update_received_ = true;
    updateNewPoints();
  }
}

template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::incomingMapMessageCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  if(!using_updates){
    ++maps_received_;
    map_updates_received_ = 0;
    setStatus(StatusProperty::Ok, "Messages", QString::number(maps_received_) + " octomap messages received");
    setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
    if(!checkType(msg->id)){
      setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
      return;
    }

    ROS_DEBUG("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

    header_ = msg->header;
    if (!updateFromTF()) {
        std::stringstream ss;
        ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
            << context_->getFrameManager()->getFixedFrame() << "]";
        setStatusStd(StatusProperty::Error, "Message", ss.str());
        return;
    }


    // creating octree
    if(oc_tree_)
      delete oc_tree_;
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree){
      oc_tree_ = dynamic_cast<OcTreeType*>(tree);
      if(!oc_tree_){
        setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
      }
    }
    else
    {
      setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize octree message.");
      return;
    }

    // reset rviz pointcloud classes
    for (std::size_t i = 0; i < max_octree_depth_; ++i)
    {
      point_buf_[i].clear();
      box_size_[i] = oc_tree_->getNodeSize(i + 1);
    }

    updateNewPoints();
  }
  else
  {
    map_sub_->unsubscribe();
  }
}


template <typename OcTreeType>
void TemplatedOccupancyGridDisplay<OcTreeType>::updateNewPoints()
{
  tree_depth_property_->setMax(oc_tree_->getTreeDepth());

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  oc_tree_->getMetricMin(minX, minY, minZ);
  oc_tree_->getMetricMax(maxX, maxY, maxZ);

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(tree_depth_property_->getInt(), oc_tree_->getTreeDepth());
    double maxHeight = std::min<double>(max_height_property_->getFloat(), maxZ);
    double minHeight = std::max<double>(min_height_property_->getFloat(), minZ);
    int stepSize = 1 << (oc_tree_->getTreeDepth() - treeDepth); // for pruning of occluded voxels
    for (typename OcTreeType::iterator it = oc_tree_->begin(treeDepth), end = oc_tree_->end(); it != end; ++it)
    {

        if(it.getZ() <= maxHeight && it.getZ() >= minHeight)
        {
          int render_mode_mask = octree_render_property_->getOptionInt();

          bool display_voxel = false;

          // the left part evaluates to 1 for free voxels and 2 for occupied voxels
          if (((int)oc_tree_->isNodeOccupied(*it) + 1) & render_mode_mask)
          {
            // check if current voxel has neighbors on all sides -> no need to be displayed
            bool allNeighborsFound = true;

            octomap::OcTreeKey key;
            octomap::OcTreeKey nKey = it.getKey();

            // determine indices of potentially neighboring voxels for depths < maximum tree depth
            // +/-1 at maximum depth, +2^(depth_difference-1) and -2^(depth_difference-1)-1 on other depths
            int diffBase = (it.getDepth() < oc_tree_->getTreeDepth()) ? 1 << (oc_tree_->getTreeDepth() - it.getDepth() - 1) : 1;
            int diff[2] = {-((it.getDepth() == oc_tree_->getTreeDepth()) ? diffBase : diffBase + 1), diffBase};

            // cells with adjacent faces can occlude a voxel, iterate over the cases x,y,z (idxCase) and +/- (diff)
            for (unsigned int idxCase = 0; idxCase < 3; ++idxCase)
            {
              int idx_0 = idxCase % 3;
              int idx_1 = (idxCase + 1) % 3;
              int idx_2 = (idxCase + 2) % 3;

              for (int i = 0; allNeighborsFound && i < 2; ++i)
              {
                key[idx_0] = nKey[idx_0] + diff[i];
                // if rendering is restricted to treeDepth < maximum tree depth inner nodes with distance stepSize can already occlude a voxel
                for (key[idx_1] = nKey[idx_1] + diff[0] + 1; allNeighborsFound && key[idx_1] < nKey[idx_1] + diff[1]; key[idx_1] += stepSize)
                {
                  for (key[idx_2] = nKey[idx_2] + diff[0] + 1; allNeighborsFound && key[idx_2] < nKey[idx_2] + diff[1]; key[idx_2] += stepSize)
                  {
                    typename OcTreeType::NodeType* node = oc_tree_->search(key, treeDepth);

                    // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                    if (!(node && ((((int)oc_tree_->isNodeOccupied(node)) + 1) & render_mode_mask)))
                    {
                      // we do not have a neighbor => break!
                      allNeighborsFound = false;
                    }
                  }
                }
              }
            }

            display_voxel |= !allNeighborsFound;
          }


          if (display_voxel)
          {
            PointCloud::Point newPoint;

            newPoint.position.x = it.getX();
            newPoint.position.y = it.getY();
            newPoint.position.z = it.getZ();



            setVoxelColor(newPoint, *it, minZ, maxZ);
            // push to point vectors
            unsigned int depth = it.getDepth();
            point_buf_[depth - 1].push_back(newPoint);

            ++pointCount;
          }
        }
    }
  }

  if (pointCount)
  {
    boost::mutex::scoped_lock lock(mutex_);

    new_points_received_ = true;

    for (size_t i = 0; i < max_octree_depth_; ++i)
      new_points_[i].swap(point_buf_[i]);

  }
}

} // namespace octomap_rviz_plugin

#include <pluginlib/class_list_macros.h>

typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTree> OcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::ColorOcTree> ColorOcTreeGridDisplay;
typedef octomap_rviz_plugin::TemplatedOccupancyGridDisplay<octomap::OcTreeStamped> OcTreeStampedGridDisplay;

PLUGINLIB_EXPORT_CLASS( OcTreeGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS( ColorOcTreeGridDisplay, rviz::Display)
PLUGINLIB_EXPORT_CLASS( OcTreeStampedGridDisplay, rviz::Display)

