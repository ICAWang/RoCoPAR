/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#ifndef MAP_MERGE_H_
#define MAP_MERGE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <combine_grids/merging_pipeline.h>
#include <geometry_msgs/Pose.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <rocopar_map_merge/MapMergeService.h>

namespace map_merge
{
struct MapSubscription {
  // protects consistency of writable_map and readonly_map
  // also protects reads and writes of shared_ptrs
  std::mutex mutex;
  std::string robot_name;

  geometry_msgs::Transform initial_pose;
  nav_msgs::OccupancyGrid::Ptr writable_map;
  nav_msgs::OccupancyGrid::ConstPtr readonly_map;
  double x;
  double y;
  double z;
  ros::Time odom_stamp=ros::Time().fromSec(0);

  ros::Subscriber map_sub;
  ros::Subscriber map_updates_sub;
  ros::Subscriber odom_sub;
};

class MapMerge
{
private:
  ros::NodeHandle node_;

  /* parameters */
  double merging_rate_;
  double discovery_rate_;
  double estimation_rate_;
  double confidence_threshold_;
  double max_com_dis;
  double map_origin_x;
  double map_origin_y;
  int robot_id;
  std::string frame_id;
  std::string my_name;
  std::string robot_map_topic_;
  std::string merged_map_topic;
  std::string robot_map_updates_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  bool have_initial_poses_;
  nav_msgs::OccupancyGrid::Ptr my_map=nullptr;
  geometry_msgs::Transform my_initial_pose;
  

  // publishing
  ros::Publisher merged_map_publisher_; 
  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  boost::shared_mutex subscriptions_mutex_;
  combine_grids::MergingPipeline pipeline_;
  std::mutex pipeline_mutex_;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const ros::master::TopicInfo& topic);
  bool getInitPose(const std::string& name, geometry_msgs::Transform& pose);

  void fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                     MapSubscription& map);
  void partialMapUpdate(const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
                        MapSubscription& map);
  void odomUpdate(const nav_msgs::Odometry::ConstPtr& msg,
                        MapSubscription& map);

public:
  MapMerge();

  bool mapMergeCallback(rocopar_map_merge::MapMergeService::Request &req,
                        rocopar_map_merge::MapMergeService::Response &res);
  void spin();
  void executetopicSubscribing();
  void executemapMerging();
  void executeposeEstimation();
  void executemapMergingService();

  void topicSubscribing();
  nav_msgs::OccupancyGridPtr mapMerging();
  /**
   * @brief Estimates initial positions of grids
   * @details Relevant only if initial poses are not known
   */
  void poseEstimation();
};

}  // namespace map_merge

#endif /* MAP_MERGE_H_ */
