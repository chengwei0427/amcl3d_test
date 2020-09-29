/*!
 * @file Node.cpp
 * @copyright Copyright (c) 2019, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Node.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/Marker.h>



namespace amcl3d
{
Node::Node(const std::string& str) : WORKING_DIR(str), nh_(ros::this_node::getName())
,g_log(new VSCOMMON::Logger("MAIN")),loc_loop_(0),grid3d_(new Grid3d(g_log)), pf_(new ParticleFilter(g_log))
{
  using namespace VSCOMMON;
  std::cout<<WORKING_DIR+"log"<<std::endl;
  createDir(WORKING_DIR+"log");
  createDir(WORKING_DIR+"log/amcl3d");
  std::string logConfig = bin_dir + "params/log4cpp_amcl3d.conf";
  readLog4cppConfigure(logConfig);
  LOG_INFO(g_log, "TOOL_VERSION= " << TOOL_VERSION << " build:" << __DATE__ << " " << __TIME__ );
  readParamFromXML();
  LOG_INFO(g_log, "Node initialized successful.");
}

Node::~Node()
{
  LOG_INFO(g_log, "Node::~Node()");
}

void Node::spin()
{
  LOG_INFO(g_log,"starting Node::spin()");

  if (!grid3d_->open(parameters_.map_path_, parameters_.sensor_dev_))
    return;

  if (parameters_.publish_grid_slice_rate_ != 0 &&
      grid3d_->buildGridSliceMsg(parameters_.grid_slice_z_, grid_slice_msg_))
  {
    grid_slice_msg_.header.frame_id = parameters_.global_frame_id_;
    grid_slice_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_slice", 1, true);
    grid_slice_pub_timer_ =
        nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_grid_slice_rate_)), &Node::publishGridSlice, this);
  }

  if (parameters_.publish_point_cloud_rate_ != 0 && grid3d_->buildMapPointCloudMsg(map_point_cloud_msg_))
  {
    map_point_cloud_msg_.header.frame_id = parameters_.global_frame_id_;
    map_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1, true);
    map_point_cloud_pub_timer_ = nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_point_cloud_rate_)),
                                                 &Node::publishMapPointCloud, this);
  }

  point_sub_ = nh_.subscribe("/laser_sensor", 1, &Node::pointcloudCallback, this);
  odom_sub_ = nh_.subscribe("/odometry", 1, &Node::odomCallback, this);
  initialPose_sub_ = nh_.subscribe("/initialpose",1,&Node::initialPoseCallback,this);

  //range_sub_ = nh_.subscribe("/radiorange_sensor", 1, &Node::rangeCallback, this);

  particles_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);
  //range_markers_pub_ = nh_.advertise<visualization_msgs::Marker>("range", 0);
  odom_base_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("base_transform", 1);

  cloud_filter_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_filtered", 0);

  tf_broadcaster.reset(new tf::TransformBroadcaster());

  while (ros::ok())
  {
    ros::spinOnce();
    usleep(100);
  }

  nh_.shutdown();
}

void Node::readParamFromXML()
{
  using namespace std;
  using namespace VSCOMMON;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string global_frame_id;
  std::string map_path; 
  bool set_initial_pose;
  double init_x;
  double init_y; 
  double init_z;
  double init_a;

  double init_x_dev,init_y_dev,init_z_dev,init_a_dev;

  double grid_slice_z;
  double publish_point_cloud_rate;
  double publish_grid_slice_rate;

  double sensor_dev;
  double sensor_range;
  double voxel_size;

  double num_particles;

  double odom_x_mod,odom_y_mod,odom_z_mod,odom_a_mod;

  int resample_interval;

  double update_rate;
  double d_th;
  double a_th;
  double take_off_height;
  double alpha;

  LOAD_PARAM_FROM_XML(path_param.c_str());
  DECLARE_PARAM_READER_BEGIN(AMCL3D)
  READ_PARAM(base_frame_id)
  READ_PARAM(odom_frame_id)
  READ_PARAM(global_frame_id)
  READ_PARAM(map_path)
  READ_PARAM(set_initial_pose)
  READ_PARAM(init_x)
  READ_PARAM(init_y)
  READ_PARAM(init_z)
  READ_PARAM(init_a)
  READ_PARAM(init_x_dev)
  READ_PARAM(init_y_dev)
  READ_PARAM(init_z_dev)
  READ_PARAM(init_a_dev)
  READ_PARAM(grid_slice_z)
  READ_PARAM(publish_point_cloud_rate)
  READ_PARAM(publish_grid_slice_rate)
  READ_PARAM(sensor_dev)
  READ_PARAM(sensor_range)
  READ_PARAM(voxel_size)
  READ_PARAM(num_particles)
  READ_PARAM(odom_x_mod)
  READ_PARAM(odom_y_mod)
  READ_PARAM(odom_z_mod)
  READ_PARAM(odom_a_mod)
  READ_PARAM(resample_interval)
  READ_PARAM(update_rate)
  READ_PARAM(d_th)
  READ_PARAM(a_th)
  READ_PARAM(take_off_height)
  READ_PARAM(alpha)
    DECLARE_PARAM_READER_END

  parameters_.base_frame_id_ = base_frame_id;
  parameters_.odom_frame_id_ = odom_frame_id;
  parameters_.global_frame_id_ = global_frame_id;
  parameters_.map_path_ = map_path;
  parameters_.set_initial_pose_ = set_initial_pose;
  parameters_.init_x_ = init_x;
  parameters_.init_y_ = init_y;
  parameters_.init_z_ = init_z;
  parameters_.init_a_ = init_a;
  parameters_.init_x_dev_ = init_x_dev;
  parameters_.init_y_dev_ = init_y_dev;
  parameters_.init_z_dev_ = init_z_dev;
  parameters_.init_a_dev_ = init_a_dev;
  parameters_.grid_slice_z_ = grid_slice_z;
  parameters_.publish_point_cloud_rate_ = publish_point_cloud_rate;
  parameters_.publish_grid_slice_rate_ = publish_grid_slice_rate;
  parameters_.sensor_dev_ = sensor_dev;
  parameters_.sensor_range_ = sensor_range;
  parameters_.voxel_size_ = voxel_size;
  parameters_.num_particles_ = num_particles;
  parameters_.odom_x_mod_ = odom_x_mod;
  parameters_.odom_y_mod_ = odom_y_mod;
  parameters_.odom_z_mod_ = odom_z_mod;
  parameters_.odom_a_mod_ = odom_a_mod;
  parameters_.resample_interval_ = resample_interval;
  parameters_.update_rate_ = update_rate;
  parameters_.d_th_ = d_th;
  parameters_.a_th_ = a_th;
  parameters_.take_off_height_ = take_off_height;
  parameters_.alpha_ = alpha;
}

void Node::publishMapPointCloud(const ros::TimerEvent&)
{
  //LOG_INFO(g_log,__FUNCTION__<<" Node::publishMapPointCloud()");

  map_point_cloud_msg_.header.stamp = ros::Time::now();
  map_point_cloud_pub_.publish(map_point_cloud_msg_);
}

void Node::publishGridSlice(const ros::TimerEvent&)
{
  //LOG_INFO(g_log,__FUNCTION__<<" Node::publishGridSlice()");

  grid_slice_msg_.header.stamp = ros::Time::now();
  grid_slice_pub_.publish(grid_slice_msg_);
}

void Node::publishParticles()
{
  /* If the filter is not initialized then exit */
  if (!pf_->isInitialized())
    return;

  /* Build the msg based on the particles position and orientation */
  geometry_msgs::PoseArray msg;
  pf_->buildParticlesPoseMsg(msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parameters_.global_frame_id_;

  /* Publish particle cloud */
  particles_pose_pub_.publish(msg);
}

void Node::publishPoseTransfrom(ros::Time& stamp)
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(mean_p_.x, mean_p_.y, mean_p_.z) );
  tf::Quaternion q;
  q.setRPY(0, 0, mean_p_.a);
  transform.setRotation(q);
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "world", "velodyne"));
}

void Node::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //LOG_INFO(g_log,"pointcloudCallback open");

  if (!is_odom_)
  {
    using namespace VSCOMMON;
    LOG_COUT_WARN(g_log,__FUNCTION__,"Odometry transform not received");
    return;
  }

  /* Check if an update must be performed or not */
  if (!checkUpdateThresholds())
    return;
  LOG_INFO(g_log,"Begin process frame. loop:" << loc_loop_++);
  static const ros::Duration update_interval(1.0 / parameters_.update_rate_);
  nextupdate_time_ = ros::Time::now() + update_interval;

  /* Apply voxel grid */
  VSCOMMON::tic("Filter");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud_src);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_src);
  sor.setLeafSize(parameters_.voxel_size_, parameters_.voxel_size_, parameters_.voxel_size_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  sor.filter(*cloud_down);
  cloud_down->header = cloud_src->header;
  sensor_msgs::PointCloud2 cloud_down_msg;
  pcl::toROSMsg(*cloud_down, cloud_down_msg);
  cloud_filter_pub_.publish(cloud_down_msg);
  LOG_INFO(g_log,"Filter time:"<<VSCOMMON::toc("Filter") * 1000<<" ms");

  /* Perform particle prediction based on odometry */
  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;
  const double delta_x = odom_increment_tf_.getOrigin().getX();
  const double delta_y = odom_increment_tf_.getOrigin().getY();
  const double delta_z = odom_increment_tf_.getOrigin().getZ();
  const double delta_a = getYawFromTf(odom_increment_tf_);

  VSCOMMON::tic("Predict");
  pf_->predict(parameters_.odom_x_mod_, parameters_.odom_y_mod_, parameters_.odom_z_mod_, parameters_.odom_a_mod_,
              delta_x, delta_y, delta_z, delta_a);
  LOG_INFO(g_log,"Predict time:"<<VSCOMMON::toc("Predict") * 1000<<" ms");

  /* Perform particle update based on current point-cloud */
  VSCOMMON::tic("Update");
  pf_->update(grid3d_, cloud_down, range_data, parameters_.alpha_, parameters_.sensor_range_, roll_, pitch_);
  LOG_INFO(g_log,"Update time:"<<VSCOMMON::toc("Update") * 1000<<" ms");

  mean_p_ = pf_->getMean();
  ros::Time pf_stamp = msg->header.stamp;
  // publishPoseTransfrom(pf_stamp);
  /* Clean the range buffer */
  range_data.clear();

  /* Update time and transform information */
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  /* Do the resampling if needed */
  VSCOMMON::tic("Resample");
  static int n_updates = 0;
  if (++n_updates > parameters_.resample_interval_)
  {
    n_updates = 0;
    pf_->resample();
  }
  LOG_INFO(g_log,"Resample time:"<<VSCOMMON::toc("Update") * 1000<<" ms");

  /* Publish particles */
  publishParticles();

  //LOG_INFO(g_log,"pointcloudCallback close");
}

/*void Node::odomCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
  ROS_DEBUG("odomCallback open");

  base_2_odom_tf_.setOrigin(
      tf::Vector3(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  base_2_odom_tf_.setRotation(tf::Quaternion(msg->transform.rotation.x, msg->transform.rotation.y,
                                             msg->transform.rotation.z, msg->transform.rotation.w));

  // If the filter is not initialized then exit 
  if (!pf_.isInitialized())
  {
    ROS_WARN("Filter not initialized yet, waiting for initial pose.");
    if (parameters_.set_initial_pose_)
    {
      tf::Transform init_pose;
      init_pose.setOrigin(tf::Vector3(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_));
      init_pose.setRotation(tf::Quaternion(0.0, 0.0, sin(parameters_.init_a_ * 0.5), cos(parameters_.init_a_ * 0.5)));
      setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                     parameters_.init_a_dev_);
    }
    return;
  }

  // Update roll and pitch from odometry 
  double yaw;
  base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(base_2_odom_tf_, ros::Time::now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));

  if (!is_odom_)
  {
    is_odom_ = true;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;
  }

  static bool has_takenoff = false;
  if (!has_takenoff)
  {
    ROS_WARN("Not <<taken off>> yet");

    // Check takeoff height 
    has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.take_off_height_;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;

    lastmean_p_ = mean_p_;  // for not 'jumping' whenever has_takenoff is true 
  }
  else
  {
    // Check if AMCL went wrong (nan, inf) 
    if (std::isnan(mean_p_.x) || std::isnan(mean_p_.y) || std::isnan(mean_p_.z) || std::isnan(mean_p_.a))
    {
      ROS_WARN("AMCL NaN detected");
      amcl_out_ = true;
    }
    if (std::isinf(mean_p_.x) || std::isinf(mean_p_.y) || std::isinf(mean_p_.z) || std::isinf(mean_p_.a))
    {
      ROS_WARN("AMCL Inf detected");
      amcl_out_ = true;
    }

    // Check jumps 
    if (fabs(mean_p_.x - lastmean_p_.x) > 1.)
    {
      ROS_WARN("AMCL Jump detected in X");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Y");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Z");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
    {
      ROS_WARN("AMCL Jump detected in Yaw");
      amcl_out_ = true;
    }

    if (!amcl_out_)
    {
      tf::Transform base_2_world_tf;
      base_2_world_tf.setOrigin(tf::Vector3(mean_p_.x, mean_p_.y, mean_p_.z));
      tf::Quaternion q;
      q.setRPY(roll_, pitch_, mean_p_.a);
      base_2_world_tf.setRotation(q);

      base_2_world_tf = base_2_world_tf * lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

      lastmean_p_ = mean_p_;

      lastbase_2_world_tf_ = base_2_world_tf;
      lastodom_2_world_tf_ = base_2_world_tf * base_2_odom_tf_.inverse();

      amcl_out_lastbase_2_odom_tf_ = lastupdatebase_2_odom_tf_;
    }
    else
    {
      lastbase_2_world_tf_ = lastbase_2_world_tf_ * amcl_out_lastbase_2_odom_tf_.inverse() * base_2_odom_tf_;
      amcl_out_lastbase_2_odom_tf_ = base_2_odom_tf_;
    }
  }

  // Publish transform 
  geometry_msgs::TransformStamped odom_2_base_tf;
  odom_2_base_tf.header.stamp = msg->header.stamp;
  odom_2_base_tf.header.frame_id = parameters_.global_frame_id_;
  odom_2_base_tf.child_frame_id = parameters_.base_frame_id_;
  odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
  odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
  odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
  odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
  odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
  odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
  odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
  odom_base_pub_.publish(odom_2_base_tf);

  tf_br.sendTransform(tf::StampedTransform(lastodom_2_world_tf_, ros::Time::now(), parameters_.global_frame_id_,
                                           parameters_.odom_frame_id_));

  ROS_DEBUG("odomCallback close");
}*/

void Node::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  //LOG_INFO(g_log,"odomCallback open");

  base_2_odom_tf_.setOrigin(
      tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  base_2_odom_tf_.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

  /* If the filter is not initialized then exit */
  if (!pf_->isInitialized())
  {
    using namespace VSCOMMON;
    LOG_COUT_WARN(g_log,__FUNCTION__,"Filter not initialized yet, waiting for initial pose.");
    if (parameters_.set_initial_pose_)
    {
      tf::Transform init_pose;
      init_pose.setOrigin(tf::Vector3(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_));
      init_pose.setRotation(tf::Quaternion(0.0, 0.0, sin(parameters_.init_a_ * 0.5), cos(parameters_.init_a_ * 0.5)));

      LOG_COUT_INFO(g_log,__FUNCTION__<<" "<<__LINE__<<" : set initial pose here.");
      setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                     parameters_.init_a_dev_);
    }
    return;
  }

  /* Update roll and pitch from odometry */
  double yaw;
  base_2_odom_tf_.getBasis().getRPY(roll_, pitch_, yaw);

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(base_2_odom_tf_, ros::Time::now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));

  if (!is_odom_)
  {
    is_odom_ = true;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;
  }

  static bool has_takenoff = false;
  if (!has_takenoff)
  {
    using namespace VSCOMMON;
    LOG_COUT_WARN(g_log,__FUNCTION__,"Not <<taken off>> yet");

    /* Check takeoff height */
    has_takenoff = base_2_odom_tf_.getOrigin().getZ() > parameters_.take_off_height_;

    lastbase_2_world_tf_ = initodom_2_world_tf_ * base_2_odom_tf_;
    lastodom_2_world_tf_ = initodom_2_world_tf_;

    lastmean_p_ = mean_p_;  // for not 'jumping' whenever has_takenoff is true */
  }
  else
  {
    /* Check if AMCL went wrong (nan, inf) */
    if (std::isnan(mean_p_.x) || std::isnan(mean_p_.y) || std::isnan(mean_p_.z) || std::isnan(mean_p_.a))
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL NaN detected");
      amcl_out_ = true;
    }
    if (std::isinf(mean_p_.x) || std::isinf(mean_p_.y) || std::isinf(mean_p_.z) || std::isinf(mean_p_.a))
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL Inf detected");
      amcl_out_ = true;
    }

    /* Check jumps */
    if (fabs(mean_p_.x - lastmean_p_.x) > 1.)
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL Jump detected in X");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.y - lastmean_p_.y) > 1.)
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL Jump detected in Y");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.z - lastmean_p_.z) > 1.)
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL Jump detected in Z");
      amcl_out_ = true;
    }
    if (fabs(mean_p_.a - lastmean_p_.a) > 1.)
    {
      using namespace VSCOMMON;
      LOG_COUT_WARN(g_log,__FUNCTION__,"AMCL Jump detected in Yaw");
      amcl_out_ = true;
    }

    if (!amcl_out_)
    {
      tf::Transform base_2_world_tf;
      base_2_world_tf.setOrigin(tf::Vector3(mean_p_.x, mean_p_.y, mean_p_.z));
      tf::Quaternion q;
      q.setRPY(roll_, pitch_, mean_p_.a);
      base_2_world_tf.setRotation(q);

      base_2_world_tf = base_2_world_tf * lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

      lastmean_p_ = mean_p_;

      lastbase_2_world_tf_ = base_2_world_tf;
      lastodom_2_world_tf_ = base_2_world_tf * base_2_odom_tf_.inverse();

      amcl_out_lastbase_2_odom_tf_ = lastupdatebase_2_odom_tf_;
    }
    else
    {
      lastbase_2_world_tf_ = lastbase_2_world_tf_ * amcl_out_lastbase_2_odom_tf_.inverse() * base_2_odom_tf_;
      amcl_out_lastbase_2_odom_tf_ = base_2_odom_tf_;
    }
  }

  /* Publish transform */
  geometry_msgs::TransformStamped odom_2_base_tf;
  odom_2_base_tf.header.stamp = msg->header.stamp;
  odom_2_base_tf.header.frame_id = parameters_.global_frame_id_;
  odom_2_base_tf.child_frame_id = parameters_.base_frame_id_;
  odom_2_base_tf.transform.translation.x = lastbase_2_world_tf_.getOrigin().getX();
  odom_2_base_tf.transform.translation.y = lastbase_2_world_tf_.getOrigin().getY();
  odom_2_base_tf.transform.translation.z = lastbase_2_world_tf_.getOrigin().getZ();
  odom_2_base_tf.transform.rotation.x = lastbase_2_world_tf_.getRotation().getX();
  odom_2_base_tf.transform.rotation.y = lastbase_2_world_tf_.getRotation().getY();
  odom_2_base_tf.transform.rotation.z = lastbase_2_world_tf_.getRotation().getZ();
  odom_2_base_tf.transform.rotation.w = lastbase_2_world_tf_.getRotation().getW();
  odom_base_pub_.publish(odom_2_base_tf);

  tf_br.sendTransform(tf::StampedTransform(lastodom_2_world_tf_, ros::Time::now(), parameters_.global_frame_id_,
                                           parameters_.odom_frame_id_));

  //LOG_INFO(g_log,"odomCallback close");
}

void Node::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
      tf::Transform init_pose;
      init_pose.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
      init_pose.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
       msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
      using namespace VSCOMMON;
      LOG_COUT_INFO(g_log,__FUNCTION__<<" "<<__LINE__<<" : get input initial pose here.");
      setInitialPose(init_pose, parameters_.init_x_dev_, parameters_.init_y_dev_, parameters_.init_z_dev_,
                     parameters_.init_a_dev_);
}

/*void Node::rangeCallback(const rosinrange_msg::RangePoseConstPtr& msg)
{
  ROS_DEBUG("rangeCallback open");

  geometry_msgs::Point anchor;
  anchor.x = msg->position.x;
  anchor.y = msg->position.y;
  anchor.z = msg->position.z;

  range_data.push_back(Range(static_cast<float>(msg->range), msg->position.x, msg->position.y, msg->position.z));

  geometry_msgs::Point uav;
  uav.x = mean_p_.x;
  uav.y = mean_p_.y;
  uav.z = mean_p_.z;

  rvizMarkerPublish(msg->source_id, static_cast<float>(msg->range), uav, anchor);

  ROS_DEBUG("rangeCallback close");
}*/

bool Node::checkUpdateThresholds()
{
  // LOG_INFO(g_log,"Checking for AMCL3D update");
  static bool b_check_first = true;
  if(b_check_first)
    {
      b_check_first = false;
      return true;
    }

  if (ros::Time::now() < nextupdate_time_)
    return false;

  odom_increment_tf_ = lastupdatebase_2_odom_tf_.inverse() * base_2_odom_tf_;

  /* Check translation threshold */
  if (odom_increment_tf_.getOrigin().length() > parameters_.d_th_)
  {
    //ROS_INFO("Translation update");
    return true;
  }

  /* Check yaw threshold */
  double yaw, pitch, roll;
  odom_increment_tf_.getBasis().getRPY(roll, pitch, yaw);
  if (fabs(yaw) > parameters_.a_th_)
  {
    //ROS_INFO("Rotation update");
    return true;
  }

  return false;
}

void Node::setInitialPose(const tf::Transform& init_pose, const float x_dev, const float y_dev, const float z_dev,
                          const float a_dev)
{
  initodom_2_world_tf_ = init_pose;

  const tf::Vector3 t = init_pose.getOrigin();

  const float x_init = t.x();
  const float y_init = t.y();
  const float z_init = t.z();
  const float a_init = static_cast<float>(getYawFromTf(init_pose));

  pf_->init(parameters_.num_particles_, x_init, y_init, z_init, a_init, x_dev, y_dev, z_dev, a_dev);

  mean_p_ = pf_->getMean();
  lastmean_p_ = mean_p_;

  /* Extract TFs for future updates */
  /* Reset lastupdatebase_2_odom_tf_ */
  lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  /* Publish particles */
  publishParticles();
}

double Node::getYawFromTf(const tf::Transform& tf)
{
  double yaw, pitch, roll;
  tf.getBasis().getRPY(roll, pitch, yaw);

  return yaw;
}

void Node::rvizMarkerPublish(const uint32_t anchor_id, const float r, const geometry_msgs::Point& uav,
                             const geometry_msgs::Point& anchor)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = parameters_.global_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = "amcl3d";
  marker.id = anchor_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = r;
  marker.scale.z = r;
  marker.color.a = 0.5;
  if (amcl_out_) /* Indicate if AMCL was lost */
  {
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
  }
  else
  {
    switch (anchor_id)
    {
      case 1:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case 2:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case 3:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
    }
  }
  marker.points.clear();
  marker.points.push_back(uav);
  marker.points.push_back(anchor);

  /* Publish marker */
  range_markers_pub_.publish(marker);
}

}  // namespace amcl3d
