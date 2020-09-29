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



namespace amcl3d
{
Node::Node(const std::string& str) : WORKING_DIR(str), nh_(ros::this_node::getName())
,g_log(new VSCOMMON::Logger("MAIN")),loc_loop_(0),grid3d_(new Grid3d(g_log)), mcl_(new MonteCarloLocalization(g_log,grid3d_))
{
  using namespace VSCOMMON;
  std::cout<<WORKING_DIR+"log"<<std::endl;
  createDir(WORKING_DIR+"log");
  createDir(WORKING_DIR+"log/amcl3d");
  std::string logConfig = bin_dir + "params/log4cpp_amcl3d.conf";
  readLog4cppConfigure(logConfig);
  LOG_INFO(g_log, "TOOL_VERSION= " << TOOL_VERSION << " build:" << __DATE__ << " " << __TIME__ );
  readParamFromXML();
  mcl_->setParams(amcl_params_);
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
      buildGridSliceMsg(parameters_.grid_slice_z_, grid_slice_msg_))
  {
    grid_slice_msg_.header.frame_id = parameters_.global_frame_id_;
    grid_slice_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("grid_slice", 1, true);
    grid_slice_pub_timer_ =
        nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_grid_slice_rate_)), &Node::publishGridSlice, this);
  }

  if (parameters_.publish_point_cloud_rate_ != 0 && buildMapPointCloudMsg(map_point_cloud_msg_))
  {
    map_point_cloud_msg_.header.frame_id = parameters_.global_frame_id_;
    map_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1, true);
    map_point_cloud_pub_timer_ = nh_.createTimer(ros::Duration(ros::Rate(parameters_.publish_point_cloud_rate_)),
                                                 &Node::publishMapPointCloud, this);
  }

  point_sub_ = nh_.subscribe("/laser_sensor", 1, &Node::pointcloudCallback, this);
  odom_sub_ = nh_.subscribe("/odometry", 1, &Node::odomCallback, this);
  initialPose_sub_ = nh_.subscribe("/initialpose",1,&Node::initialPoseCallback,this);

  particles_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particle_cloud", 1, true);

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
  double voxel_size;

  double num_particles;

  double odom_x_mod,odom_y_mod,odom_z_mod,odom_a_mod;

  int resample_interval;

  double update_rate;
  double d_th;
  double a_th;

  //  amcl3d param
  int min_particle_num;
  int max_particle_num_global;
  int max_particle_num_local;

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
  READ_PARAM(min_particle_num)
  READ_PARAM(max_particle_num_global)
  READ_PARAM(max_particle_num_local)
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

  parameters_.grid_slice_z_ = grid_slice_z;
  parameters_.publish_point_cloud_rate_ = publish_point_cloud_rate;
  parameters_.publish_grid_slice_rate_ = publish_grid_slice_rate;
  parameters_.sensor_dev_ = sensor_dev;
  parameters_.voxel_size_ = voxel_size;
  parameters_.num_particles_ = num_particles;
  parameters_.resample_interval_ = resample_interval;
  parameters_.update_rate_ = update_rate;
  parameters_.d_th_ = d_th;
  parameters_.a_th_ = a_th;

  amcl_params_.min_particle_num = min_particle_num;
  amcl_params_.max_particle_num_global = max_particle_num_global;
  amcl_params_.max_particle_num_local = max_particle_num_local;
  amcl_params_.init_x_dev_ = init_x_dev;
  amcl_params_.init_y_dev_ = init_y_dev;
  amcl_params_.init_z_dev_ = init_z_dev;
  amcl_params_.init_a_dev_ = init_a_dev;
  amcl_params_.odom_x_mod_ = odom_x_mod;
  amcl_params_.odom_y_mod_ = odom_y_mod;
  amcl_params_.odom_z_mod_ = odom_z_mod;
  amcl_params_.odom_a_mod_ = odom_a_mod;
}

void Node::publishMapPointCloud(const ros::TimerEvent&)
{
  map_point_cloud_msg_.header.stamp = ros::Time::now();
  map_point_cloud_pub_.publish(map_point_cloud_msg_);
}

void Node::publishGridSlice(const ros::TimerEvent&)
{
  grid_slice_msg_.header.stamp = ros::Time::now();
  grid_slice_pub_.publish(grid_slice_msg_);
}

void Node::publishParticles()
{
  /* If the filter is not initialized then exit */
  if (!mcl_->isInitialized())
    return;

  /* Build the msg based on the particles position and orientation */
  std::vector<Particle> pf_vec = mcl_->getParticle();

  geometry_msgs::PoseArray msg;
  msg.poses.resize(pf_vec.size());
  for (uint32_t i = 0; i < pf_vec.size(); ++i)
  {
    msg.poses[i].position.x = static_cast<double>(pf_vec[i].x);
    msg.poses[i].position.y = static_cast<double>(pf_vec[i].y);
    msg.poses[i].position.z = static_cast<double>(pf_vec[i].z);
    msg.poses[i].orientation.x = 0.;
    msg.poses[i].orientation.y = 0.;
    msg.poses[i].orientation.z = sin(static_cast<double>(pf_vec[i].a * 0.5f));
    msg.poses[i].orientation.w = cos(static_cast<double>(pf_vec[i].a * 0.5f));
  }
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = parameters_.global_frame_id_;

  /* Publish particle cloud */
  particles_pose_pub_.publish(msg);
}

void Node::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!is_odom_arrive_)
  {
    using namespace VSCOMMON;
    LOG_COUT_WARN(g_log,__FUNCTION__,"Odometry transform not received");
    return;
  }

  VSCOMMON::tic("ProcessFrame");
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
  odom_increment_eigen_ = lastupdatebase_2_odom_eigen_.inverse()*base_2_odom_eigen_;

  float _t_x, _t_y, _t_z, _t_roll, _t_pitch, _t_yaw;
  pcl::getTranslationAndEulerAngles(odom_increment_eigen_, _t_x, _t_y, _t_z, _t_roll, _t_pitch, _t_yaw);

  const double delta_x = _t_x;
  const double delta_y = _t_y;
  const double delta_z = _t_z;
  const double delta_a = _t_yaw;

  VSCOMMON::tic("PFMove");
  mcl_->PFMove(delta_x, delta_y, delta_z, delta_a);
  LOG_INFO(g_log,"PFMove time:"<<VSCOMMON::toc("PFMove") * 1000<<" ms");

  /* Perform particle update based on current point-cloud */
  VSCOMMON::tic("Update");
  mcl_->update(cloud_down, roll_, pitch_);
  LOG_INFO(g_log,"Update time:"<<VSCOMMON::toc("Update") * 1000<<" ms");

  //  best particle need compute before resample
  Particle best_p = mcl_->getBestParticle();
  /* Do the resampling if needed */
  VSCOMMON::tic("Resample");
  static int n_updates = 0;
  if (++n_updates > parameters_.resample_interval_)
  {
    n_updates = 0;
    mcl_->PFResample();
  }

  mean_p_ = mcl_->getMean();

  /* Update time and transform information */
  lastupdatebase_2_odom_eigen_ = base_2_odom_eigen_;
  //lastupdatebase_2_odom_tf_ = base_2_odom_tf_;

  LOG_INFO(g_log,"Resample time:"<<VSCOMMON::toc("Resample") * 1000<<" ms");
  LOG_INFO(g_log,"Finish process frame. cost: " << VSCOMMON::toc("ProcessFrame") * 1000 << " ms."
          <<" particle num:"<< mcl_->getParticle().size());
  LOG_INFO(g_log,"Current robot pose:" << mean_p_.x<<" "<< mean_p_.y<<" "<< mean_p_.z <<" "
          << mean_p_.a << " weight:" << mean_p_.w);

  LOG_INFO(g_log,"Current robot bests pose:" << best_p.x<<" "<< best_p.y<<" "<< best_p.z <<" "
          << best_p.a << " weight:" << best_p.w);

  /* Publish particles */
  publishParticles();

}

Eigen::Vector3f Node::quaternion2eulerAngle(float x,float y,float z,float w)
{
  //tf::Matrix3x3().getRPY()函数将四元数转换为rpy(分别为绕xyz)
  Eigen::Quaternionf quaternion(w,x,y,z);
  Eigen::Vector3f eulerAngle=quaternion.matrix().eulerAngles(0,1,2);  //绕x-y-z
  return eulerAngle;
}

Eigen::Quaternionf Node::eulerAngle2quaternion(float roll,float pitch,float yaw)
{
  Eigen::Vector3f eulerAngle(roll,pitch,yaw); //  初始化euler,X-Y-Z
  Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitX()));
  Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
  Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitZ())); 
  Eigen::Quaternionf quaternion;
  quaternion=yawAngle*pitchAngle*rollAngle;
  return quaternion;
}


void Node::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  /*double _o_roll, _o_pitch, _o_yaw;
  tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)).getRPY(_o_roll, _o_pitch, _o_yaw);*/

  Eigen::Vector3f eulerAngle = quaternion2eulerAngle(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  base_2_odom_eigen_ = pcl::getTransformation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z
                      ,eulerAngle[0], eulerAngle[1], eulerAngle[2]);

  /* If the filter is not initialized then exit */
  if (!mcl_->isInitialized())
  {
    using namespace VSCOMMON;
    LOG_COUT_WARN(g_log,__FUNCTION__,"amcl3d not initialized yet, waiting for initial pose.");
    if (parameters_.set_initial_pose_)
    {
      Eigen::Affine3f init_pose = pcl::getTransformation(parameters_.init_x_, parameters_.init_y_, parameters_.init_z_
                                  ,0,0,parameters_.init_a_);
      LOG_COUT_INFO(g_log,__FUNCTION__<<" "<<__LINE__<<" : reloc pose from xml: "
        << parameters_.init_x_<<" "<< parameters_.init_y_<<" "<< parameters_.init_z_ );
      setInitialPose(init_pose);
    }
    return;
  }

  /* Update roll and pitch from odometry */
  double yaw = eulerAngle[2];
  roll_ = eulerAngle[0],pitch_ = eulerAngle[1];

  static tf::TransformBroadcaster tf_br;
  tf_br.sendTransform(
      tf::StampedTransform(getTransformFromAffine3f(base_2_odom_eigen_), ros::Time::now(), parameters_.odom_frame_id_, parameters_.base_frame_id_));

  if (!is_odom_arrive_)
  {
    is_odom_arrive_ = true;
    lastbase_2_world_eigen_ = initodom_2_world_eigen_;
    lastodom_2_world_eigen_ = lastbase_2_world_eigen_*base_2_odom_eigen_.inverse();
  }

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
      Eigen::Affine3f base_2_world_eigen_ = pcl::getTransformation(mean_p_.x, 
            mean_p_.y, mean_p_.z, roll_, pitch_, mean_p_.a);

      //  interpolate pose use odom pose
      base_2_world_eigen_ = base_2_world_eigen_*lastupdatebase_2_odom_eigen_.inverse()*base_2_odom_eigen_;

      lastmean_p_ = mean_p_;

      lastbase_2_world_eigen_ = base_2_world_eigen_;
      lastodom_2_world_eigen_ = base_2_world_eigen_*base_2_odom_eigen_.inverse();

      amcl_out_lastbase_2_odom_eigen_ = lastupdatebase_2_odom_eigen_;
    }
    else
    {
      //  无amcl定位数据，里程计航迹推演
      lastbase_2_world_eigen_ = lastbase_2_world_eigen_*amcl_out_lastbase_2_odom_eigen_.inverse()*base_2_odom_eigen_;
      amcl_out_lastbase_2_odom_eigen_ = base_2_odom_eigen_;
    }
  }


  //  send transform between odom and world frame
  tf_br.sendTransform(tf::StampedTransform(getTransformFromAffine3f(lastodom_2_world_eigen_), ros::Time::now(), parameters_.global_frame_id_,
                                           parameters_.odom_frame_id_));
}

void Node::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
      /*double _i_roll, _i_pitch, _i_yaw;
      tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)).getRPY(_i_roll, _i_pitch, _i_yaw);*/

      Eigen::Vector3f eulerAngle = quaternion2eulerAngle(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                             msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      Eigen::Affine3f init_pose = pcl::getTransformation(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z
                                  ,eulerAngle[0],eulerAngle[1],eulerAngle[2]);
      using namespace VSCOMMON;
      LOG_COUT_INFO(g_log,__FUNCTION__<<" "<<__LINE__<<" : reloc pose from rviz."
        << init_pose(0,3)<<" "<< init_pose(1,3)<<" "<< init_pose(2,3)<<" "
        << eulerAngle[0]<<" "<< eulerAngle[1]<<" "<<eulerAngle[2]);
      setInitialPose(init_pose);
}

bool Node::checkUpdateThresholds()
{
  static bool b_check_first = true;
  if(b_check_first)
    {
      b_check_first = false;
      return true;
    }

  if (ros::Time::now() < nextupdate_time_)
    return false;
  //  odom increment between last amcl pose and current
  odom_increment_eigen_ = lastupdatebase_2_odom_eigen_.inverse()*base_2_odom_eigen_;

  float c_x,c_y,c_z,c_roll,c_pitch,c_yaw;
  pcl::getTranslationAndEulerAngles(odom_increment_eigen_, c_x,c_y,c_z,c_roll,c_pitch,c_yaw);
  /* Check translation threshold */
  if(c_x*c_x+c_y*c_y+c_z*c_z > parameters_.d_th_*parameters_.d_th_)
  {
    return true;
  }
  if(fabs(c_yaw) > parameters_.a_th_)
  {
    return true;
  }

  return false;
}

void Node::setInitialPose(const Eigen::Affine3f& init_pose)
{
  initodom_2_world_eigen_ = init_pose;
  float _s_x, _s_y, _s_z, _s_roll, _s_pitch, _s_yaw;
  pcl::getTranslationAndEulerAngles(init_pose, _s_x, _s_y, _s_z, _s_roll, _s_pitch, _s_yaw);

  const float x_init = _s_x;
  const float y_init = _s_y;
  const float z_init = _s_z;
  const float a_init = _s_yaw;

  mcl_->init(parameters_.num_particles_, x_init, y_init, z_init, a_init);

  mean_p_ = mcl_->getMean();
  lastmean_p_ = mean_p_;

  lastupdatebase_2_odom_eigen_ = base_2_odom_eigen_;

  publishParticles();
}

tf::Transform Node::getTransformFromAffine3f(const Eigen::Affine3f& tf_pose)
{
  float _g_x, _g_y, _g_z, _g_roll, _g_pitch, _g_yaw;
  pcl::getTranslationAndEulerAngles(tf_pose, _g_x, _g_y, _g_z, _g_roll, _g_pitch, _g_yaw);  

  Eigen::Quaternionf eq = eulerAngle2quaternion(_g_roll,_g_pitch,_g_yaw);

  tf::Quaternion q(eq.x(),eq.y(),eq.z(),eq.w());
  //q.setRPY(_g_roll, _g_pitch, _g_yaw);

  //std::cout<<q.x()<<" "<< q.y()<<" "<<q.z()<<" "<< q.w()<<" --- "<< eq.x()<<" "<< eq.y()<<" "<< eq.z()<<" "<< eq.w()<<std::endl;
  tf::Transform out_pose;
  out_pose.setOrigin(tf::Vector3(_g_x, _g_y, _g_z));
  out_pose.setRotation(q);
  return out_pose;
}

bool Node::buildMapPointCloudMsg(sensor_msgs::PointCloud2& msg) const
{
  if (!grid3d_->pc_info_ || !grid3d_->pc_info_->cloud)
    return false;

  pcl::toROSMsg(*(grid3d_->pc_info_->cloud), msg);
  LOG_INFO(g_log, __FUNCTION__<<" "<<__LINE__<<" build pointcloud msg successful. ");
  return true;
}

bool Node::buildGridSliceMsg(const double z, nav_msgs::OccupancyGrid& msg) const
{
  if (!grid3d_->grid_info_ || !grid3d_->pc_info_)
    return false;

  if (z < grid3d_->pc_info_->octo_min_z || z > grid3d_->pc_info_->octo_max_z)
    return false;

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = grid3d_->pc_info_->octo_resol;
  msg.info.width = grid3d_->grid_info_->size_x;
  msg.info.height = grid3d_->grid_info_->size_y;
  msg.info.origin.position.x = 0.;
  msg.info.origin.position.y = 0.;
  msg.info.origin.position.z = z;
  msg.info.origin.orientation.x = 0.;
  msg.info.origin.orientation.y = 0.;
  msg.info.origin.orientation.z = 0.;
  msg.info.origin.orientation.w = 1.;

  /* Extract max probability */
  const uint32_t init = grid3d_->point2grid(grid3d_->pc_info_->octo_min_x, grid3d_->pc_info_->octo_min_y, z);
  const uint32_t end = grid3d_->point2grid(grid3d_->pc_info_->octo_max_x, grid3d_->pc_info_->octo_max_y, z);
  float temp_prob, max_prob = -1.0;
  auto grid_ptr = grid3d_->grid_info_->grid.data();
  for (uint32_t i = init; i < end; ++i)
  {
    temp_prob = grid_ptr[i].prob;
    if (temp_prob > max_prob)
      max_prob = temp_prob;
  }

  /* Copy data into grid msg and scale the probability to [0, 100] */
  if (max_prob < 0.000001f)
    max_prob = 0.000001f;
  max_prob = 100.f / max_prob;
  msg.data.resize(end - init);
  for (uint32_t i = 0; i < msg.data.size(); ++i)
    msg.data[i] = static_cast<int8_t>(grid_ptr[init + i].prob * max_prob);
    LOG_INFO(g_log,__FUNCTION__<<" "<<__LINE__<<" build grid slice msg successful. ");
  return true;
}


}  // namespace amcl3d
