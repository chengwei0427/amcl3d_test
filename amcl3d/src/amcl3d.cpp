/*!
 * @file ParticleFilter.cpp
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

#include "amcl3d.h"

namespace amcl3d
{
  MonteCarloLocalization::MonteCarloLocalization(VSCOMMON::LoggerPtr& t_log) 
      : g_log(t_log)
      ,ParticleFilter(t_log)
      ,loc_loop_(0)
      ,cur_loc_status_(GLOBAL_LOC)
      ,global_loc_done_(false)
      ,cur_loc_status_cnt_(0)
  {  }

  MonteCarloLocalization::~MonteCarloLocalization()
  { }

  bool MonteCarloLocalization::init()
  {
    cloud_ds.reset(new pcl::PointCloud<PointType>());
    ds_.setLeafSize(params_.voxel_size_, params_.voxel_size_, params_.voxel_size_);
    if (!grid3d_->open(params_.map_path_, params_.sensor_dev_))
      return false;
    kdtree_keypose_3d_.reset(new pcl::KdTreeFLANN<PointType>());
    kdtree_keypose_3d_->setInputCloud(grid3d_->keyposes_3d);
    return true;
  }

  void MonteCarloLocalization::processFrame(pcl::PointCloud<PointType>::Ptr& input_cloud,Eigen::Affine3f& odom_increment)
  {
    VSCOMMON::tic("ProcessFrame");
    LOG_INFO(g_log,"Begin process frame. loop:" << loc_loop_++);

    //pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>());
    ds_.setInputCloud(input_cloud);
    ds_.filter(*cloud_ds);
    
    static bool first_process = true;
    if(!first_process)
    {
      current_pose = mean_;
      if((current_pose.x-Last_pose.x)*(current_pose.x-Last_pose.x) + (current_pose.y-Last_pose.y)*(current_pose.y-Last_pose.y) > 25)
      {
        if(grid3d_->updateMap(current_pose.x,current_pose.y,current_pose.z))
          Last_pose = current_pose;
      }
    }

    float _t_x, _t_y, _t_z, _t_roll, _t_pitch, _t_yaw;
    pcl::getTranslationAndEulerAngles(odom_increment, _t_x, _t_y, _t_z, _t_roll, _t_pitch, _t_yaw);
    odomIncre_ = Particle(_t_x, _t_y, _t_z, _t_roll, _t_pitch, _t_yaw);

    if(!global_loc_done_) //  do global localization here
    {
      caculateGlobalNoise();
      globalLocalization();
    }
    else                  //  do local localization for times
    {
      caculateLocalNoise();
      localLocalization();
    }

    getMean();
    if(first_process)
    {
      first_process = false;
      Last_pose = mean_;
    }
    LOG_INFO(g_log,"Finish process frame. cost: " << VSCOMMON::toc("ProcessFrame") * 1000 << " ms."
          <<" robot pose:"<<mean_.x<<" "<<mean_.y<<" "<<mean_.z<<" "<<mean_.roll<<" "<<mean_.pitch<<" "<<mean_.yaw
          <<", particle num:"<< getParticle().size()<<" state count: "<<cur_loc_status_cnt_++);
  }

  void MonteCarloLocalization::caculateGlobalNoise(const float f)  //  global noise twice local nose
  {
    double x_dev = VSCOMMON::clip(fabs(odomIncre_.x * localization_params_.odom_x_mod_),localization_params_.min_xy_noise_,0.2);
    double y_dev = VSCOMMON::clip(fabs(odomIncre_.y * localization_params_.odom_y_mod_),localization_params_.min_xy_noise_,0.2);
    double z_dev = VSCOMMON::clip(fabs(odomIncre_.z * localization_params_.odom_z_mod_),localization_params_.min_z_noise_,0.15);
    double roll_dev = VSCOMMON::clip(fabs(odomIncre_.roll * localization_params_.odom_roll_mod_),localization_params_.min_rp_noise_,0.1);
    double pitch_dev = VSCOMMON::clip(fabs(odomIncre_.pitch * localization_params_.odom_pitch_mod_),localization_params_.min_rp_noise_,0.1);
    double yaw_dev = VSCOMMON::clip(fabs(odomIncre_.yaw * localization_params_.odom_yaw_mod_),localization_params_.min_yaw_noise_,0.1);
    cur_odom_noise_ = Particle(x_dev*f,y_dev*f,z_dev*f,roll_dev*f,pitch_dev*f,yaw_dev*f);
  }

  void MonteCarloLocalization::caculateLocalNoise()
  {
    double x_dev = VSCOMMON::clip(fabs(odomIncre_.x * localization_params_.odom_x_mod_),localization_params_.min_xy_noise_,0.2);
    double y_dev = VSCOMMON::clip(fabs(odomIncre_.y * localization_params_.odom_y_mod_),localization_params_.min_xy_noise_,0.2);
    double z_dev = VSCOMMON::clip(fabs(odomIncre_.z * localization_params_.odom_z_mod_),localization_params_.min_z_noise_,0.15);
    double roll_dev = VSCOMMON::clip(fabs(odomIncre_.roll * localization_params_.odom_roll_mod_),localization_params_.min_rp_noise_,0.1);
    double pitch_dev = VSCOMMON::clip(fabs(odomIncre_.pitch * localization_params_.odom_pitch_mod_),localization_params_.min_rp_noise_,0.1);
    double yaw_dev = VSCOMMON::clip(fabs(odomIncre_.yaw * localization_params_.odom_yaw_mod_),localization_params_.min_yaw_noise_,0.1);
    cur_odom_noise_ = Particle(x_dev,y_dev,z_dev,roll_dev,pitch_dev,yaw_dev);
  }

  void MonteCarloLocalization::globalLocalization()
  {
    using namespace VSCOMMON;
    if(cur_loc_status_cnt_ == 1)
    {
      LOG_COUT_INFO(g_log,"Reloc at " << set_pose_.x << " " << set_pose_.y << " "<< set_pose_.z
      <<" "<< VSCOMMON::rad2deg(set_pose_.roll) <<" "<<VSCOMMON::rad2deg(set_pose_.pitch)  
      <<" " << VSCOMMON::rad2deg(set_pose_.yaw));
    }
    PFMove(odomIncre_,cur_odom_noise_);

    update(cloud_ds);

    PFResample(true);

    static int consist_cnt = 0;
    //TODO: check for location successful or not
    Particle best_pf = getBestParticle();
    Particle mean_pf = getMean();
    int ptc_num = getParticle().size();
    float weight_pf = grid3d_->computeCloudWeight(cloud_ds, mean_pf.x, mean_pf.y, mean_pf.z, mean_pf.roll, mean_pf.pitch, mean_pf.yaw);  

    LOG_COUT_INFO(g_log,"Global localization." << " cnt: " << cur_loc_status_cnt_
        <<",particle num: "<< ptc_num<<",w_pf: "<< weight_pf<<",cloud_ds: "<< cloud_ds->points.size());

    if(ptc_num < 550 && weight_pf>localization_params_.weight_global_)
    {
      consist_cnt++;
      if(consist_cnt > 10/*consist_cnt_thres_*/)
      {
        consist_cnt = 0;
        LOG_COUT_INFO(g_log,"Global localization done. Current pose is " <<mean_pf.x<<" "<<mean_pf.y<<" "<<mean_pf.z
          <<" "<<mean_pf.roll<<" "<<mean_pf.pitch<<" "<<mean_pf.yaw<< " weight: " << weight_pf);
        global_loc_done_ = true;
        setParticleNum(localization_params_.max_particle_num_local,localization_params_.min_particle_num_local);
        cur_loc_status_cnt_ = 0;
      }
    }
    else
      consist_cnt = 0;
  }

  void MonteCarloLocalization::localLocalization()
  {
    PFMove(odomIncre_,cur_odom_noise_);

    update(cloud_ds);
    //TODO: goto different process,like blind tracking,pose tracking or laserodom tracking.
    PFResample(); 
  }
  //  TODO: reloc from rviz, Z is 0 always,may check for trajectory pose hight here
  void MonteCarloLocalization::RelocPose(const int num_particles, 
    const float x_init,const float y_init, const float z_init, 
    const float roll_init, const float pitch_init,const float yaw_init)
  {
    PointType kd_nearest_pt = getMapHeight(x_init,y_init);
    grid3d_->updateMap(kd_nearest_pt.x,kd_nearest_pt.y,kd_nearest_pt.z);

    LOG_INFO(g_log,"find nearest pt: "<< kd_nearest_pt.x<<" "<< kd_nearest_pt.y<<" "<< kd_nearest_pt.z <<" from: "<< x_init<<" "<< y_init<<" "<< z_init);
    set_pose_ = Particle(x_init, y_init, kd_nearest_pt.z, roll_init,pitch_init,yaw_init);
    ParticleFilter::relocPose(num_particles, x_init, y_init, kd_nearest_pt.z, roll_init,pitch_init,yaw_init,
        localization_params_.init_x_dev_,localization_params_.init_y_dev_,localization_params_.init_z_dev_,
        localization_params_.init_roll_dev_,localization_params_.init_pitch_dev_,localization_params_.init_yaw_dev_);

    setParticleNum(localization_params_.max_particle_num_global,localization_params_.min_particle_num_global);
    cur_loc_status_cnt_ = 1;
    global_loc_done_ = false;
  }

  void MonteCarloLocalization::PFMove(const Particle& odom_increment, const Particle& odom_increment_noise)
  {
    VSCOMMON::tic("PFMove");
    ParticleFilter::predict(odom_increment.x,odom_increment.y,odom_increment.z,odom_increment.roll,odom_increment.pitch,
      odom_increment.yaw,odom_increment_noise.x,odom_increment_noise.y,odom_increment_noise.z,
      odom_increment_noise.roll,odom_increment_noise.pitch,odom_increment_noise.yaw);
    LOG_INFO(g_log,"PFMove time:"<<VSCOMMON::toc("PFMove") * 1000<<" ms");
  }

  void MonteCarloLocalization::PFResample(const bool weight_multiply)
  {
    VSCOMMON::tic("Resample");
    if(weight_multiply)
      addParticleWeight();  //  for global location,weight multiply, speed converge
    else
      setParticleWeight();
    int sample_num = computeSampleNum();
    uniformSample(sample_num);
    updateSampleHeight();
    LOG_INFO(g_log,"Resample time:"<<VSCOMMON::toc("Resample") * 1000<<" ms");
  }

  void MonteCarloLocalization::addParticleWeight()
  {
    //  compute max weight
    float max_w = 0;
    for(uint32_t i = 0;i < p_.size();++i)
      if(p_[i].w > max_w)
        max_w = p_[i].w;

    float k = 1;
    if(max_w>0) k = 1.0/max_w;
    for(uint32_t i = 0;i < p_.size();++i)
      p_[i].w = k*p_[i].w;
  }

  void MonteCarloLocalization::setParticleWeight()
  {   }

  void computeBoundary(const std::vector<Particle>& p_set,float& xmin
    ,float& ymin,float& zmin,float& xmax,float& ymax,float& zmax)
  {
    xmin = xmax = ymin = ymax = zmin = zmax = 0.0;
    if(p_set.empty())
      return;
    xmin = xmax = p_set[0].x;
    ymin = ymax = p_set[0].y;
    zmin = zmax = p_set[0].z;
    for(uint32_t i = 0;i < p_set.size();++i)
    {
      Particle _p = p_set[i];
      xmax = xmax>_p.x ? xmax:_p.x;
      xmin = xmin>_p.x ? _p.x:xmin;
      ymax = ymax>_p.y ? ymax:_p.y;
      ymin = ymin>_p.y ? _p.y:ymin;
      zmax = zmax>_p.z ? zmax:_p.z;
      zmin = zmin>_p.z ? _p.z:zmin;
    }
  }
  int MonteCarloLocalization::computeSampleNum(const int cnt_per_grid /*= 9*/)
  {
    float xyz_step = 0.2;
    float theta_step = 8.0;
    float xmin, xmax, ymin, ymax ,zmin, zmax;
    computeBoundary(p_,xmin,ymin,zmin,xmax,ymax,zmax);
    
    int xwidth = std::ceil((xmax - xmin)/xyz_step)+1;
    int ywidth = std::ceil((ymax - ymin)/xyz_step)+1;
    int zwidth = std::ceil((zmax - zmin)/xyz_step)+1;
    int theta = std::ceil((M_PI*2/theta_step*180))+1;
    LOG_INFO(g_log,"bdx: "<< xmin<<","<< xmax<<", bdy: "<< ymin<<","<< ymax<<", bdz: "
        << zmin<<","<< zmax <<", xwidth: "<< xwidth<<", "<< ywidth<<", "<< zwidth<<", "<< theta);
    if(xwidth*ywidth*zwidth == 0)
      return min_resamp_num_;
    else if(xwidth*ywidth*zwidth > 1e3)
      return max_resamp_num_;

    std::vector<std::vector<std::vector<std::vector<int>>>> grid(xwidth,std::vector<std::vector<std::vector<int>>>(ywidth,std::vector<std::vector<int>>(zwidth,std::vector<int>(theta,0))));
    for(uint32_t i = 0;i < p_.size();++i)
    {
      Particle _p = p_[i];
      int x = (_p.x - xmin)/xyz_step;
      int y = (_p.y - ymin)/xyz_step;
      int z = (_p.z - zmin)/xyz_step;
      int t = (_p.yaw - (-M_PI))/theta_step;
      if(x>=0 && x < xwidth && y>=0 && y < ywidth && z >= 0 && z < zwidth && t >=0 && y < theta)
        grid[x][y][z][t]++;
      else
        std::cout<<"error: pf out of boundary"<<std::endl;
    }

    int cnt = 0;
    for(uint32_t i = 0;i < xwidth;i++)
      for(uint32_t j = 0;j < ywidth; j++)
        for(uint32_t k = 0;k < zwidth;k++)
          for(uint32_t t = 0;t < theta;t++)
            cnt +=(grid[i][j][k][t]>0?1:0);

    int sample_num = cnt * cnt_per_grid;
    LOG_INFO(g_log,"cnt: "<< cnt<<", sample_num: "<< sample_num);
    if(sample_num < min_resamp_num_)
      return min_resamp_num_;
    if(sample_num > max_resamp_num_)
      return max_resamp_num_;
    return sample_num;
  }

  pcl::PointCloud<PointType> MonteCarloLocalization::getMapCloud()
  {
    return (*(grid3d_->pc_info_->cloud));
  }

  PointType MonteCarloLocalization::getMapHeight(const float& x,const float& y)
  {
    PointType kd_pt;
    kd_pt.x = x,kd_pt.y = y,kd_pt.z = 0.;
    std::vector<int> point_idx_(1);
    std::vector<float> point_nkn_squared_(1);
    kdtree_keypose_3d_->nearestKSearch(kd_pt,1,point_idx_,point_nkn_squared_);

    PointType kd_nearest_pt; 
    kd_nearest_pt.x = grid3d_->keyposes_3d->points[point_idx_[0]].x,
    kd_nearest_pt.y = grid3d_->keyposes_3d->points[point_idx_[0]].y,
    kd_nearest_pt.z = grid3d_->keyposes_3d->points[point_idx_[0]].z;
    return kd_nearest_pt;
  }

  void MonteCarloLocalization::updateSampleHeight()
  {
    VSCOMMON::tic("updateSampleHeight");
    Particle pt = getMean();
    PointType pt_pcl = getMapHeight(pt.x,pt.y);
    float delta_h = pt_pcl.z - pt.z;
    for (uint32_t i = 0; i < p_.size(); ++i)
    {
      p_[i].z += delta_h;
    }
    LOG_INFO(g_log,"update Sample Height takes:"<<VSCOMMON::toc("updateSampleHeight") * 1000<<" ms");
  }
}  // namespace amcl3d
