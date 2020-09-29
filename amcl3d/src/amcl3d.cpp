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
  MonteCarloLocalization::MonteCarloLocalization(VSCOMMON::LoggerPtr& t_log,Grid3d* grid3d) 
      : g_log(t_log)
      ,ParticleFilter(t_log,grid3d)
      ,loc_loop_(0)
      ,cur_loc_status_(GLOBAL_LOC)
      ,max_sample_num(600)
  { 
  }

  MonteCarloLocalization::~MonteCarloLocalization()
  {
  }

  void MonteCarloLocalization::init(const int num_particles, 
    const float x_init,const float y_init, const float z_init, 
    const float roll_init, const float pitch_init,const float yaw_init)
  {
      ParticleFilter::init(num_particles, x_init, y_init, z_init, roll_init,pitch_init,yaw_init,
        localization_params_.init_x_dev_,localization_params_.init_y_dev_,localization_params_.init_z_dev_,
        localization_params_.init_roll_dev_,localization_params_.init_pitch_dev_,localization_params_.init_yaw_dev_);
  }

  void MonteCarloLocalization::PFMove(const double delta_x, const double delta_y, const double delta_z, 
    const double delta_roll, const double delta_pitch, const double delta_yaw)
  {
    ParticleFilter::predict(localization_params_.odom_x_mod_,localization_params_.odom_y_mod_
      ,localization_params_.odom_z_mod_,localization_params_.odom_roll_mod_,
      localization_params_.odom_pitch_mod_,localization_params_.odom_yaw_mod_,
      delta_x,delta_y,delta_z,delta_roll,delta_pitch,delta_yaw);
  }

  void MonteCarloLocalization::PFResample()
  {
    if(global_init_num > 0)
      {
        addParticleWeight();
        global_init_num--;
      }
    else
      setParticleWeight();
    int sample_num = computeSampleNum();
    if(global_init_num > 0)
      sample_num = localization_params_.max_particle_num_global;
    uniformSample(sample_num);
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
      return localization_params_.min_particle_num;
    else if(xwidth*ywidth*zwidth > 1e3)
      return max_sample_num;

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
    if(sample_num < localization_params_.min_particle_num)
      return localization_params_.min_particle_num;
    if(sample_num > max_sample_num)
      return max_sample_num;
    return sample_num;
  }
}  // namespace amcl3d
