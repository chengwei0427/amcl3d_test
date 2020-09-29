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

#include "ParticleFilter.h"

namespace amcl3d
{
ParticleFilter::ParticleFilter(VSCOMMON::LoggerPtr& t_log) 
  : generator_(rd_()),g_log(t_log),grid3d_(new Grid3d(t_log))
{
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::relocPose(const int num_particles, const float x_init, const float y_init, const float z_init,
                          const float roll_init, const float pitch_init, const float yaw_init,
                          const float x_dev, const float y_dev, const float z_dev,
                          const float roll_dev, const float pitch_dev, const float yaw_dev)
{
  /*  Resize particle set */
  p_.resize(abs(num_particles));

  /*  Sample the given pose */
  const float dev = std::max(std::max(x_dev, y_dev), z_dev);
  const float gauss_const_1 = 1. / (dev * sqrt(2 * M_PI));
  const float gauss_const_2 = 1. / (2 * dev * dev);

  p_[0].x = x_init;
  p_[0].y = y_init;
  p_[0].z = z_init;
  p_[0].roll = roll_init;
  p_[0].pitch = pitch_init;
  p_[0].yaw = yaw_init;
  p_[0].w = gauss_const_1;

  float wt = p_[0].w;
  float dist;

  for (uint32_t i = 1; i < p_.size(); ++i)
  {
    p_[i].x = p_[0].x + ranGaussian(0, x_dev);
    p_[i].y = p_[0].y + ranGaussian(0, y_dev);
    p_[i].z = p_[0].z + ranGaussian(0, z_dev);
    p_[i].roll = p_[0].roll + ranGaussian(0, roll_dev);
    p_[i].pitch = p_[0].pitch + ranGaussian(0, pitch_dev);
    p_[i].yaw = p_[0].yaw + ranGaussian(0, yaw_dev);

    dist = sqrt((p_[i].x - p_[0].x) * (p_[i].x - p_[0].x) + (p_[i].y - p_[0].y) * (p_[i].y - p_[0].y) +
                (p_[i].z - p_[0].z) * (p_[i].z - p_[0].z));

    p_[i].w = gauss_const_1 * exp(-dist * dist * gauss_const_2);

    wt += p_[i].w;
  }

  Particle mean_p;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    p_[i].w /= wt;

    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.roll += p_[i].w * p_[i].roll;
    mean_p.pitch += p_[i].w * p_[i].pitch;
    mean_p.yaw += p_[i].w * p_[i].yaw;
  }
  mean_ = mean_p;

  initialized_ = true;
  using namespace VSCOMMON;
  LOG_COUT_INFO(g_log,"mcl initialized.");
}

void ParticleFilter::predict(const double delta_x, const double delta_y, const double delta_z,
                            const double delta_roll,const double delta_pitch,const double delta_yaw,
                            const double odom_x_noise, const double odom_y_noise, const double odom_z_noise,
                            const double odom_roll_noise, const double odom_pitch_noise,const double odom_yaw_noise)
{
  /*  Make a prediction for all particles according to the odometry */
  float sa, ca, rand_x, rand_y;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    // TODO: 角度变换，roll pitch对坐标的影响
    sa = sin(p_[i].yaw);
    ca = cos(p_[i].yaw);
    rand_x = delta_x + ranGaussian(0, odom_x_noise);
    rand_y = delta_y + ranGaussian(0, odom_y_noise);
    p_[i].x += ca * rand_x - sa * rand_y;
    p_[i].y += sa * rand_x + ca * rand_y;
    p_[i].z += delta_z + ranGaussian(0, odom_z_noise);
    p_[i].roll += delta_roll + ranGaussian(0,odom_roll_noise);
    p_[i].pitch += delta_pitch + ranGaussian(0,odom_pitch_noise);
    p_[i].yaw += delta_yaw + ranGaussian(0, odom_yaw_noise);
  }
}

void ParticleFilter::update(const pcl::PointCloud<PointType>::Ptr& cloud)
{
  /*  Incorporate measurements */
  VSCOMMON::tic("Update");
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    /*  Get particle information */
    float tx = p_[i].x;
    float ty = p_[i].y;
    float tz = p_[i].z;

    /*  Check the particle is into the map */
    if (!grid3d_->isIntoMap(tx, ty, tz))
    {
      // std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl;
      p_[i].w = 0;
      continue;
    }

    //  Evaluate the weight of the point cloud 
    //  TODO: ground and non ground points compute weight respectively

    p_[i].w = grid3d_->computeCloudWeight(cloud, tx, ty, tz, p_[i].roll, p_[i].pitch, p_[i].yaw);  
  }
  LOG_INFO(g_log,"Update time:"<<VSCOMMON::toc("Update") * 1000<<" ms");
}

void ParticleFilter::particleNormalize()
{
  /*  Normalize all weights */
  float wtp = 0,w_max = 0;
  for(uint32_t i = 0;i < p_.size();i++)
  {
    if(p_[i].w > 0)
      wtp += p_[i].w;
    if(p_[i].w > w_max)
      w_max = p_[i].w;
  }
  float wt = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if (wtp > 0)
      p_[i].w /= wtp;
    else
      p_[i].w = 0;

    if (!grid3d_->isIntoMap(p_[i].x, p_[i].y, p_[i].z))
    {
      /* std::cout << "Not into map: " << grid3d_.isIntoMap(tx, ty, tz-1.0) << std::endl; */
      p_[i].w = 0;
    }
    else
      p_[i].w = p_[i].w;
    wt += p_[i].w;
  }
}

Particle ParticleFilter::getMean()
{
  Particle mean_p;
  float w_max = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    mean_p.x += p_[i].w * p_[i].x;
    mean_p.y += p_[i].w * p_[i].y;
    mean_p.z += p_[i].w * p_[i].z;
    mean_p.roll += p_[i].w * p_[i].roll;
    mean_p.pitch += p_[i].w * p_[i].pitch;
    mean_p.yaw += p_[i].w * p_[i].yaw;
    if(p_[i].w > w_max)
      w_max = p_[i].w;
  }
  mean_p.w = w_max; //  out put max pf weight
  mean_ = mean_p;
  return mean_p;
}

Particle ParticleFilter::getBestParticle()
{
  Particle best_p;
  best_p.w = 0;
  for (uint32_t i = 0; i < p_.size(); ++i)
  {
    if(p_[i].w > best_p.w)
      best_p = p_[i];
  }
  return best_p;
}

void ParticleFilter::resample()
{
  std::vector<Particle> new_p(p_.size());
  const float factor = 1.f / p_.size();
  const float r = factor * rngUniform(0, 1);
  float c = p_[0].w;
  float u;

  //! Do resamplig
  for (uint32_t m = 0, i = 0; m < p_.size(); ++m)
  {
    u = r + factor * m;
    while (u > c)
    {
      if (++i >= p_.size())
        break;
      c += p_[i].w;
    }
    new_p[m] = p_[i];
    new_p[m].w = factor;
  }

  //! Asign the new particles set
  p_ = new_p;
}

void ParticleFilter::uniformSample(int& sample_num)
{
  particleNormalize();
  std::vector<float> left_sum(p_.size());
  left_sum[0] = p_[0].w;
  for(int i=1;i<p_.size();i++)
  {
    left_sum[i] = p_[i].w + left_sum[i - 1];
  }

  std::vector<Particle> new_p(sample_num);
  const float inteval = 1.0f / sample_num;
  float samp_value = 0.5f / sample_num;
  int idx=0;
  for (int i = 0; i<left_sum.size(); i++) {
    while(idx < sample_num && left_sum[i] > samp_value) {
      new_p[idx] = p_[i];
      new_p[idx].w = inteval;
      idx++;
      samp_value += inteval;
    }
  }
  p_ = new_p;
  // std::cout<<"sample_num: "<< sample_num<<" --> "<< p_.size()<<std::endl;
}

float ParticleFilter::ranGaussian(const double mean, const double sigma)
{
  std::normal_distribution<float> distribution(mean, sigma);
  return distribution(generator_);
}

float ParticleFilter::rngUniform(const float range_from, const float range_to)
{
  std::uniform_real_distribution<float> distribution(range_from, range_to);
  return distribution(generator_);
}

void ParticleFilter::setParticleNum(int max_sample ,int min_sample)
{
  max_resamp_num_ = max_sample;
  min_resamp_num_ = min_sample;
}

}  // namespace amcl3d
