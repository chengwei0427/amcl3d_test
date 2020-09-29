/*!
 * @file ParticleFilter.h
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

#pragma once

#include "ParticleFilter.h"
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

namespace amcl3d
{

/*! \brief Class that contains the stages of the Particle Filter.
 */
class MonteCarloLocalization : public ParticleFilter
{
public:
  /*! \brief MonteCarloLocalization class constructor.
   */
  explicit MonteCarloLocalization(VSCOMMON::LoggerPtr&,Grid3d* );
  /*! \brief MonteCarloLocalization class destructor.
   */
  virtual ~MonteCarloLocalization();

  void processFrame(pcl::PointCloud<PointType>::Ptr& input,Eigen::Affine3f& odom_increment);

  void RelocPose(const int num_particles, const float x_init, const float y_init, const float z_init, 
    const float roll_init, const float pitch_init,const float yaw_init);
  
  void PFMove(const Particle& odom_increment, const Particle& odom_increment_noise);

  void PFResample(const bool weight_multiply = false);

  std::vector<Particle> getParticle()
  {
    return p_;
  }

  void setParams(const LocalizationParam& option,const Parameters& param) 
  {
    localization_params_ = option;
    params_ = param;
  }

  void init();

private:
  void caculateGlobalNoise(const float f = 1);

  void caculateLocalNoise();

  void globalLocalization();

  void localLocalization();

  void addParticleWeight();

  void setParticleWeight();

  int computeSampleNum(const int cnt_per_grid = 3);

  Parameters params_;

  enum{GLOBAL_LOC, LASER_ODOM, POSE_TRACKING, BLIND_TRACKING};
  int cur_loc_status_;


  VSCOMMON::LoggerPtr g_log;

  pcl::VoxelGrid<PointType> ds_;
  pcl::PointCloud<PointType>::Ptr cloud_ds;
  int loc_loop_; /*process pointcloud loop num*/
  int cur_loc_status_cnt_;


  bool global_loc_done_;
  Particle set_pose_;
  Particle odomIncre_;
  Particle cur_odom_noise_;
};

}  // namespace amcl3d
