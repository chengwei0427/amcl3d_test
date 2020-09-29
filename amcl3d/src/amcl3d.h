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

  /*! \brief To inicialite the grid map.
   *
   * \return <b>bool=False</b> - If it has not been initialized.
   * \return <b>bool=True</b> - If it has been initialized.
   *
   * It only return the variable initialized_, and this is modified in the code when the filter does the
   * MonteCarloLocalization::init method.
   */
  /*bool isInitialized() const
  {
    return initialized_;
  }*/
  void init(const int num_particles, const float x_init, const float y_init, const float z_init, 
    const float roll_init, const float pitch_init,const float yaw_init);
  
  void PFMove(const double delta_x, const double delta_y, const double delta_z, 
    const double delta_roll, const double delta_pitch, const double delta_yaw);

  void PFResample();

  std::vector<Particle> getParticle()
  {
    return p_;
  }

  void setParams(const LocalizationParam& option) 
  {
    localization_params_ = option;
  }


private:
  void addParticleWeight();

  void setParticleWeight();

  int computeSampleNum(const int cnt_per_grid = 4);


  enum{GLOBAL_LOC, LASER_ODOM, POSE_TRACKING, BLIND_TRACKING};
  int cur_loc_status_;

  int max_sample_num;
  int loc_loop_; /*process pointcloud loop num*/
  VSCOMMON::LoggerPtr g_log;
};

}  // namespace amcl3d
