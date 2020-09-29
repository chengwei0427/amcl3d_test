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

struct LocalizationParam{
  LocalizationParam()
    :min_particle_num(50)
    ,max_particle_num_local(200)
    ,max_particle_num_global(600)
    ,init_x_dev_(0.5)
    ,init_y_dev_(0.5)
    ,init_z_dev_(0.4)
    ,init_a_dev_(0.4)
    ,odom_x_mod_(0.4)
    ,odom_y_mod_(0.4)
    ,odom_z_mod_(0.05)
    ,odom_a_mod_(0.2)
    {}

  int min_particle_num;
  int max_particle_num_global;
  int max_particle_num_local;
  double init_x_dev_; /*!< Thresholds x-axis position in initialization*/
  double init_y_dev_; /*!< Thresholds y-axis position in initialization*/
  double init_z_dev_; /*!< Thresholds z-axis position in initialization*/
  double init_a_dev_; /*!< Thresholds yaw angle in initialization*/

  double odom_x_mod_; /*!< Thresholds x-axis position in the prediction */
  double odom_y_mod_; /*!< Thresholds y-axis position in the prediction */
  double odom_z_mod_; /*!< Thresholds z-axis position in the prediction */
  double odom_a_mod_; /*!< Thresholds yaw angle in the prediction */

};

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
  void init(const int num_particles, const float x_init, const float y_init, const float z_init, const float a_init);
  
  void PFMove(const double delta_x, const double delta_y, const double delta_z, const double delta_a);

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

  int computeSampleNum(const int cnt_per_grid = 9);


  enum{GLOBAL_LOC, LASER_ODOM, POSE_TRACKING, BLIND_TRACKING};
  int cur_loc_status_;

  LocalizationParam localization_params_;

  int max_sample_num;
  int loc_loop_; /*process pointcloud loop num*/
  VSCOMMON::LoggerPtr g_log;
};

}  // namespace amcl3d
