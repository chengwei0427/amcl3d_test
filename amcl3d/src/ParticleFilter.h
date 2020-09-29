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

#include "Grid3d.h"
#include "Parameters.h"

#include <random>

#include <common/vs_common.h>
#include <common/vs_timer.h>

namespace amcl3d
{
/*! \brief Struct that contains the data concerning one particle.
 */
struct Particle
{
  float x; /*!< Position x */
  float y; /*!< Position y */
  float z; /*!< Position z */
  float roll;
  float pitch;
  float yaw; /*!< Yaw angle */

  float w;  /*!< Total weight */

  Particle() : x(0), y(0), z(0),roll(0),pitch(0), yaw(0), w(0)
  {
  }
  Particle(float _x,float _y,float _z,float _roll,float _pitch,float _yaw)
    :x(_x),y(_y),z(_z),roll(_roll),pitch(_pitch),yaw(_yaw),w(-1){}
};

struct LocalizationParam{
  LocalizationParam()
    :min_particle_num_local(50)
    ,min_particle_num_global(150)
    ,max_particle_num_local(200)
    ,max_particle_num_global(600)
    ,init_x_dev_(0.5)
    ,init_y_dev_(0.5)
    ,init_z_dev_(0.4)
    ,init_roll_dev_(0.2)
    ,init_pitch_dev_(0,2)
    ,init_yaw_dev_(0.4)
    ,odom_x_mod_(0.4)
    ,odom_y_mod_(0.4)
    ,odom_z_mod_(0.05)
    ,odom_roll_mod_(0.05)
    ,odom_pitch_mod_(0.05)
    ,odom_yaw_mod_(0.2)
    ,min_xy_noise_(0.1)
    ,min_z_noise_(0.05)
    ,min_rp_noise_(0.05)
    ,min_yaw_noise_(0.05)
    ,weight_global_(4.0)
    {}

  int min_particle_num_global;
  int min_particle_num_local;
  int max_particle_num_global;
  int max_particle_num_local;
  double init_x_dev_; /*!< Thresholds x-axis position in initialization*/
  double init_y_dev_; /*!< Thresholds y-axis position in initialization*/
  double init_z_dev_; /*!< Thresholds z-axis position in initialization*/
  double init_roll_dev_;
  double init_pitch_dev_;
  double init_yaw_dev_; /*!< Thresholds yaw angle in initialization*/

  double odom_x_mod_; /*!< Thresholds x-axis position in the prediction */
  double odom_y_mod_; /*!< Thresholds y-axis position in the prediction */
  double odom_z_mod_; /*!< Thresholds z-axis position in the prediction */
  double odom_roll_mod_;
  double odom_pitch_mod_;
  double odom_yaw_mod_; /*!< Thresholds yaw angle in the prediction */

  double min_xy_noise_;
  double min_z_noise_;
  double min_rp_noise_;
  double min_yaw_noise_;

  double weight_global_;
};


/*! \brief Class that contains the stages of the Particle Filter.
 */
class ParticleFilter
{
public:
  /*! \brief ParticleFilter class constructor.
   */
  explicit ParticleFilter(VSCOMMON::LoggerPtr&);
  /*! \brief ParticleFilter class destructor.
   */
  virtual ~ParticleFilter();

  /*! \brief To inicialite the grid map.
   *
   * \return <b>bool=False</b> - If it has not been initialized.
   * \return <b>bool=True</b> - If it has been initialized.
   *
   * It only return the variable initialized_, and this is modified in the code when the filter does the
   * ParticleFilter::init method.
   */
  bool isInitialized() const
  {
    return initialized_;
  }

  /*! \brief To get the information from the Particle struct.
   *
   * \return Particle - Particle struct.
   */
  Particle getMean();

  Particle getBestParticle();

  /*! \brief To build the particles pose message.
   *
   * \param msg Type of message that it is wanted to build.
   */
  //void buildParticlesPoseMsg(geometry_msgs::PoseArray& msg) const;

  /*! \brief This function implements the PF init stage.
   *
   * \param num_particles Particle number in the filter.
   * \param x_init Init x-axis position.
   * \param y_init Init x-axis position.
   * \param z_init Init x-axis position.
   * \param a_init Init yaw angle orientation.
   * \param x_dev Init thresholds of x-axis position.
   * \param y_dev Init thresholds of y-axis position.
   * \param z_dev Init thresholds of z-axis position.
   * \param a_dev Init thresholds of yaw angle orientation.
   *
   * It restructures the particle vector to adapt it to the number of selected particles. Subsequently, it initializes
   * it using a Gaussian distribution and the deviation introduced. Subsequently, it calculates what would be the
   * average particle that would simulate the estimated position of the UAV.
   */
  void relocPose(const int num_particles, const float x_init, const float y_init, const float z_init, 
            const float roll_init, const float pitch_init, const float yaw_init,
            const float x_dev, const float y_dev, const float z_dev, 
            const float roll_dev, const float pitch_dev, const float yaw_dev);

  /*! \brief This function implements the PF prediction stage.
   * (Translation in X, Y and Z in meters and yaw angle incremenet in rad.)
   *
   * \param odom_x_mod Increased odometry in the position of the x-axis.
   * \param odom_y_mod Increased odometry in the position of the x-axis.
   * \param odom_z_mod Increased odometry in the position of the x-axis.
   * \param odom_a_mod Increased odometry in the position of the x-axis.
   * \param delta_x Thresholds of x-axis position in prediction.
   * \param delta_y Thresholds of y-axis position in prediction.
   * \param delta_z Thresholds of z-axis position in prediction.
   * \param delta_a Thresholds of yaw angle orientation in prediction.
   *
   * It calculates the increase that has occurred in the odometry and makes predictions of where it is possible that the
   * UAV is, taking into account selected thresholds.
   */
  void predict(const double delta_x, const double delta_y, const double delta_z,
              const double delta_roll,const double delta_pitch,const double delta_yaw,
              const double odom_x_noise, const double odom_y_noise, const double odom_z_noise,
              const double odom_roll_noise, const double odom_pitch_noise,const double odom_yaw_noise);

  /*! \brief This function implements the PF update stage.
   *
   * \param grid3d Instance of the Grid3d class.
   * \param cloud Point cloud from the UAV view.
   * \param range_data Information of the radio-range sensor.
   * \param alpha Percentage weight between point cloud and range sensor.
   * \param sigma Desviation in the measurement of the radio-range sensor.
   *
   * It takes the positions of the particles to change if they are on the map. Then, it evaluates the weight of the
   * particle according to the point cloud and the measurement of the radio sensors. Finally, it normalizes the weights
   * for all particles and finds the average for the composition of the UAV pose.
   */
  void update(const pcl::PointCloud<PointType>::Ptr& cloud);
  void update(const pcl::PointCloud<PointType>::Ptr& cloud,const int& sampleNum);

  void particleNormalize();

  /*! \brief This function implements the PF resample stage.
   * Translation in X, Y and Z in meters and yaw angle incremenet in rad.
   *
   * \param num_particles Particle number in the filter.
   * \param x_dev Init thresholds of x-axis position.
   * \param y_dev Init thresholds of y-axis position.
   * \param z_dev Init thresholds of z-axis position.
   * \param a_dev Init thresholds of yaw angle orientation.
   *
   * Sample the particle set again using low variance samples. So that the particles with less weights are discarded. To
   * complete the number of particles that the filter must have, new ones are introduced taking the average of those
   * that passed the resampling and applying the same variance thresholds that is applied in the prediction.
   */
  void resample();

  void uniformSample(int& sample_num);

  void setParticleNum(int max_sample = 800,int min_sample = 150);


public:
  std::vector<Particle> p_; /*!< Vector of particles */
  Particle mean_;           /*!< Particle to show the mean of all the particles */
  Grid3d* grid3d_;         /*!< Instance of the Grid3d class */

  LocalizationParam localization_params_;

  int max_resamp_num_,min_resamp_num_;

private:

  /*! \brief To generate the random value by the Gaussian distribution.
   *
   * \param mean Average of the distribution.
   * \param sigma Desviation of the distribution.
   * \return <b>float</b> - Random value.
   */
  float ranGaussian(const double mean, const double sigma);

  /*! \brief To generate the random between two values.
   *
   * \param range_from Lower end of range.
   * \param range_to Upper end of range.
   * \return <b>float</b> - Random value.
   */
  float rngUniform(const float range_from, const float range_to);

  bool initialized_{ false }; /*!< To indicate the initialition of the filter */

  std::random_device rd_;  /*!< Random device */
  std::mt19937 generator_; /*!< Generator of random values */

  VSCOMMON::LoggerPtr g_log;
};

}  // namespace amcl3d
