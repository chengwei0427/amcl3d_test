/*!
 * @file Parameters.h
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

#include <string>

namespace amcl3d
{
/*! \brief Class that contains the global parameters of the algorithm.
 */
struct Parameters
{
public:
  /*! \brief Parameters class constructor.
   */
  //explicit Parameters();

  std::string base_frame_id_;   /*!< Name of the robot TF */
  std::string odom_frame_id_;   /*!< Name of the flight origin of the robot TF */
  std::string global_frame_id_; /*!< Name of the test-bed origin TF */
  std::string map_path_;        /*!< Route to the localization of the environment map  */

  bool set_initial_pose_; /*!< Flag to indicate if t he initial pose has been received */

  double init_x_; /*!< Start x-axis position */
  double init_y_; /*!< Start y-axis position */
  double init_z_; /*!< Start z-axis position */
  double init_roll_;
  double init_pitch_;
  double init_yaw_; /*!< Start yaw angle */



  double grid_slice_z_;             /*!< Height of grid slice */
  double publish_point_cloud_rate_; /*!< Map point cloud publishing rate */
  double publish_grid_slice_rate_;  /*!< map grid slice publishing rate */

  double sensor_dev_;   /*!< Desviation of 3D point cloud sensor */
  double voxel_size_;   /*!< Size of voxel grid filter */

  int num_particles_; /*!< Particle number in the filter */

  int resample_interval_; /*!< Resampling control */

  double update_rate_; /*!< Filter updating frequency */
  double d_th_;        /*!< Threshold in the distance for the update */
  double a_th_;        /*!< Threshold in yaw angle for the update */

private:
  /*! \brief To check parameter errors.
   *
   * \param parameter_str Name of parameter.
   *
   * It give a error if the input parameter was not set.
   */
  void exitWithParameterError(const char* parameter_str);
};

}  // namespace amcl3d
