/*!
 * @file Grid3d.h
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "PointCloudTools.h"

#include <common/vs_common.h>
#include <common/vs_timer.h>

namespace amcl3d
{
/*! \brief Class that contains the stages of the grid construction.
 */
class Grid3d
{
public:
  /*! \brief Grid3d class constructor.
   */
  explicit Grid3d(VSCOMMON::LoggerPtr&);
  /*! \brief Grid3d class destructor.
   */
  virtual ~Grid3d()
  {
  }

  /*! \brief To inicialite the grid map.
   *
   * \param map_path Route of the map environment.
   * \param sensor_dev Deviation of the point cloud sensor.
   * \return <b>bool=False</b> - If there are problems openning the octotree.
   * \return <b>bool=True</b> - If the grid map has been loaded and saved.
   *
   * It tries to load the octomap with the input route, showing that the loading was correct and the size that occupies
   * the octomap. If it can not load the octomap, it shuttles an error to indicate it and the program finish. But it
   * there is not any problem, the associated grid-map from the input route is looked up by loadGrid method. When it is
   * found, it is loaded. If there are not any grid-map associated, computeGrid method is called to
   * create it. Finally, the grid map is saved.
   */
  bool open(const std::string& map_path, const double sensor_dev);


  /*! \brief To calculate the particle weight.
   *
   * \param cloud Point cloud message.
   * \param tx Particle x-axis position.
   * \param ty Particle y-axis position.
   * \param tz Particle z-axis position.
   * \param a Particle yaw orientation.
   * \return <b>float</b> - Particle weight.
   *
   * It checks possible errors with the information of the grid and point cloud of the environment map. It uses the
   * information of this enviroment map and the particle to create a loop that records the likeness between the sensor
   * point cloud and the map. Depending on the resemblance the particle acquires the weight.
   */
  float computeCloudWeight(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const float tx, const float ty,
                           const float tz, const float roll, const float pitch, const float yaw) const;

  float computeCloudWeight(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const float tx, const float ty,
                           const float tz, const float roll, const float pitch, const float yaw, const int sampleNun) const;

  /*! \brief To check if the particle is into the map.
   *
   * \param x Particle x-axis position.
   * \param y Particle y-axis position.
   * \param z Particle z-axis position.
   * \return <b>bool=False</b> - If the particle is not into the map.
   * \return <b>bool=True</b> - If the particle is into the map.
   *
   * It looks up if there are problems with the information of the point cloud of the map to return false if this
   * happens. Then, it checks if the position of the particle is into the map using the extreme values of the octomap.
   */
  bool isIntoMap(const float x, const float y, const float z) const;

  bool loadPCD(std::string file_path,pcl::PointCloud < pcl::PointXYZI>::Ptr& input);

  bool updateMap(const float x, const float y, const float z,const float radius = 2,const float voxel_size = 0.5);

  /*! \brief To create a vector that contains the grid data.
   *
   * \param x Point x-axis position.
   * \param y Point y-axis position.
   * \param z Point z-axis position.
   * \return <b>uint32_t</b> - Vector with the information of the grid map.
   *
   * It needs the point of point cloud to do the relation with the octomap and the grid.
   * It turns the matrix into a vector so you can move through it. It was used in the "Grid3d: buildGridSliceMsg" to
   * extract the maximum probability touring said vector.
   */
  uint32_t point2grid(const float x, const float y, const float z) const;

  PointCloudInfo::Ptr pc_info_; /*!< 3D point cloud representation of the map. */

  Grid3dInfo::Ptr grid_info_; /*!< 3D probabilistic grid cell */

  std::vector<pcl::PointCloud<PointType>::Ptr> corner_keyframes;
  std::vector<pcl::PointCloud<PointType>::Ptr>  surf_keyframes;
  std::vector<pcl::PointCloud<PointType>::Ptr>  outlier_keyframes;
  pcl::PointCloud<PointType>::Ptr  keyposes_3d;

  pcl::KdTreeFLANN<PointType>::Ptr kd_map_ptr_;
  pcl::KdTreeFLANN<PointType>::Ptr kd_pose_ptr_;
private:
  /*! \brief To save the file of map like grid.
   *
   * \param grid_path Route where the grid map will be saved.
   * \return <b>bool=False</b> - If the map has not been save correctly.
   * \return <b>bool=True</b> - If the map has been save correctly.
   *
   * It checks if the grid information has error to return false. If there are not problems, it creates a file checking
   * the procees of creation. When the file is created, the information is added.
   */
  bool saveGrid(const std::string& grid_path);

  /*! \brief To load the grid file.
   *
   * \param grid_path Route where the grid map will be saved.
   * \param sensor_dev Deviation of the point cloud sensor.
   * \return <b>bool=False</b> - If there are problems with the grid map information or writting the file.
   * \return <b>bool=True</b> - If the grid map file is saved without problems.
   *
   * It checks if there are problems reading the file and if the desviation of the sensor is diferent between the grid
   * and the variable that user set. Then, the method reads the grid cells and close the file.
   */
  bool loadGrid(const std::string& grid_path, const double sensor_dev);


  VSCOMMON::LoggerPtr g_log;
};

}  // namespace amcl3d
