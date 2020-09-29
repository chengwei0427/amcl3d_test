/*!
 * @file Grid3d.cpp
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

#include "Grid3d.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace amcl3d
{
/*bool Grid3d::open(const std::string& map_path, const double sensor_dev)
{
  try
  {
    auto octo_tree = openOcTree(map_path); // Load octomap 

    ROS_INFO("[%s] Octomap loaded", ros::this_node::getName().data());

    pc_info_ = computePointCloud(octo_tree); // Compute the point-cloud associated to the octomap 

    ROS_INFO("[%s]"
             "\n   Map size:"
             "\n      X: %lf to %lf"
             "\n      Y: %lf to %lf"
             "\n      Z: %lf to %lf"
             "\n      Res: %lf",
             ros::this_node::getName().data(), pc_info_->octo_min_x, pc_info_->octo_max_x, pc_info_->octo_min_y,
             pc_info_->octo_max_y, pc_info_->octo_min_z, pc_info_->octo_max_z, pc_info_->octo_resol);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("[%s] %s", ros::this_node::getName().data(), e.what());
    return false;
  }

  // Try to load the associated grid-map from file 
  std::string grid_path;
  if (map_path.compare(map_path.length() - 3, 3, ".bt") == 0)
    grid_path = map_path.substr(0, map_path.find(".bt")) + ".grid";
  if (map_path.compare(map_path.length() - 3, 3, ".ot") == 0)
    grid_path = map_path.substr(0, map_path.find(".ot")) + ".grid";

  if (loadGrid(grid_path, sensor_dev))
    return true;

  // Compute the gridMap using kdtree search over the point-cloud 
  ROS_INFO("[%s] Computing 3D occupancy grid. This will take some time...", ros::this_node::getName().data());
  grid_info_ = computeGrid(pc_info_, sensor_dev);
  ROS_INFO("[%s] Computing 3D occupancy grid done!", ros::this_node::getName().data());

  // Save grid on file 
  saveGrid(grid_path);

  return true;
}*/

bool loadPCD(std::string file_path,pcl::PointCloud < pcl::PointXYZ>::Ptr& input)
{
  #define PointType pcl::PointXYZI
  std::vector<pcl::PointCloud<PointType>::Ptr> corner_keyframes;
  std::vector<pcl::PointCloud<PointType>::Ptr>  surf_keyframes;
  std::vector<pcl::PointCloud<PointType>::Ptr>  outlier_keyframes;
  pcl::PointCloud<PointType>::Ptr  keyposes_3d;
  // VSCOMMON::tic("loadpcd");
  pcl::PointCloud<PointType>::Ptr corner_pc(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_pc(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr outlier_pc(new pcl::PointCloud<PointType>());
  keyposes_3d.reset(new pcl::PointCloud<PointType>());

  if (pcl::io::loadPCDFile(file_path+"keypose_b.pcd", *keyposes_3d) == -1 || pcl::io::loadPCDFile(file_path+"corner_b.pcd", *corner_pc) == -1
    || pcl::io::loadPCDFile(file_path+"surf_b.pcd", *surf_pc) == -1 || pcl::io::loadPCDFile(file_path+"outlier_b.pcd", *outlier_pc) == -1)
  {
    std::cout << "couldn't load pcd file." << std::endl;
    return false;
  }
  std::cout<< __FUNCTION__<<" "<<__LINE__<<" load pcd file here."<< keyposes_3d->size()<<std::endl;
  corner_keyframes.resize(keyposes_3d->points.size());
  surf_keyframes.resize(keyposes_3d->points.size());
  outlier_keyframes.resize(keyposes_3d->points.size());
  for (int i = 0; i < keyposes_3d->points.size(); ++i)
  {
    corner_keyframes[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    surf_keyframes[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    outlier_keyframes[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  }
  for (int i = 0; i < corner_pc->points.size(); ++i)
  {
    const auto &p = corner_pc->points[i];
    corner_keyframes[int(p.intensity)]->points.push_back(p);
  }
  for (int i = 0; i < surf_pc->points.size(); ++i)
  {
    const auto &p = surf_pc->points[i];
    surf_keyframes[int(p.intensity)]->points.push_back(p);
  }
  for (int i = 0; i < outlier_pc->points.size(); ++i)
  {
    const auto &p = outlier_pc->points[i];
    outlier_keyframes[int(p.intensity)]->points.push_back(p);
  }
  // std::cout << __FUNCTION__ << ":takes: " << VSCOMMON::toc("loadpcd") * 1000 << "ms" << std::endl;
  // 
      pcl::PointCloud<PointType>::Ptr showcloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr showcloud_ds(new pcl::PointCloud<PointType>());
    for (int i = 0; i < keyposes_3d->points.size(); ++i)
    {
      *showcloud += *(surf_keyframes[i]);
      *showcloud += *(outlier_keyframes[i]);
      *showcloud += *(corner_keyframes[i]);
    }
    pcl::VoxelGrid<PointType> downsample;
    downsample.setLeafSize(0.5, 0.5, 0.5);
    downsample.setInputCloud(showcloud);
    downsample.filter(*showcloud_ds);
    //showCloud(showcloud_ds);
    //pcl::PointCloud < pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
      
    pcl::copyPointCloud(*showcloud_ds, *input);
    std::cout<<"load point cloud successful. "<<__LINE__<<std::endl;
  return true;
}

bool Grid3d::open(const std::string& map_path, const double sensor_dev)
{
  try
  {
    pcl::PointCloud < pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud < pcl::PointXYZ>());
    std::cout<<map_path<<std::endl;
    if(!loadPCD(map_path,cloud))
      return false;
    // auto octo_tree = openOcTree(map_path); // Load octomap 

    ROS_INFO("[%s] Octomap loaded", ros::this_node::getName().data());

    //pc_info_ = computePointCloud(octo_tree); // Compute the point-cloud associated to the octomap 
    pc_info_ = computePointCloud(cloud,0.5);

    ROS_INFO("[%s]"
             "\n   Map size:"
             "\n      X: %lf to %lf"
             "\n      Y: %lf to %lf"
             "\n      Z: %lf to %lf"
             "\n      Res: %lf",
             ros::this_node::getName().data(), pc_info_->octo_min_x, pc_info_->octo_max_x, pc_info_->octo_min_y,
             pc_info_->octo_max_y, pc_info_->octo_min_z, pc_info_->octo_max_z, pc_info_->octo_resol);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("[%s] %s", ros::this_node::getName().data(), e.what());
    return false;
  }

  // Try to load the associated grid-map from file 
  std::string grid_path = map_path+"cloud.grid";
  /*if (map_path.compare(map_path.length() - 3, 3, ".bt") == 0)
    grid_path = map_path.substr(0, map_path.find(".bt")) + ".grid";
  if (map_path.compare(map_path.length() - 3, 3, ".ot") == 0)
    grid_path = map_path.substr(0, map_path.find(".ot")) + ".grid";
*/
  if (loadGrid(grid_path, sensor_dev))
    return true;

  // Compute the gridMap using kdtree search over the point-cloud 
  ROS_INFO("[%s] Computing 3D occupancy grid. This will take some time...", ros::this_node::getName().data());
  grid_info_ = computeGrid(pc_info_, sensor_dev);
  ROS_INFO("[%s] Computing 3D occupancy grid done!", ros::this_node::getName().data());

  // Save grid on file 
  saveGrid(grid_path);

  return true;
}

bool Grid3d::buildGridSliceMsg(const double z, nav_msgs::OccupancyGrid& msg) const
{
  if (!grid_info_ || !pc_info_)
    return false;

  if (z < pc_info_->octo_min_z || z > pc_info_->octo_max_z)
    return false;

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = pc_info_->octo_resol;
  msg.info.width = grid_info_->size_x;
  msg.info.height = grid_info_->size_y;
  msg.info.origin.position.x = 0.;
  msg.info.origin.position.y = 0.;
  msg.info.origin.position.z = z;
  msg.info.origin.orientation.x = 0.;
  msg.info.origin.orientation.y = 0.;
  msg.info.origin.orientation.z = 0.;
  msg.info.origin.orientation.w = 1.;

  /* Extract max probability */
  const uint32_t init = point2grid(pc_info_->octo_min_x, pc_info_->octo_min_y, z);
  const uint32_t end = point2grid(pc_info_->octo_max_x, pc_info_->octo_max_y, z);
  float temp_prob, max_prob = -1.0;
  auto grid_ptr = grid_info_->grid.data();
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
    std::cout<< __FUNCTION__<<" "<<__LINE__<<" build grid slice msg successful "<<std::endl;
  return true;
}

bool Grid3d::buildMapPointCloudMsg(sensor_msgs::PointCloud2& msg) const
{
  if (!pc_info_ || !pc_info_->cloud)
    return false;

  pcl::toROSMsg(*pc_info_->cloud, msg);
std::cout<< __FUNCTION__<<" "<<__LINE__<<" build pointcloud msg successful "<<std::endl;
  return true;
}

float Grid3d::computeCloudWeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const float tx, const float ty,
                                 const float tz, const float roll, const float pitch, const float yaw) const
{
  if (!grid_info_ || !pc_info_)
    return 0;

  const auto sr = sin(roll);
  const auto cr = cos(roll);
  const auto sp = sin(pitch);
  const auto cp = cos(pitch);
  const auto sy = sin(yaw);
  const auto cy = cos(yaw);

  float r00, r01, r02, r10, r11, r12, r20, r21, r22;
  r00 = cy*cp; r01 = cy*sp*sr - sy*cr; r02 = cy*sp*cr + sy*sr;
  r10 = sy*cp; r11 = sy*sp*sr + cy*cr; r12 = sy*sp*cr - cy*sr;
  r20 =   -sp; r21 =            cp*sr; r22 =            cp*cr;

  const auto octo_size_x = pc_info_->octo_max_x - pc_info_->octo_min_x;
  const auto octo_size_y = pc_info_->octo_max_y - pc_info_->octo_min_y;
  const auto octo_size_z = pc_info_->octo_max_z - pc_info_->octo_min_z;

  const auto offset_x = tx - pc_info_->octo_min_x;
  const auto offset_y = ty - pc_info_->octo_min_y;
  const auto offset_z = tz - pc_info_->octo_min_z;

  auto grid_ptr = grid_info_->grid.data();
  const auto grid_size = grid_info_->grid.size();
  pcl::PointXYZ new_point;
  uint32_t ix, iy, iz;
  uint32_t grid_index;
  float weight = 0.;
  int n = 0;
  const float error_z = 0;

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin(); it != cloud->end(); ++it)
  {
    auto point = dynamic_cast<const pcl::PointXYZ*>(&(*it));
    if (point == nullptr)
      continue;

    new_point.x = point->x*r00 + point->y*r01 + (point->z + error_z)*r02 + offset_x;
    new_point.y = point->x*r10 + point->y*r11 + (point->z + error_z)*r12 + offset_y;
    new_point.z = point->x*r20 + point->y*r21 + (point->z + error_z)*r22 + offset_z;

    if (new_point.x >= 0 && new_point.x < octo_size_x && new_point.y >= 0 && new_point.y < octo_size_y &&
        new_point.z >= 0 && new_point.z < octo_size_z)
    {
      ix = static_cast<uint32_t>(floor(new_point.x / pc_info_->octo_resol));
      iy = static_cast<uint32_t>(floor(new_point.y / pc_info_->octo_resol));
      iz = static_cast<uint32_t>(floor(new_point.z / pc_info_->octo_resol));

      if (ix < grid_info_->size_x && iy < grid_info_->size_y && iz < grid_info_->size_z)
      {
        grid_index = ix + iy * grid_info_->step_y + iz * grid_info_->step_z;

        if(grid_index < grid_size)
        {
          weight += grid_ptr[grid_index].prob;
          n += 1;
        }
      }
    }
  }

  return (n <= 10) ? 0 : weight / n;
}

bool Grid3d::isIntoMap(const float x, const float y, const float z) const
{
  if (!pc_info_)
    return false;

  return !pc_info_ || (x >= pc_info_->octo_min_x && x < pc_info_->octo_max_x && y >= pc_info_->octo_min_y &&
                       y < pc_info_->octo_max_y && z >= pc_info_->octo_min_z && z < pc_info_->octo_max_z);
}

bool Grid3d::saveGrid(const std::string& grid_path)
{
  if (!grid_info_)
    return false;

  auto pf = fopen(grid_path.c_str(), "wb");
  if (!pf)
  {
    ROS_ERROR("[%s] Error opening file %s for writing", ros::this_node::getName().data(), grid_path.c_str());
    return false;
  }

  /* Write grid general info */
  fwrite(&grid_info_->size_x, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_->size_y, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_->size_z, sizeof(uint32_t), 1, pf);
  fwrite(&grid_info_->sensor_dev, sizeof(double), 1, pf);

  /* Write grid cells */
  const auto grid_size = grid_info_->size_x * grid_info_->size_y * grid_info_->size_z;
  fwrite(grid_info_->grid.data(), sizeof(Grid3dCell), grid_size, pf);

  fclose(pf);

  ROS_INFO("[%s] Grid map successfully saved on %s", ros::this_node::getName().data(), grid_path.c_str());

  return true;
}

bool Grid3d::loadGrid(const std::string& grid_path, const double sensor_dev)
{
  auto pf = fopen(grid_path.c_str(), "rb");
  if (!pf)
  {
    ROS_WARN("[%s] Error opening file %s for reading", ros::this_node::getName().data(), grid_path.c_str());
    return false;
  }

  grid_info_.reset(new Grid3dInfo());

  /* Read grid general info */
  fread(&grid_info_->size_x, sizeof(uint32_t), 1, pf);
  fread(&grid_info_->size_y, sizeof(uint32_t), 1, pf);
  fread(&grid_info_->size_z, sizeof(uint32_t), 1, pf);
  fread(&grid_info_->sensor_dev, sizeof(double), 1, pf);

  if (std::fabs(grid_info_->sensor_dev - sensor_dev) >= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN("[%s] Loaded sensorDev is different", ros::this_node::getName().data());
    return false;
  }

  grid_info_->step_y = grid_info_->size_x;
  grid_info_->step_z = grid_info_->size_x * grid_info_->size_y;

  /* Read grid cells */
  const auto grid_size = grid_info_->size_x * grid_info_->size_y * grid_info_->size_z;
  grid_info_->grid.resize(grid_size);
  fread(grid_info_->grid.data(), sizeof(Grid3dCell), grid_size, pf);

  fclose(pf);

  ROS_INFO("[%s] Grid map successfully loaded from %s", ros::this_node::getName().data(), grid_path.c_str());

  return true;
}

inline uint32_t Grid3d::point2grid(const float x, const float y, const float z) const
{
  return static_cast<uint32_t>((x - pc_info_->octo_min_x) / pc_info_->octo_resol) +
         static_cast<uint32_t>((y - pc_info_->octo_min_y) / pc_info_->octo_resol) * grid_info_->step_y +
         static_cast<uint32_t>((z - pc_info_->octo_min_z) / pc_info_->octo_resol) * grid_info_->step_z;
}

}  // namespace amcl3d