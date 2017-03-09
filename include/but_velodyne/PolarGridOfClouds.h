/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2015
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POLARGRIDOFCLOUDS_H_
#define POLARGRIDOFCLOUDS_H_

#include <velodyne_pointcloud/point_types.h>
#include <boost/array.hpp>
#include <cv.h>
#include <pcl/common/eigen.h>

#include <but_velodyne/VelodynePointCloud.h>

namespace but_velodyne
{

/**!
 * Identifier of the polar cell.
 */
class CellId {
public:
  int polar;    ///! discretized horizontal angle
  int ring;     ///! Velodyne ring ID

  CellId(int p, int r) :
   polar(p), ring(r) {
  }

  bool operator<(const CellId &other) const {
    return (this->ring < other.ring) ?
        true :
        (this->ring == other.ring) && (this->polar < other.polar);
  }

  bool operator==(const CellId &other) const {
    return (this->ring == other.ring) && (this->polar == other.polar);
  }
};

std::ostream& operator<< (std::ostream &out, const CellId &id);

/**!
 * Polar grid structure grouping the Velodyne 3D points of the same ring and the similar horizontal angle.
 */
class PolarGridOfClouds
{
public:

  typedef boost::shared_ptr<PolarGridOfClouds> Ptr;

  /**!
   * Redistributes the original 3D points into the polar grid structure
   *
   * @param point_cloud oroginal Velodyne point cloud
   */
  PolarGridOfClouds(const VelodynePointCloud &point_cloud, bool redistribute = false);

  static Ptr of(const VelodynePointCloud &point_cloud) {
    PolarGridOfClouds *grid = new PolarGridOfClouds(point_cloud);
    return Ptr(grid);
  }

  /**!
   * Visualization of the grouping into the polar bins - same bin = same color.
   */
  void showColored();

  PolarGridOfClouds::Ptr summarize() const;

  /**!
   * @param cellId identifier of the cell
   * @returns the points of the specific polar grid
   */
  const VelodynePointCloud& operator[](const CellId &cellId) const {
    return polar_grid[cellId.polar][cellId.ring];
  }

  const VelodynePointCloud& at(const CellId &cellId) const {
    return polar_grid[cellId.polar][cellId.ring];
  }

  VelodynePointCloud& at(const CellId &cellId) {
    return polar_grid[cellId.polar][cellId.ring];
  }

  /**!
   * Transforms the positions of points in the grid structure. Distribution of points
   * into the polar bins remains the same since only rigid transformation is assumed.
   *
   * @param t rigid 3D transformation [R|t]
   */
  void transform(const Eigen::Matrix4f &t);

  static int POLAR_SUPERBINS;                        ///!! number of polar super-bins
  static int BIN_SUBDIVISION;                        ///!! each bin is split into N sub-bins

  /**!
   * @return number of subdivided polar bins
   */
  static int getPolarBins() {
    return POLAR_SUPERBINS*BIN_SUBDIVISION;
  }

  const std::vector<CellId>& getIndices() {
    return indices;
  }

  template <typename PointT>
  static int getPolarBinIndex(const PointT &point, int bins = getPolarBins()) {
      float angle = VelodynePointCloud::horizontalAngle(point.z, point.x);

      // we want 0deg to point to the back
      angle += 180;
      if(angle >= 360) {
          angle -= 360;
      }

      float polar_bin_size = 360.0f / bins;
    return floor(angle/polar_bin_size);
  }

protected:

  PolarGridOfClouds();

  void fill(const VelodynePointCloud &point_cloud, bool redistribute);

  int computeNewRingIndex(const velodyne_pointcloud::PointXYZIR &point,
                          const std::vector<float> &borders);

  std::vector<
    boost::array<VelodynePointCloud, VelodynePointCloud::VELODYNE_RINGS_COUNT>
  > polar_grid;            // polar bins of rings

  std::vector<CellId> indices;
};

} /* namespace but_velodyne */

#endif /* POLARGRIDOFCLOUDS_H_ */
