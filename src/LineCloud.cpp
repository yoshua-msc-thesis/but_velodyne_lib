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

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/program_options/errors.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <but_velodyne/LineCloud.h>

using namespace std;
using namespace pcl;

namespace but_velodyne
{

std::istream& operator>> (std::istream &in, LineCloud::PreservedFactorBy &factor_type) {
  string token;
  in >> token;

  boost::to_upper(token);

  if (token == "ANGLE_WITH_GROUND") {
    factor_type = LineCloud::ANGLE_WITH_GROUND;
  } else if (token == "NONE") {
    factor_type = LineCloud::NONE;
  } else {
      throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value,
                                                     "lines_preserved_factor_by");
  }

  return in;
}


LineCloud::LineCloud(const PolarGridOfClouds &polar_grid,
                     const int lines_per_cell_pair_generated,
                     const int lines_per_cell_pair_preserved,
                     const PreservedFactorBy preservedFactorType) :
    rng(cv::theRNG()),
    preservedFactorType(preservedFactorType)
{
  for(int polar = 0; polar < PolarGridOfClouds::POLAR_BINS; polar++) {
    for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT-1; ring++) {
      vector<PointCloudLine> lines_among_cells;
      generateLineCloudFromCell(polar_grid,
                                CellId(polar, ring),
                                lines_per_cell_pair_generated,
                                lines_per_cell_pair_preserved,
                                lines_among_cells);
      line_cloud.insert(line_cloud.end(),
                        lines_among_cells.begin(), lines_among_cells.end());
      for(vector<PointCloudLine>::iterator line = lines_among_cells.begin();
          line < lines_among_cells.end();
          line++) {
        Eigen::Vector3f middle = line->middle();
        line_middles.push_back(PointXYZ(middle.x(), middle.y(), middle.z()));
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LineCloud::generateDenseCloud(
    const PolarGridOfClouds &polar_grid,
    const int lines_per_cell_pair_generated,
    const int lines_per_cell_pair_preserved,
    const int points_per_cell) const {

  pcl::PointCloud<pcl::PointXYZ>::Ptr generated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int cloud_index = 0;

  for(int polar = 0; polar < PolarGridOfClouds::POLAR_BINS; polar++) {
   for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT-1; ring++) {
     std::vector<PointCloudLine> lines_among_cells;
     generateLineCloudFromCell(polar_grid,
                               CellId(polar, ring),
                               lines_per_cell_pair_generated,
                               lines_per_cell_pair_preserved,
                               lines_among_cells);
     if(lines_among_cells.size() == 0) {
       continue;
     }
     int points_per_line = MAX(points_per_cell/lines_among_cells.size(), 1);
     int additional_capacity = lines_among_cells.size()*points_per_line;
     //cerr << "points_per_line: " << points_per_line << endl << flush;
     //cerr << "additional_capacity: " << additional_capacity << endl << flush;
     //cerr << "lines_among_cells.size(): " << lines_among_cells.size() << endl << flush;
     //cerr << "generated_cloud->size(): " << generated_cloud->size() << endl << flush;
     generated_cloud->resize(generated_cloud->size() + additional_capacity);
     //cerr << "generated_cloud->size(): " << generated_cloud->size() << endl << flush;
     for(std::vector<PointCloudLine>::iterator line = lines_among_cells.begin();
         line < lines_among_cells.end();
         line++) {
       float max_factor = line->orientation.norm();
       for(int i = 0; i < points_per_line; i++) {
         float factor = rng.uniform(0.0f, max_factor);
         Eigen::Vector3f new_point = line->point + line->getOrientationOfSize(factor);
         generated_cloud->points[cloud_index++].getVector3fMap() = new_point;
       }
     }
   }
  }
  return generated_cloud;
}

float horizontalRangeDiffSq(const PointCloudLine &line) {
  Eigen::Vector3f start = line.point;
  Eigen::Vector3f end = line.point + line.orientation;
  return pow(start.x() - end.x(), 2) + pow(start.z() - end.z(), 2);
}

bool orderLinesByHorizontalRangeDiff(const PointCloudLine &first, const PointCloudLine &second) {
  return horizontalRangeDiffSq(first) < horizontalRangeDiffSq(second);
}

void LineCloud::generateLineCloudAmongCells(const PolarGridOfClouds &polar_grid,
                                            const CellId &cell1_id, const CellId &cell2_id,
                                            const int lines_per_cell_pair_generated,
                                            const int lines_per_cell_pair_preserved,
                                            vector<PointCloudLine> &output_lines) const {
  const VelodynePointCloud& cell1 = polar_grid[cell1_id];
  const VelodynePointCloud& cell2 = polar_grid[cell2_id];
  int lines_to_generate = MIN(lines_per_cell_pair_generated, cell1.size()*cell2.size());
  VelodynePointCloud all_points;
  all_points += cell1; all_points += cell2;
  float preserved_factor = getPreservedFactor(all_points);
  int lines_to_preserve = MIN(lines_per_cell_pair_preserved*preserved_factor,
                              lines_to_generate);
  assert(lines_to_generate >= lines_to_preserve);

  for(int i = 0; i < lines_to_generate; i++) {
    int cell1_index = rng(cell1.size());
    int cell2_index = rng(cell2.size());
    PointCloudLine generated_line(cell1[cell1_index],
                                  cell2[cell2_index]);
    output_lines.push_back(generated_line);
  }
  sort(output_lines.begin(), output_lines.end(), orderLinesByHorizontalRangeDiff);
  output_lines.erase(output_lines.begin()+lines_to_preserve, output_lines.end());
}

void LineCloud::generateLineCloudFromCell(const PolarGridOfClouds &polar_grid,
                               const CellId &source_cell,
                               const int lines_per_cell_pair_generated,
                               const int lines_per_cell_pair_preserved,
                               std::vector<PointCloudLine> &line_cloud) const {
  vector<CellId> target_cells = getTargetCells(source_cell);
  for(vector<CellId>::iterator target_cell = target_cells.begin(); target_cell < target_cells.end(); target_cell++) {
    generateLineCloudAmongCells(polar_grid,
                                source_cell, *target_cell,
                                lines_per_cell_pair_generated,
                                lines_per_cell_pair_preserved,
                                line_cloud);
  }
}

vector<CellId> LineCloud::getTargetCells(const CellId &source_cell) const {
  vector<CellId> target_cells;

  int min_polar = source_cell.polar - PolarGridOfClouds::BIN_SUBDIVISION / 2;
  int max_polar = source_cell.polar + PolarGridOfClouds::BIN_SUBDIVISION / 2;

  for(int polar = min_polar; polar <= max_polar; polar++) {
    int polar_periodic = (polar + PolarGridOfClouds::POLAR_BINS) % PolarGridOfClouds::POLAR_BINS;
    target_cells.push_back(CellId(polar_periodic, source_cell.ring+1));
  }

  return target_cells;
}

float LineCloud::sinOfPlaneAngleWithGround(const VelodynePointCloud &points) const {
  NormalEstimation<PointXYZ, Normal> normal_est;
  vector<int> indices;
  for(int i = 0; i < points.size(); i++) {
    indices.push_back(i);
  }
  float nx, ny, nz, c;
  normal_est.computePointNormal(*(points.getXYZCloudPtr()), indices, nx, ny, nz, c);
  float angle_cos = ny / sqrt(nx*nx + ny*ny + nz*nz);
  float angle_sin = sqrt(1 - angle_cos*angle_cos);
  return (isnan(angle_sin)) ? 1.0 : angle_sin;
}

void LineCloud::transform(const Eigen::Matrix4f &transformation, LineCloud &output) const {
  output.line_cloud.clear();
  for(std::vector<PointCloudLine>::const_iterator line = line_cloud.begin();
      line < line_cloud.end(); line++) {
    output.line_cloud.push_back(line->transform(transformation));
  }
  pcl::transformPointCloud(line_middles, output.line_middles, transformation);
}

void LineCloud::transform(const Eigen::Matrix4f &transformation) {
  for(std::vector<PointCloudLine>::iterator line = line_cloud.begin();
      line < line_cloud.end(); line++) {
    *line = line->transform(transformation);
  }
  pcl::transformPointCloud(line_middles, line_middles, transformation);
}

float LineCloud::getPreservedFactor(const VelodynePointCloud &all_points) const {
  switch(preservedFactorType) {
    case ANGLE_WITH_GROUND:
      return sinOfPlaneAngleWithGround(all_points) + 1.0;
    default:
      assert(preservedFactorType == NONE);
      return 1.0;
  }
}

} /* namespace but_velodyne */
