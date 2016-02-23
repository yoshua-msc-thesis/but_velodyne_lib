/*
 * Detection of moving objects.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 18/02/2016
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

#include <cstdlib>
#include <cstdio>
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/program_options.hpp>
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


#define log cerr

class Parameters {
public:
  Parameters(
      int lines_generated_ = 100,
      int lines_preserved_ = 50,
      int points_per_cell_ = 400,
      float horizontal_range_diff_tolerance_rel_ = 1.5,
      float horizontal_range_diff_tolerance_abs_ = 0.3,
      float max_horizontal_range_diff_ = 3.0) :
        lines_generated(lines_generated_),
        lines_preserved(lines_preserved_),
        points_per_cell(points_per_cell_),
        horizontal_range_diff_tolerance_rel(horizontal_range_diff_tolerance_rel_),
        horizontal_range_diff_tolerance_abs(horizontal_range_diff_tolerance_abs_),
        max_horizontal_range_diff(max_horizontal_range_diff_) {
  }

public:
  int lines_generated;
  int lines_preserved;
  int points_per_cell;
  float horizontal_range_diff_tolerance_rel;
  float horizontal_range_diff_tolerance_abs;
  float max_horizontal_range_diff;
};

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr downsampleCloud(
                typename pcl::PointCloud<PointType>::Ptr input,
                double resolution = 0.005f) {

    pcl::VoxelGrid<PointType> vg;
    typename pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
    vg.setInputCloud (input);
    vg.setLeafSize (resolution, resolution, resolution);
    vg.filter (*cloud_filtered);

    return cloud_filtered;
}

PointCloud<PointXYZRGB>::Ptr color_rings(const VelodynePointCloud &cloud) {
  uchar red = 0;
  PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
  for(int r = 0; r < VelodyneSpecification::RINGS; r++, red+=255/VelodyneSpecification::RINGS) {
    float range = VelodyneSpecification::getExpectedRange(r, VelodyneSpecification::KITTI_HEIGHT);
    //cerr << "ring: " << r << " range: " << range << endl;
    for(VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
      if(pt->ring == r) {
        uchar red, green, blue;
        //Visualizer3D::colorizeIntensity(r/(float)(VelodyneSpecification::RINGS-1), red, green, blue);
        red = green = blue = 0.0;
        if(r%3 == 0) {
          red = 255;
        } else if(r%3 == 1) {
          green = 255;
        } else {
          blue = 255;
        }
        PointXYZRGB colored_pt(red, green, blue);
        colored_pt.x = pt->x;
        colored_pt.y = pt->y;
        colored_pt.z = pt->z;
        colored_cloud->push_back(colored_pt);
      }
    }
  }
  return colored_cloud;
}

bool parse_arguments(int argc, char **argv, Parameters &parameters,
                     vector<string> &clouds_to_process);

/**
 * ./move-detection cloud.bin
 */
int main(int argc, char** argv) {

  Parameters parameters;
  vector<string> clouds_to_process;
  if(!parse_arguments(argc, argv, parameters, clouds_to_process)) {
    return EXIT_FAILURE;
  }
  string filename(clouds_to_process[0]);

  VelodynePointCloud original_cloud;
  log << "KITTI file: " << filename << endl << flush;
  if (filename.find(".pcd") != string::npos) {
    io::loadPCDFile(filename, original_cloud);
  } else {
    VelodynePointCloud::fromKitti(filename, original_cloud);
  }

  Visualizer3D::getCommonVisualizer()->setColor(0,0,200).addColorPointCloud(color_rings(original_cloud));
  PolarGridOfClouds polar_grid(original_cloud);
  AngularCollarLinesFilter filter(parameters.lines_preserved, CollarLinesFilter::HORIZONTAL_RANGE_DIFF,
                                  parameters.horizontal_range_diff_tolerance_rel,
                                  parameters.horizontal_range_diff_tolerance_abs,
                                  parameters.max_horizontal_range_diff);
  LineCloud lines(polar_grid, parameters.lines_generated, filter);

  /*Visualizer3D vis;
  vis.setColor(0,0,200).addPointCloud(original_cloud);
  for(vector<PointCloudLine>::const_iterator l = lines.getLines().begin(); l < lines.getLines().end(); l++) {
    if(l->point.z() > 5 && fabs(l->point.x()) < 2) {
      vis.addLine(*l);
    }
  }
  vis.show();*/

  PointCloud<PointXYZ>::Ptr dense_cloud = lines.generateDenseCloud(parameters.points_per_cell);

  PointCloud<PointXYZ>::Ptr downsampled_cloud = downsampleCloud<pcl::PointXYZ>(dense_cloud, 0.2);

  Visualizer3D().addCloudColoredByHeight(*downsampled_cloud).addColorPointCloud(color_rings(original_cloud)).show();

  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv, Parameters &parameters,
                     vector<string> &clouds_to_process) {
  bool use_kalman = false;
  int linear_estimator = 3;

  po::options_description desc("Detection of moving objects\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("lines_generated,g", po::value<int>(&parameters.lines_generated)->default_value(parameters.lines_generated),
          "How many collar lines are generated per single polar bin")
      ("lines_preserved,p", po::value<int>(&parameters.lines_preserved)->default_value(parameters.lines_preserved),
          "How many collar lines are preserved per single polar bin after filtering")
      ("points_per_cell", po::value<int>(&parameters.points_per_cell)->default_value(parameters.points_per_cell),
          "How many points are generated within each polar bin")
      ("polar_superbins", po::value<int>(&PolarGridOfClouds::POLAR_SUPERBINS)->default_value(PolarGridOfClouds::POLAR_SUPERBINS),
          "Number of polar bins in the grid")
      ("bin_subdivision", po::value<int>(&PolarGridOfClouds::BIN_SUBDIVISION)->default_value(5),
          "How many times is the polar bin sub-divided")
      ("max_line_horizontal_diff", po::value<float>(&parameters.max_horizontal_range_diff)->default_value(parameters.max_horizontal_range_diff),
          "Max difference of horizontal ranges for preserved line")
      ("line_horizontal_diff_rel_tolerance", po::value<float>(&parameters.horizontal_range_diff_tolerance_rel)->default_value(parameters.horizontal_range_diff_tolerance_rel),
          "Relative tolerance of line horizontal range difference")
      ("line_horizontal_diff_abs_tolerance", po::value<float>(&parameters.horizontal_range_diff_tolerance_abs)->default_value(parameters.horizontal_range_diff_tolerance_abs),
          "Absolute tolerance of line horizontal range difference")
   ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1)
    {
        std::cerr << desc << std::endl;
        return false;
    }

    try
    {
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    return true;
}

