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
#include <pcl/common/centroid.h>

#include <boost/program_options.hpp>
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/Visualizer2D.h>
#include <but_velodyne/Regular2DGridGenerator.h>
#include <but_velodyne/Regular2DGridProcessor.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;


#define log cerr

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

class HeightAverage : public Modificator<PointCloud<PointXYZ>, float> {
public:
  virtual void operator()(const PointCloud<PointXYZ> &cloud, float &height_avg) {
    if(cloud.empty()) {
      height_avg = NAN;
    } else {
      Eigen::Vector4f centroid;
      compute3DCentroid(cloud, centroid);
      height_avg = centroid.y();
    }
  }
};

class MoveDetection {
public:

  class Parameters {
  public:
    Parameters(
        int lines_generated_ = 100,
        int points_per_cell_ = 400) :
          lines_generated(lines_generated_),
          points_per_cell(points_per_cell_) {
    }

  public:
    int lines_generated;
    int points_per_cell;
  };

  MoveDetection(Parameters params_,
                AngularCollarLinesFilter &filter_,
                Regular2DGridGenerator::Parameters grid_params) :
    params(params_),
    filter(filter_),
    grid_generator(grid_params) {
  }

  // TODO return 2D bool grid
  void run(const VelodynePointCloud &new_cloud) {
    PolarGridOfClouds polar_grid(new_cloud);
    filter.addNewMaxRingRanges(new_cloud.getMaxOfRingRanges());
    LineCloud lines(polar_grid, params.lines_generated, filter);
    PointCloud<PointXYZ>::Ptr dense_cloud = lines.generateDenseCloud(params.points_per_cell);
    PointCloud<PointXYZ>::Ptr dense_downsampled_cloud = downsampleCloud<pcl::PointXYZ>(dense_cloud, 0.1);
    //Visualizer3D().addCloudColoredByHeight(*dense_downsampled_cloud).addRingColoredCloud(new_cloud).show();
    *dense_downsampled_cloud += *new_cloud.getXYZCloudPtr();

    Regular2DGrid< PointCloud<PointXYZ> > new_grid(grid_generator.params.rows, grid_generator.params.cols);
    grid_generator.generate(*dense_downsampled_cloud, new_grid);
    //Visualizer3D().addEvenOddColoredGrid(new_grid).show();

    Regular2DGrid<float> new_grid_height(grid_generator.params.rows, grid_generator.params.cols);
    HeightAverage height_avg;
    Regular2DGridProcessor::modify(new_grid, new_grid_height, height_avg);
    Visualizer2D(cv::Rect(0, 0, grid_generator.params.cols, grid_generator.params.rows), "HeightMap")
        .addHeightMap(new_grid_height).show(100);
  }

  Parameters params;
  AngularCollarLinesFilter &filter;
  Regular2DGridGenerator grid_generator;
};

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters,
                     AngularCollarLinesFilter::Parameters &filter_parameters,
                     vector<string> &clouds_to_process);

class Modif : public Modificator<float, int> {
public:
  virtual void operator()(const float &s, int &t) {
    t = round(s);
  }
};

class Sum : public Aggregator<float> {
public:
  virtual void operator()(const vector< boost::shared_ptr<float> > &s, float &t) {
    t = 0;
    for(int i = 0; i < s.size(); i++) {
      t += *s[i];
    }
  }
};

void gridProcessingTest() {
  Regular2DGrid<float>::Ptr fgrid(new Regular2DGrid<float>(2,2));
  *fgrid->at(0,0) = 1.2;
  *fgrid->at(0,1) = 2.4;
  *fgrid->at(1,0) = 3.6;
  *fgrid->at(1,1) = 4.8;
  Regular2DGrid<int>::Ptr igrid(new Regular2DGrid<int>(2,2));

  Modif m;
  Regular2DGridProcessor::modify(*fgrid, *igrid, m);
  cerr << *fgrid << endl;
  cerr << *igrid << endl;

  vector<Regular2DGrid<float>::Ptr> to_sum;
  to_sum.push_back(fgrid);
  to_sum.push_back(fgrid);
  to_sum.push_back(fgrid);
  Sum summ_f;
  Regular2DGrid<float> sum_targ(2,2);
  Regular2DGridProcessor::aggregate(to_sum, sum_targ, summ_f);
  cerr << sum_targ << endl;
}

/**
 * ./move-detection cloud.bin
 */
int main(int argc, char** argv) {

  MoveDetection::Parameters parameters;
  AngularCollarLinesFilter::Parameters filter_parameters;
  vector<string> clouds_to_process;
  if(!parse_arguments(argc, argv, parameters, filter_parameters, clouds_to_process)) {
    return EXIT_FAILURE;
  }

  for(vector<string>::iterator filename = clouds_to_process.begin(); filename < clouds_to_process.end(); filename++) {
    VelodynePointCloud new_cloud;
    log << "Processing KITTI file: " << *filename << endl << flush;
    if (filename->find(".pcd") != string::npos) {
      io::loadPCDFile(*filename, new_cloud);
    } else {
      VelodynePointCloud::fromKitti(*filename, new_cloud);
    }

    AngularCollarLinesFilter filter(CollarLinesFilter::HORIZONTAL_RANGE_DIFF, filter_parameters);
    Regular2DGridGenerator::Parameters grid_params;
    MoveDetection move_detection(parameters, filter, grid_params);
    move_detection.run(new_cloud);
  }

  return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv, MoveDetection::Parameters &parameters, AngularCollarLinesFilter::Parameters &filter_parameters,
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
      ("lines_preserved,p", po::value<int>(&filter_parameters.lines_to_preserve)->default_value(filter_parameters.lines_to_preserve),
          "How many collar lines are preserved per single polar bin after filtering")
      ("points_per_cell", po::value<int>(&parameters.points_per_cell)->default_value(parameters.points_per_cell),
          "How many points are generated within each polar bin")
      ("polar_superbins", po::value<int>(&PolarGridOfClouds::POLAR_SUPERBINS)->default_value(PolarGridOfClouds::POLAR_SUPERBINS),
          "Number of polar bins in the grid")
      ("bin_subdivision", po::value<int>(&PolarGridOfClouds::BIN_SUBDIVISION)->default_value(5),
          "How many times is the polar bin sub-divided")
      ("max_line_horizontal_diff", po::value<float>(&filter_parameters.max_horizontal_range_diff)->default_value(filter_parameters.max_horizontal_range_diff),
          "Max difference of horizontal ranges for preserved line")
      ("line_horizontal_diff_rel_tolerance", po::value<float>(&filter_parameters.horizontal_range_diff_tolerance_rel)->default_value(filter_parameters.horizontal_range_diff_tolerance_rel),
          "Relative tolerance of line horizontal range difference")
      ("line_horizontal_diff_abs_tolerance", po::value<float>(&filter_parameters.horizontal_range_diff_tolerance_abs)->default_value(filter_parameters.horizontal_range_diff_tolerance_abs),
          "Absolute tolerance of line horizontal range difference")
      ("horizontal_range_diff_weight", po::value<float>(&filter_parameters.weight_of_expected_horizontal_range_diff)->default_value(filter_parameters.weight_of_expected_horizontal_range_diff),
          "Weight of horizontal range difference estimation (value < 0 => MAX)")
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

