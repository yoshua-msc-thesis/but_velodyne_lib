/*
 * Odometry estimation by collar line segments of Velodyne scan.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
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

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

class CollarLinesRegistrationToMap {
public:
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;

  CollarLinesRegistrationToMap(CollarLinesRegistrationPipeline::Parameters pipeline_params_,
                               CollarLinesRegistration::Parameters registration_params_) :
                                 filter(pipeline_params_.linesPerCellPreserved),
                                 params(pipeline_params_),
                                 registration_params(registration_params_),
                                 indexed(false) {
  }

  void addToMap(const VelodynePointCloud &src_cloud,
      const Eigen::Affine3f &src_pose) {
    PolarGridOfClouds src_polar_grid(src_cloud);
    LineCloud src_line_cloud(src_polar_grid, params.linesPerCellGenerated, filter);
    src_line_cloud.transform(src_pose.matrix());
    lines_map += src_line_cloud;
    indexed = false;
  }

  Eigen::Matrix4f runRegistration(const VelodynePointCloud &target_cloud,
      const Eigen::Affine3f &init_pose) {
    if(!indexed) {
      buildKdTree();
    }
    PolarGridOfClouds target_polar_grid(target_cloud);
    LineCloud::Ptr target_line_cloud(new LineCloud(target_polar_grid, params.linesPerCellGenerated, filter));
    return registerLineClouds(lines_map, *target_line_cloud, init_pose.matrix());
  }

protected:

  void buildKdTree(void) {
    map_kdtree.setInputCloud(lines_map.line_middles.makeShared());
    indexed = true;
  }

  Eigen::Matrix4f registerLineClouds(const LineCloud &source,
      const LineCloud &target, const Eigen::Matrix4f &initial_transformation) {
    Termination termination(params.minIterations, params.maxIterations,
        params.maxTimeSpent, params.significantErrorDeviation,
        params.targetError);
    Eigen::Matrix4f transformation = initial_transformation;
    while (!termination()) {
      CollarLinesRegistration icl_fitting(source, target, registration_params,
          transformation);
      float error = icl_fitting.refine();
      termination.addNewError(error);
      transformation = icl_fitting.getTransformation();
    }
    return transformation;
  }

private:
  CollarLinesFilter filter;
  LineCloud lines_map;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree;
  bool indexed;
};

void read_lines(const string &fn, vector<string> &lines) {
  ifstream file(fn.c_str());
  string line;
  while (getline(file, line)) {
    lines.push_back(line);
  }
}

bool parse_arguments(int argc, char **argv,
    CollarLinesRegistration::Parameters &registration_parameters,
    CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
    vector<Eigen::Affine3f> &src_poses, vector<string> &src_clouds_filenames,
    vector<Eigen::Affine3f> &trg_poses, vector<string> &trg_clouds_filenames) {
  string source_poses_file, target_poses_file;
  string source_clouds_list, target_clouds_list;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("matching_threshold", po::value<CollarLinesRegistration::Threshold>(&registration_parameters.distance_threshold)->default_value(registration_parameters.distance_threshold),
      "How the value of line matching threshold is estimated (mean/median/... of line pairs distance). Possible values: MEDIAN_THRESHOLD|MEAN_THRESHOLD|NO_THRESHOLD")
    ("line_weightning", po::value<CollarLinesRegistration::Weights>(&registration_parameters.weighting)->default_value(registration_parameters.weighting),
      "How the weights are assigned to the line matches - prefer vertical lines, close or treat matches as equal. Possible values: DISTANCE_WEIGHTS|VERTICAL_ANGLE_WEIGHTS|NO_WEIGHTS")
    ("shifts_per_match", po::value<int>(&registration_parameters.correnspPerLineMatch)->default_value(registration_parameters.correnspPerLineMatch),
      "[Experimental] How many shift vectors (for SVD) are generated per line match - each is amended by small noise")
    ("shifts_noise_sigma", po::value<float>(&registration_parameters.lineCorrenspSigma)->default_value(registration_parameters.lineCorrenspSigma),
      "[Experimental] Deviation of noise generated for shift vectors (see above)")
    ("lines_per_bin_generated,g", po::value<int>(&pipeline_parameters.linesPerCellGenerated)->default_value(pipeline_parameters.linesPerCellGenerated),
      "How many collar lines are generated per single polar bin")
    ("lines_per_bin_preserved,p", po::value<int>(&pipeline_parameters.linesPerCellPreserved)->default_value(pipeline_parameters.linesPerCellPreserved),
      "How many collar lines are preserved per single polar bin after filtering")
    ("min_iterations", po::value<int>(&pipeline_parameters.minIterations)->default_value(pipeline_parameters.minIterations),
      "Minimal number of registration iterations (similar to ICP iterations)")
    ("max_iterations", po::value<int>(&pipeline_parameters.maxIterations)->default_value(pipeline_parameters.maxIterations),
      "Maximal number of registration iterations")
    ("max_time_for_registration", po::value<int>(&pipeline_parameters.maxTimeSpent)->default_value(pipeline_parameters.maxTimeSpent),
      "Maximal time for registration [sec]")
    ("target_error", po::value<float>(&pipeline_parameters.targetError)->default_value(pipeline_parameters.targetError),
      "Minimal error (average distance of line matches) causing termination of registration")
    ("significant_error_deviation", po::value<float>(&pipeline_parameters.significantErrorDeviation)->default_value(pipeline_parameters.significantErrorDeviation),
      "If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated")
    ("source_clouds_list", po::value<string>(&source_clouds_list)->required(),
      "List of source point clouds")
    ("target_clouds_list", po::value<string>(&target_clouds_list)->required(),
      "List of target point clouds")
    ("source_poses_file", po::value<string>(&source_poses_file)->required(),
      "File with source poses")
    ("target_poses_file", po::value<string>(&target_poses_file)->required(),
      "File with poses of target clouds for initialisation")
    ("nearest_neighbors", po::value<int>(&registration_parameters.nearestNeighbors)->default_value(registration_parameters.nearestNeighbors),
      "How many nearest neighbors (matches) are found for each line of source frame.")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
    std::cerr << desc << std::endl;
    return false;
  }

  try {
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }

  src_poses = KittiUtils::load_kitti_poses(source_poses_file);
  trg_poses = KittiUtils::load_kitti_poses(target_poses_file);
  read_lines(source_clouds_list, src_clouds_filenames);
  read_lines(target_clouds_list, trg_clouds_filenames);

  return true;
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  vector<Eigen::Affine3f> src_poses, trg_poses;
  vector<string> src_clouds_filenames, trg_clouds_filenames;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters,
      src_poses, src_clouds_filenames, trg_poses, trg_clouds_filenames)) {
    return EXIT_FAILURE;
  }

  CollarLinesRegistrationToMap registration(pipeline_parameters, registration_parameters);

  int map_size = MIN(src_poses.size(), src_clouds_filenames.size());
  for(int i = 0; i < map_size; i++) {
    VelodynePointCloud src_cloud;
    VelodynePointCloud::fromFile(src_clouds_filenames[i], src_cloud, false);
    registration.addToMap(src_cloud, src_poses[i]);
  }

  int clouds_to_register = MIN(trg_clouds_filenames.size(), trg_poses.size());
  for(int i = 0; i < clouds_to_register; i++) {
    VelodynePointCloud trg_cloud;
    VelodynePointCloud::fromFile(trg_clouds_filenames[i], trg_cloud, false);
    Eigen::Matrix4f delta = registration.runRegistration(trg_cloud, trg_poses[i]);
    KittiUtils::printPose(std::cout, trg_poses[i].matrix()*delta);
  }

  return EXIT_SUCCESS;
}
