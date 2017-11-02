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
#include <pcl/registration/transformation_estimation_svd.h>

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

class SubseqRegistration {
public:
  SubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
      const Eigen::Affine3f &init_transform_,
      CollarLinesRegistrationPipeline::Parameters &params_,
      CollarLinesRegistration::Parameters &registration_params_) :
    src_lines(src_lines_), trg_lines(trg_lines_),
    params(params_), registration_params(registration_params_),
    estimated_transform(init_transform_) {
  }

  virtual ~SubseqRegistration() {}

  virtual Eigen::Affine3f run() {
    return Eigen::Affine3f(
        registerLineClouds(src_lines, trg_lines,
                           estimated_transform.matrix(),
                           registration_params, params));
  }

protected:

  Eigen::Matrix4f registerLineClouds(
      const LineCloud &source, const LineCloud &target,
      const Eigen::Matrix4f &initial_transformation,
      CollarLinesRegistration::Parameters registration_params,
      CollarLinesRegistrationPipeline::Parameters pipeline_params) {
    Termination termination(pipeline_params.minIterations, pipeline_params.maxIterations,
        pipeline_params.maxTimeSpent, pipeline_params.significantErrorDeviation,
        pipeline_params.targetError);
    Eigen::Matrix4f transformation = initial_transformation;
    CollarLinesRegistration icl_fitting(source, target, registration_params,
        transformation);
    while (!termination()) {
      float error = icl_fitting.refine();
      termination.addNewError(error);
      transformation = icl_fitting.getTransformation();
    }
    return transformation;
  }

  Eigen::Affine3f estimated_transform;
  LineCloud src_lines, trg_lines;
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;
};

class ManualSubseqRegistration : public SubseqRegistration {
public:
  ManualSubseqRegistration(const LineCloud &src_lines_, const LineCloud &trg_lines_,
      const Eigen::Affine3f &init_transform_,
      CollarLinesRegistrationPipeline::Parameters &params_,
      CollarLinesRegistration::Parameters &registration_params_) :
    SubseqRegistration(src_lines_, trg_lines_,
        init_transform_,
        params_, registration_params_),
    split_idx(src_lines_.line_cloud.size()) {

    pclVis = visualizer.getViewer();
    pclVis->registerPointPickingCallback(&ManualSubseqRegistration::pickPointCallback, *this);
    pclVis->registerKeyboardCallback(&ManualSubseqRegistration::keyCallback, *this);
  }

  Eigen::Affine3f run() {
    setDataToVisualizer();
    visualizer.show();
    return estimated_transform;
  }

protected:

  void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*) {
    int idx = event.getPointIndex();
    if (idx == -1)
      return;

    if(idx < split_idx) {
      if(src_indices.size() < trg_indices.size()) {
        src_indices.push_back(idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked source point but expected target - ignoring\n");
      }
    } else {
      if(src_indices.size() == trg_indices.size()) {
        trg_indices.push_back(idx-split_idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked target point but expected source - ignoring\n");
      }
    }
  }

  void setDataToVisualizer() {
    PointCloud<PointXYZRGB>::Ptr sum_cloud(new PointCloud<PointXYZRGB>);
    *sum_cloud += *Visualizer3D::colorizeCloud(src_lines.line_middles, 255, 0, 0);
    PointCloud<PointXYZ> trg_cloud_transformed;
    transformPointCloud(trg_lines.line_middles, trg_cloud_transformed, estimated_transform);
    *sum_cloud += *Visualizer3D::colorizeCloud(trg_cloud_transformed, 0, 0, 255);
    pclVis->removeAllShapes();
    pclVis->removeAllPointClouds();
    visualizer.addColorPointCloud(sum_cloud);
    for(int i = 0; i < src_indices.size(); i++) {
      visualizer.addArrow(PointCloudLine(src_lines.line_middles[src_indices[i]],
                                         trg_cloud_transformed[trg_indices[i]]));
    }
    if(src_indices.size() < trg_indices.size()) {
      pclVis->addSphere(trg_cloud_transformed[trg_indices.back()], 0.1, "sphere");
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "s") {
        PCL_DEBUG("Running transformation estimation using SVD ...\n");
        estimateManualTransform();
        setDataToVisualizer();
      } else if(event.getKeySym() == "u") {
        if(src_indices.size() < trg_indices.size() && !trg_indices.empty()) {
          trg_indices.pop_back();
        } else if(!src_indices.empty()) {
          src_indices.pop_back();
        }
        setDataToVisualizer();
      } else if(event.getKeySym() == "a") {
        runAutomaticRegistration(CollarLinesRegistration::NO_THRESHOLD);
        setDataToVisualizer();
      } else if(event.getKeySym() == "m") {
        runAutomaticRegistration(CollarLinesRegistration::MEDIAN_THRESHOLD);
        setDataToVisualizer();
      } else if(event.getKeySym() == "n") {
        runAutomaticRegistration(CollarLinesRegistration::QUARTER_THRESHOLD);
        setDataToVisualizer();
      } else if(event.getKeySym() == "b") {
        runAutomaticRegistration(CollarLinesRegistration::TENTH_THRESHOLD);
        setDataToVisualizer();
      }
    }
  }

  void estimateManualTransform() {
    if(src_indices.size() > 2) {
      pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> tfe;
      Eigen::Matrix4f t;
      tfe.estimateRigidTransformation(trg_lines.line_middles, trg_indices,
          src_lines.line_middles, src_indices, t);
      estimated_transform = Eigen::Affine3f(t);
    } else {
      PCL_WARN("Unable to estimate transformation with less than 3 matches - skipping.\n");
    }
  }

  void runAutomaticRegistration(CollarLinesRegistration::Threshold th_type) {
    PCL_DEBUG("Running automatic transformation estimation using CLS ...\n");
    registration_params.distance_threshold = th_type;
    estimated_transform = Eigen::Affine3f(registerLineClouds(src_lines, trg_lines,
        estimated_transform.matrix(), registration_params, params));
  }

  int split_idx;
  Visualizer3D visualizer;
  pcl::visualization::PCLVisualizer::Ptr pclVis;
  vector<int> src_indices;
  vector<int> trg_indices;
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
    Eigen::Affine3f &init_transform,
    vector<Eigen::Affine3f> &src_poses, vector<string> &src_clouds_filenames,
    vector<Eigen::Affine3f> &trg_poses, vector<string> &trg_clouds_filenames,
    SensorsCalibration &calibration,
    bool &manual) {
  string source_poses_file, target_poses_file;
  string source_clouds_list, target_clouds_list;
  string init_transform_filename, calibration_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
  ;

  registration_parameters.prepareForLoading(desc);

  pipeline_parameters.prepareForLoading(desc);

  desc.add_options()
    ("source_clouds_list", po::value<string>(&source_clouds_list)->required(),
      "List of source point clouds")
    ("target_clouds_list", po::value<string>(&target_clouds_list)->required(),
      "List of target point clouds")
    ("source_poses_file", po::value<string>(&source_poses_file)->required(),
      "File with source poses")
    ("target_poses_file", po::value<string>(&target_poses_file)->required(),
      "File with poses of target clouds for initialisation")
    ("init_transform,i", po::value<string>(&init_transform_filename)->default_value(""),
      "Transform for initialisation (pose file)")
    ("manual", po::bool_switch(&manual),
      "Enable manual verification and correction.")
    ("calibration,c", po::value<string>(&calibration_filename)->default_value(""),
        "Calibration file (sensors poses).")
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

  if(!init_transform_filename.empty()) {
    init_transform = KittiUtils::load_kitti_poses(init_transform_filename).front();
  } else {
    init_transform = Eigen::Affine3f::Identity();
  }

  if(calibration_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(calibration_filename);
  }

  return true;
}

void buildLineCloud(const vector<string> &cloud_files,
    const vector<Eigen::Affine3f> &poses,
    const SensorsCalibration &calibration,
    int lines_per_cell_gen, int lines_per_cell_preserve,
    LineCloud &out_lines) {
  CollarLinesFilter filter(lines_per_cell_preserve);
  VelodyneFileSequence sequence(cloud_files, calibration);
  for(int i = 0; sequence.hasNext(); i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PolarGridOfClouds polar_grid(multiframe.clouds, multiframe.calibration);
    LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
    line_cloud.transform(poses[i].matrix());
    out_lines += line_cloud;
  }
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

  CollarLinesRegistration::Parameters registration_parameters;
  CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
  Eigen::Affine3f init_transform;
  vector<Eigen::Affine3f> src_poses, trg_poses;
  vector<string> src_clouds_filenames, trg_clouds_filenames;
  SensorsCalibration calibration;
  bool manual;

  if (!parse_arguments(argc, argv,
      registration_parameters, pipeline_parameters,
      init_transform,
      src_poses, src_clouds_filenames, trg_poses, trg_clouds_filenames,
      calibration, manual)) {
    return EXIT_FAILURE;
  }

  LineCloud src_lines, trg_lines;
  buildLineCloud(src_clouds_filenames, src_poses,
      calibration,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      src_lines);
  buildLineCloud(trg_clouds_filenames, trg_poses,
      calibration,
      pipeline_parameters.linesPerCellGenerated, pipeline_parameters.linesPerCellPreserved,
      trg_lines);

  Eigen::Affine3f t;
  if(manual) {
    ManualSubseqRegistration registration(src_lines, trg_lines,
        init_transform,
        pipeline_parameters,
        registration_parameters);
    t = registration.run();
  } else {
    SubseqRegistration registration(src_lines, trg_lines,
        init_transform,
        pipeline_parameters,
        registration_parameters);
    t = registration.run();
  }
  KittiUtils::printPose(std::cout, t.matrix());

  return EXIT_SUCCESS;
}
