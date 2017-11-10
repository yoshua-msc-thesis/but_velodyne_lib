/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
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

#include <boost/program_options.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/common.h>
#include <but_velodyne/NormalsEstimation.h>
#include <but_velodyne/GlobalOptimization.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv.h>
#include <cxeigen.hpp>
#include <ml.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
using namespace cv;
using namespace cv::ml;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     float &subsampling_rate, float &radius) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Velodyne Points Clustering\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("subsampling_rate,r", po::value<float>(&subsampling_rate)->default_value(0.1), "Subsampling rate.")
    ("radius", po::value<float>(&radius)->default_value(0.1), "Radius to consider.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_process.size() < 1) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);

  if(sensor_poses_filename.empty()) {
    calibration = SensorsCalibration();
  } else {
    calibration = SensorsCalibration(sensor_poses_filename);
  }

  return true;
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  float subsampling_rate, radius;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames,
      subsampling_rate, radius)) {
    return EXIT_FAILURE;
  }

  vector<Origin> sum_origins;
  PointCloud<PointXYZI>::Ptr sum_cloud(new PointCloud<PointXYZI>);

  VelodyneFileSequence sequence(filenames, calibration);
  for(int frame_i = 0; sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PointCloud<PointXYZI>::Ptr joined(new PointCloud<PointXYZI>);
    multiframe.joinTo(*joined);
    transformPointCloud(*joined, *joined, poses[frame_i]);
    *sum_cloud += *joined;
    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      vector<Origin> origins(multiframe.clouds[sensor_i]->size(), Origin(frame_i, sensor_i));
      sum_origins.insert(sum_origins.end(), origins.begin(), origins.end());
    }
  }

  vector<Origin> subsampled_origins;
  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  PointIndices::Ptr indices(new PointIndices);
  regular_subsampling<PointXYZI>(sum_cloud, subsampling_rate, indices, subsampled_cloud);
  extract_indices(sum_origins, indices->indices, subsampled_origins);

  PointCloud<Normal> subsampled_normals;
  getNormals(*subsampled_cloud, *sum_cloud,
      subsampled_origins, poses, calibration,
      radius,
      subsampled_normals);

  PointIndices::Ptr filtered_indices(new PointIndices);
  removeNaNNormalsFromPointCloud(subsampled_normals, subsampled_normals, filtered_indices->indices);
  extract_indices(subsampled_cloud, filtered_indices, *subsampled_cloud);
  extract_indices(indices->indices, filtered_indices->indices, indices->indices);

  io::savePCDFileBinary("normals.pcd", subsampled_normals);
  save_vector(indices->indices, "normals.indices");

  return EXIT_SUCCESS;
}
