/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 19/04/2017
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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
    vector<Eigen::Affine3f> &poses,
    vector<Eigen::Affine3f> &sensor_poses,
    vector<string> &clouds_to_process) {
  string pose_filename;
  string sensor_poses_filename;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
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
    if(!sensor_poses_filename.empty()) {
      sensor_poses = KittiUtils::load_kitti_poses(sensor_poses_filename);
    } else {
      sensor_poses.push_back(Eigen::Affine3f::Identity());
    }
    return true;
}


int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses, sensor_poses;
  if(!parse_arguments(argc, argv,
      poses, sensor_poses, filenames)) {
    return EXIT_FAILURE;
  }

  Eigen::Vector4f last_centroid;
  Eigen::Vector4f translation = Eigen::Vector4f::Zero();
  bool initialized = false;
  int pose_index = 0;
  VelodyneFileSequence fileSequence(filenames, sensor_poses);
  while(fileSequence.hasNext()) {
    VelodyneMultiFrame multiframe = fileSequence.getNext();
    PointCloud<velodyne_pointcloud::VelodynePoint> cloud;
    multiframe.joinTo(cloud);
    transformPointCloud(cloud, cloud, poses[pose_index]);
    Eigen::Vector4f centroid;
    compute3DCentroid(cloud, centroid);
    if(initialized) {
      translation += last_centroid - centroid;
    }
    initialized = true;
    KittiUtils::save_kitti_pose(
        Eigen::Translation3f(translation.head<3>())*poses[pose_index],
        cout);
    last_centroid = centroid;
    pose_index++;
  }
  return EXIT_SUCCESS;
}
