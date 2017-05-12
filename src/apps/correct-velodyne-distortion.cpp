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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/Termination.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

const int SLICES_COUNT = 36;

void getSlices(const VelodynePointCloud &in_cloud, vector<VelodynePointCloud> &slices) {
  float polar_bin_size = 1.0 / slices.size();
  for(VelodynePointCloud::const_iterator pt = in_cloud.begin(); pt < in_cloud.end(); pt++) {
    int bin = floor(pt->phase/polar_bin_size);
    slices[bin].push_back(*pt);
  }
}

bool parse_arguments(
    int argc, char **argv,
    vector<Eigen::Affine3f> &poses,
    vector<string> &clouds_to_process,
    string &out_dir) {

  string pose_filename;

  po::options_description desc(
      "Correction of Velodyne point clouds distortion\n"
          "======================================\n"
          " * Reference(s): ???\n"
          " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("out_dir,o", po::value<string>(&out_dir)->required(), "Output directory for clouds.");

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
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
    return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);

  return true;
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<string> clouds_to_process;
  string out_dir;
  if(!parse_arguments(argc, argv, poses, clouds_to_process, out_dir)) {
    return EXIT_FAILURE;
  }

//  Visualizer3D vis;
  int output_count = MIN(clouds_to_process.size(), poses.size()) - 1;
  for(int cloud_i = 0; cloud_i < output_count; cloud_i++) {
    VelodynePointCloud in_cloud, out_cloud;
    VelodynePointCloud::fromFile(clouds_to_process[cloud_i], in_cloud, false);

    Eigen::Affine3f delta = poses[cloud_i].inverse()*poses[cloud_i+1];

    vector<VelodynePointCloud> slices(SLICES_COUNT);
    getSlices(in_cloud, slices);

    /*
    vis.keepOnlyClouds(0);
    for(int i = 0; i < slices.size(); i++) {
      vis.setColor(0, 100, i*(255.0/SLICES_COUNT)).addPointCloud(slices[i]);
    }
    vis.show();
    */

    vector<float> dof(6);
    getTranslationAndEulerAngles(delta, dof[0], dof[1], dof[2], dof[3], dof[4], dof[5]);
    for(int i = 0; i < slices.size(); i++) {
      Eigen::Affine3f t = getTransformation(dof[0]/SLICES_COUNT*i, dof[1]/SLICES_COUNT*i, dof[2]/SLICES_COUNT*i,
          dof[3]/SLICES_COUNT*i, dof[4]/SLICES_COUNT*i, dof[5]/SLICES_COUNT*i);
      transformPointCloud(slices[i], slices[i], t);
//      vis.setColor(i*(255.0/SLICES_COUNT), 100, 0).addPointCloud(slices[i]);
      out_cloud += slices[i];
    }
//    vis.show();

    transformPointCloud(out_cloud, out_cloud, in_cloud.getAxisCorrection().inverse());
    io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(cloud_i, ".pcd"), out_cloud);
  }

  return EXIT_SUCCESS;
}
