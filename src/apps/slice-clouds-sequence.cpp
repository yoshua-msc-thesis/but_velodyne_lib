/*
 * Odometry estimation by collar line segments of Velodyne scan.
 *
 * Published in:
 *  Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 *  Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
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

bool parse_arguments(int argc, char **argv,
                     vector<string> &clouds_to_process, vector<Eigen::Affine3f> &input_poses,
                     string &out_filename) {
  string pose_file;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ???\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_file)->required(), "Input pose file")
      ("out_cloud,o", po::value<string>(&out_filename)->required(), "Output cloud filename")
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

  input_poses = KittiUtils::load_kitti_poses(pose_file);

  return true;
}

class PosesSelector {

public:
  PosesSelector(const vector<Eigen::Affine3f> &poses_) :
    poses(Visualizer3D::posesToPoints(poses_)) {
    visualizer.getViewer()->registerAreaPickingCallback(&PosesSelector::pickPointsCallback, *this);
    addDataToVisualizer();
  }

  const vector<int>& getIndices() const {
    return selected_indices;
  }

protected:
  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    event.getPointsIndices(selected_indices);
    addDataToVisualizer();
  }

  void addDataToVisualizer() {
    PointCloud<PointXYZRGB>::Ptr colored_poses(new PointCloud<PointXYZRGB>);
    colored_poses->resize(poses.size());
    for(int i = 0; i < poses.size(); i++) {
      copyXYZ(poses[i], colored_poses->at(i));
      colored_poses->at(i).r = colored_poses->at(i).g = 0;
      colored_poses->at(i).b = 255;
    }
    for(vector<int>::iterator i = selected_indices.begin(); i < selected_indices.end(); i++) {
      colored_poses->at(*i).g = colored_poses->at(*i).b = 0;
      colored_poses->at(*i).r = 255;
    }
    visualizer
      .keepOnlyClouds(0)
      .addColorPointCloud(colored_poses)
      .show();
  }

private:
  Visualizer3D visualizer;
  PointCloud<PointXYZ> poses;
  vector<int> selected_indices;
};

int main(int argc, char** argv) {
  vector<string> clouds_to_process;
  vector<Eigen::Affine3f> poses;
  string out_filename;

  if(!parse_arguments(argc, argv, clouds_to_process, poses, out_filename)) {
    return EXIT_FAILURE;
  }

  ofstream out_clouds_list((out_filename + ".clouds.list").c_str());
  ofstream out_poses_file((out_filename + ".poses.txt").c_str());

  PosesSelector selector(poses);
  vector<int> indices = selector.getIndices();

  VelodynePointCloud output_cloud;
  for (vector<int>::iterator i = indices.begin(); i < indices.end(); i++) {
    if(*i < clouds_to_process.size()) {
      VelodynePointCloud in_cloud;
      VelodynePointCloud::fromFile(clouds_to_process[*i], in_cloud, false);
      transformPointCloud(in_cloud, in_cloud, poses[*i]);
      output_cloud += in_cloud;

      out_clouds_list << clouds_to_process[*i] << endl;
      KittiUtils::printPose(out_poses_file, poses[*i].matrix());
    }
  }

  cerr << "Saving " << out_filename << std::endl;
  io::savePCDFileBinary(out_filename, output_cloud);

  return EXIT_SUCCESS;
}
