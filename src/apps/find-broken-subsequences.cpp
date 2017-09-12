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
#include <but_velodyne/MoveEstimation.h>

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
                     vector<Eigen::Affine3f> &poses) {
  string pose_filename;

  po::options_description desc("Picking subseqences to close loop\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
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
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  poses = KittiUtils::load_kitti_poses(pose_filename);

  return true;
}

class StartEnd {
public:
  int start, end, idx;
  StartEnd(void) :
    start(1000000000), end(-1), idx(0) {
  }
  bool operator <(const StartEnd &other) const {
    return this->start < other.start;
  }
  bool valid() const {
    return start < end;
  }
};

class SubseqPicker {
public:
  SubseqPicker(const PointCloud<PointXYZ> &poses_) :
    poses(poses_) {
    vis.getViewer()->registerAreaPickingCallback(&SubseqPicker::pickPointsCallback, *this);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      sort(indices.begin(), indices.end());
      int start = indices.front();
      int end = indices.back();
      cerr << start << " - " << end << endl;
      if(seq1.valid() == seq2.valid()) {
        seq1.start = start;
        seq1.end = end;
      } else {
        seq2.start = start;
        seq2.end = end;
      }
      setDataToVisualizer();
    }
  }

  void run(StartEnd &seq1_, StartEnd &seq2_) {
    setDataToVisualizer();
    vis.show();
    seq1_ = seq1;
    seq2_ = seq2;
  }

protected:
  void setDataToVisualizer() {
    PointCloud<PointXYZRGB>::Ptr colored_poses = Visualizer3D::colorizeCloud(poses, 200, 200, 200);
    if(seq1.valid()) {
      for(int i = seq1.start; i <= seq1.end; i++) {
        colored_poses->at(i).r = 255;
        colored_poses->at(i).b = colored_poses->at(i).g = 100;
      }
    }
    if(seq2.valid()) {
      for(int i = seq2.start; i <= seq2.end; i++) {
        colored_poses->at(i).b = 255;
        colored_poses->at(i).r = colored_poses->at(i).g = 100;
      }
    }
    vis.keepOnlyClouds(0).addColorPointCloud(colored_poses);
  }

private:
  const PointCloud<PointXYZ> poses;
  Visualizer3D vis;
  StartEnd seq1, seq2;
};

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  if(!parse_arguments(argc, argv,
      poses)) {
    return EXIT_FAILURE;
  }

  int history_size = 3;
  float threshold = 1.5;

  LinearMoveEstimator estimator(history_size);

  PointCloud<PointXYZI> poses_error;
  poses_error.resize(poses.size());
  PointCloud<PointXYZ> poses_points = Visualizer3D::posesToPoints(poses);

  for(int i = 0; i < poses.size(); i++) {
    Eigen::Affine3f delta;
    if(i == 0) {
      delta = poses[0].inverse() * poses[1];
    } else {
      delta = poses[i-1].inverse() * poses[i];
    }
    MoveParameters current(delta);
    float error = 0.0;
    if(i > history_size) {
      MoveParameters estimation = estimator.predict();
      float current_speed = sqrt(current.x*current.x + current.y*current.y + current.z*current.z);
      float estimated_speed = sqrt(estimation.x*estimation.x + estimation.y*estimation.y + estimation.z*estimation.z);
      error = MAX(current_speed, estimated_speed) / MIN(current_speed, estimated_speed);
    }

    cerr << error << endl;
    copyXYZ(poses_points[i], poses_error[i]);
    poses_error[i].intensity = error;

    estimator.addMeassurement(current);
  }

  PointCloud<PointXYZRGB>::Ptr poses_errors_vis(new PointCloud<PointXYZRGB>);
  poses_errors_vis->resize(poses.size());
  for(int i = 0; i < poses_error.size(); i++) {
    PointXYZRGB &colored_pt = poses_errors_vis->at(i);
    copyXYZ(poses_error[i], colored_pt);
    colored_pt.r = colored_pt.g = colored_pt.b = 0;
    if(poses_error[i].intensity > threshold) {
      colored_pt.r = 255;
    } else {
      colored_pt.b = 255;
    }
  }
  Visualizer3D().addColorPointCloud(poses_errors_vis).show();

  return EXIT_SUCCESS;
}
