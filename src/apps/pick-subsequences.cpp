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

  PointCloud<PointXYZ> poses_points = Visualizer3D::posesToPoints(poses);

  SubseqPicker picker(poses_points);

  StartEnd s1, s2;
  picker.run(s1, s2);

  cout << s1.start << " " << s1.end << endl;
  cout << s2.start << " " << s2.end << endl;

  return EXIT_SUCCESS;
}
