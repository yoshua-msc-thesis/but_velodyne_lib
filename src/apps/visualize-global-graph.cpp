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

#include <boost/algorithm/string.hpp>

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistration.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

class CorrespondenceVisualizer {
public:
  CorrespondenceVisualizer(const LineCloud &correspondences_) :
    correspondences(correspondences_) {
    visualizer.addPointCloud(correspondences.line_middles);
    visualizer.getViewer()->registerAreaPickingCallback(&CorrespondenceVisualizer::pickPointsCallback, *this);
  }

  void addPoses(const vector<Eigen::Affine3f> &poses) {
    for(int i = 0; i < poses.size(); i++) {
      visualizer.getViewer()->addCoordinateSystem(0.1, poses[i]);
    }
  }

  void addLandmarks(const PointCloud<PointXYZ> &landmarks) {
    visualizer.addPointCloud(landmarks);
  }

  void run() {
    vector<int> init_indices;
    for(int i = 0; i < correspondences.line_cloud.size(); i+=10) {
      init_indices.push_back(i);
    }
    if(init_indices.size() > 1000) {
      random_shuffle(init_indices.begin(), init_indices.end());
      init_indices.erase(init_indices.begin()+1000, init_indices.end());
    }
    setDataToVisualizer(init_indices);
    visualizer.show();
  }

protected:

  void setDataToVisualizer(const vector<int> &indices) {
    LineCloud vis_lines;
    for(vector<int>::const_iterator i = indices.begin(); i < indices.end(); i++) {
      vis_lines.push_back(correspondences.line_cloud[*i]);
    }
    cerr << "Adding " << vis_lines.line_cloud.size() << " lines" << endl;
    visualizer.addLines(vis_lines);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices)) {
      setDataToVisualizer(indices);
    }
  }

private:
  const LineCloud correspondences;
  Visualizer3D visualizer;
};

int main(int argc, char** argv) {

  if(argc != 2) {
    cerr << "ERROR, usage: " << argv[0] << " <global.graph>" << endl;
    return EXIT_FAILURE;
  }

  vector<Eigen::Affine3f> poses(1, Eigen::Affine3f::Identity());
  LineCloud correspondences;

  ifstream graph_file(argv[1]);
  std::string line;
  PointXYZ last_landmark;
  bool last_landmark_valid = false;
  PointCloud<PointXYZ> landmarks1, landmarks2;
  while (std::getline(graph_file, line)) {

    string edge_t;
    int from, to;
    stringstream line_stream(line);

    line_stream >> edge_t >> from >> to;

    if(edge_t == "EDGE3") {
      float tx, ty, tz, rx, ry, rz;
      line_stream >> tx >> ty >> tz >> rx >> ry >> rz;
      Eigen::Affine3f pose_to = poses[from] * getTransformation(tx, ty, tz, rx, ry, rz);
      if(poses.size() != to) {
        cerr << "ERROR: expected " << to << " poses. Found " << poses.size() << endl;
        return EXIT_FAILURE;
      }
      poses.push_back(pose_to);
    } else if(edge_t == "EDGE_SE3_XYZ") {
      int pose_idx = from;
      int line_idx = to - poses.size();

      PointXYZ landmark;
      line_stream >> landmark.x >> landmark.y >> landmark.z;
      landmark = transformPoint(landmark, poses[pose_idx]);
      if(pose_idx < poses.size()/2) {
        landmarks1.push_back(landmark);
      } else {
        landmarks2.push_back(landmark);
      }

      if(last_landmark_valid) {
        correspondences.push_back(PointCloudLine(last_landmark, landmark));
        last_landmark_valid = false;
      } else {
        last_landmark = landmark;
        last_landmark_valid = true;
      }
    }
  }

  cerr << "Have lines: " << correspondences.line_cloud.size() << endl;

  CorrespondenceVisualizer cvis(correspondences);
  //cvis.addPoses(poses);
  //cvis.addLandmarks(landmarks1);
  //cvis.addLandmarks(landmarks2);
  cvis.run();

  return EXIT_SUCCESS;
}
