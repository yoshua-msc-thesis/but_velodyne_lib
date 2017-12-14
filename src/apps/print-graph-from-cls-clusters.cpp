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
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistration.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

cv::Mat POSES_COVARIANCE, LANDMARK_COVARIANCE;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     string &lines_fn, string &labels_origins_fn) {
  string pose_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("lines_file,l", po::value<string>(&lines_fn)->required(), "Lines encoded.")
    ("labels_origins,c", po::value<string>(&labels_origins_fn)->required(), "Cluster labels and origins of lines.")
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

/**
 * @param poses KITTI poses of frames
 * @param lineClusters clisters of generated lines
 * @param originClusters IDs of point clouds where lines originate
 */
void printPoseGraph(const vector<Eigen::Affine3f> &poses,
    const vector<LineCloud> &lineClusters, const vector< vector<int> > &originClusters,
    vector<LineCloud> &matchesToVis) {
  int edge_i;
  for(edge_i = 1; edge_i < poses.size(); edge_i++) {
    Eigen::Affine3f delta_pose = poses[edge_i-1].inverse() * poses[edge_i];
    cout << PoseGraphEdge(edge_i-1, edge_i, delta_pose.matrix(), POSES_COVARIANCE) << endl;
  }
  assert(lineClusters.size() == originClusters.size());
  for(int ci = 0; ci < lineClusters.size(); ci++) {
    const vector<PointCloudLine> &lines = lineClusters[ci].line_cloud;
    const vector<int> &origins = originClusters[ci];
    for(int i = 0; i+1 < lines.size(); i+=2) {
      Eigen::Vector3f pt1_vec, pt2_vec;
      lines[i].closestPointsWith(lines[i+1], pt1_vec, pt2_vec);
      PointXYZ pt1, pt2;
      pt1.getVector3fMap() = pt1_vec;
      pt2.getVector3fMap() = pt2_vec;
      matchesToVis[ci].push_back(PointCloudLine(pt1, pt2));
      pt1 = transformPoint(pt1, poses[origins[i]].inverse());
      pt2 = transformPoint(pt2, poses[origins[i+1]].inverse());
      cout << PoseToLandmarkGraphEdge(origins[i], edge_i, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << endl;
      cout << PoseToLandmarkGraphEdge(origins[i+1], edge_i, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << endl;
      edge_i++;
    }
  }
}

void saveClusters(const std::vector<LineCloud> &clustersLines,
    const std::vector< std::vector<int> > &clustersLinesOrigins) {
  ofstream clustersAndOrigins("clusters_origins.txt");
  LineCloud allLines;
  for(int c = 0; c < clustersLines.size(); c++) {
    allLines += clustersLines[c];
    for(vector<int>::const_iterator o = clustersLinesOrigins[c].begin(); o < clustersLinesOrigins[c].end(); o++) {
      clustersAndOrigins << c << " " << *o << endl;
    }
  }
  allLines.save("all_lines.pcd");
}

int main(int argc, char** argv) {

  POSES_COVARIANCE = cv::Mat::eye(6, 6, CV_32FC1)*0.1;
  LANDMARK_COVARIANCE = cv::Mat::eye(3, 3, CV_32FC1)*0.01;

  vector<Eigen::Affine3f> poses;
  string lines_fn, labels_origins_fn;

  if(!parse_arguments(argc, argv,
      poses, lines_fn, labels_origins_fn)) {
    return EXIT_FAILURE;
  }

  LineCloud all_lines;
  all_lines.load(lines_fn);

  vector<int> clusters(all_lines.line_cloud.size());
  vector<int> origins(all_lines.line_cloud.size());
  ifstream labels_origins_file(labels_origins_fn.c_str());
  int i = 0;
  int clusters_count = -1;
  for(int i = 0; i < clusters.size() && labels_origins_file.good(); i++) {
    labels_origins_file >> clusters[i] >> origins[i];
    clusters_count = MAX(clusters_count, clusters[i]+1);
  }

  vector<LineCloud> clustersLines(clusters_count);
  vector< vector<int> > clustersLinesOrigins(clusters_count);
  for(int i = 0; i < clusters.size() && labels_origins_file.good(); i++) {
    clustersLines[clusters[i]].push_back(all_lines.line_cloud[i]);
    clustersLinesOrigins[clusters[i]].push_back(origins[i]);
  }

  vector<LineCloud> matchesToVis(clusters_count);
  printPoseGraph(poses, clustersLines, clustersLinesOrigins, matchesToVis);

  Visualizer3D vis;
  for(int i = 0; i < clustersLines.size(); i++) {
    vis.addPointCloud(clustersLines[i].line_middles);
    /*Eigen::Vector4f centroidVec;
    compute3DCentroid(clustersLines[i].line_middles, centroidVec);
    PointXYZ centroidPt;
    centroidPt.getVector4fMap() = centroidVec;
    centroidPt.y -= 0.3;
    stringstream ss;
    ss << i;
    vis.getViewer()->addText3D(ss.str(), centroidPt, 0.1, 0.3, 0.3, 0.3, ss.str());*/
  }
  //vis.addPointCloud(clustersLines[764].line_middles);
  //vis.addLines(clustersLines[764].line_cloud);
  /*for(int i = 0; i < matchesToVis[764].line_cloud.size(); i++) {
    vis.addArrow(matchesToVis[764].line_cloud[i]);
  }*/
  vis.show();

  return EXIT_SUCCESS;
}
