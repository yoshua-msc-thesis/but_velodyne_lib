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
namespace po = boost::program_options;

cv::Mat POSE_LANDMARK_COVARIANCE, POSES_COVARIANCE;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     vector<string> &clouds_to_process,
                     int &avg_cluster_size,
                     int &lines_generated,
                     int &lines_preserved) {
  string pose_filename, skip_filename;

  po::options_description desc("Collar Lines Clustering\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
      ("cluster_size,c", po::value<int>(&avg_cluster_size)->default_value(1000), "Average cluster size.")
      ("lines_generated,g", po::value<int>(&lines_generated)->default_value(5), "lines_generated")
      ("lines_preserved,k", po::value<int>(&lines_preserved)->default_value(1), "lines_preserved (kept)")
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
  return true;
}

void cluster(const PointCloud<PointXYZ> &points, const int K, vector<int> &indices) {
  cv::Mat data(points.size(), 3, CV_32F);
  for (int i = 0; i < points.size(); i++) {
    data.at<float>(i, 0) = points[i].x;
    data.at<float>(i, 1) = points[i].y;
    data.at<float>(i, 2) = points[i].z;
  }
  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1);
  int attempts = 1;
  cv::kmeans(data, K, labels, termination, attempts, cv::KMEANS_PP_CENTERS);
  labels.copyTo(indices);
}

void buildLineCloud(const vector<string> &cloud_files,
    const vector<Eigen::Affine3f> &poses,
    int lines_per_cell_gen, int lines_per_cell_preserve,
    LineCloud &out_lines) {
  CollarLinesFilter filter(lines_per_cell_preserve);
  for(int i = 0; i < cloud_files.size(); i++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(cloud_files[i], cloud, false);
    PolarGridOfClouds polar_grid(cloud);
    LineCloud line_cloud(polar_grid, lines_per_cell_gen, filter);
    line_cloud.transform(poses[i].matrix());
    out_lines += line_cloud;
  }
}

void getTestLines(vector<Eigen::Affine3f> &poses, LineCloud &lines,
    vector<int> &origin, vector<int> &indices, int &K) {
  poses.clear();
  poses.push_back(Eigen::Affine3f::Identity());
  Eigen::Affine3f transformation = getTransformation(0.48, 0.25, 0.55, 0.25, 0.35, 0.18);
  poses.push_back(transformation);
  transformation = getTransformation(0.5, 0.3, 0.6, 0.3, 0.4, 0.1);

  lines.push_back(PointCloudLine(Eigen::Vector3f(1, 1, 0), Eigen::Vector3f(2, 1, 0)));
  lines.push_back(PointCloudLine(Eigen::Vector3f(0, 1, 1), Eigen::Vector3f(0, 1, 2)));
  lines.push_back(PointCloudLine(Eigen::Vector3f(1, 0, 3), Eigen::Vector3f(1, 0, 1.5)));
  LineCloud cloud2;
  lines.transform(transformation.matrix(), cloud2);
  lines += cloud2;

  origin.resize(6);
  origin[0] = origin[1] = origin[2] = 0;
  origin[3] = origin[4] = origin[5] = 1;

  K = 3;
  indices.resize(6);
  indices[0] = indices[3] = 0;
  indices[1] = indices[4] = 1;
  indices[2] = indices[5] = 2;
}

void getClusters(const LineCloud &lines, const vector<int> &indices, const int K, const vector<int> &origins,
    vector<LineCloud> &clustersLines, vector< PointCloud<PointXYZ> > &clustersMiddles, vector< vector<int> > &clustersOrigins) {
  clustersLines.resize(K);
  clustersMiddles.resize(K);
  clustersOrigins.resize(K);
  vector<int> shuffled(indices.size());
  for(int i = 0; i < indices.size(); i++) {
    shuffled[i] = i;
  }
  random_shuffle(shuffled.begin(), shuffled.end());
  for(vector<int>::iterator i = shuffled.begin(); i < shuffled.end(); i++) {
    clustersLines[indices[*i]].push_back(lines.line_cloud[*i]);
    clustersMiddles[indices[*i]].push_back(lines.line_middles[*i]);
    clustersOrigins[indices[*i]].push_back(origins[*i]);
  }
}

class PoseToLandmarkGraphEdge {
public:
  PoseToLandmarkGraphEdge(int sourceIdx_, int targetIdx_,
      float dx_, float dy_, float dz_, cv::Mat covariance_) :
        sourceIdx(sourceIdx_), targetIdx(targetIdx_), dx(dx_), dy(dy_), dz(dz_),
        covariance(covariance_) {
  }

  const int sourceIdx, targetIdx;
  const float dx, dy, dz;
  const cv::Mat covariance;
};

std::ostream& operator<<(std::ostream &stream, const PoseToLandmarkGraphEdge &edge) {
  stream << "EDGE_SE3_XYZ" << " " <<
      edge.sourceIdx << " " << edge.targetIdx << " " <<
      edge.dx << " " << edge.dy << " " << edge.dz;

  Eigen::MatrixXf eigenCovariance;
  cv2eigen(edge.covariance, eigenCovariance);

  Eigen::MatrixXf correction = Eigen::MatrixXf::Identity(edge.covariance.rows, edge.covariance.cols) * PoseGraphEdge::CORRECTION;
  Eigen::MatrixXf precision = (eigenCovariance + correction).inverse();

  for(int row = 0; row < edge.covariance.rows; row++) {
    for(int col = 0; col < edge.covariance.cols; col++) {
      if(row <= col) {
        stream << " " << precision(row, col);
      }
    }
  }
  return stream;
}

LineCloud matches;

void printPoseGraph(const vector<Eigen::Affine3f> &poses,
    const vector<LineCloud> &lineClusters, const vector< vector<int> > &originClusters) {
  vector<PoseToLandmarkGraphEdge> output;
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
      matches.push_back(PointCloudLine(pt1, pt2));
      pt1 = transformPoint(pt1, poses[origins[i]].inverse());
      pt2 = transformPoint(pt2, poses[origins[i+1]].inverse());
      cout << PoseToLandmarkGraphEdge(origins[i], edge_i, pt1.x, pt1.y, pt1.z, POSE_LANDMARK_COVARIANCE) << endl;
      cout << PoseToLandmarkGraphEdge(origins[i+1], edge_i, pt2.x, pt2.y, pt2.z, POSE_LANDMARK_COVARIANCE) << endl;
      edge_i++;
    }
  }
}

int main(int argc, char** argv) {

  MoveParameters::getDummyCovariance(0.01, 0.02, POSES_COVARIANCE);
  POSE_LANDMARK_COVARIANCE = POSES_COVARIANCE(cv::Rect(0, 0, 3, 3));

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  int avg_cluster_size, lines_generated, lines_preserved;

  /*
  if(!parse_arguments(argc, argv,
      poses, filenames,
      avg_cluster_size,
      lines_generated, lines_preserved)) {
    return EXIT_FAILURE;
  }

  LineCloud lines;
  buildLineCloud(filenames, poses, lines_generated, lines_preserved, lines, origin);

  vector<int> indices;
  int K = lines.line_middles.size()/avg_cluster_size;
  cluster(lines.line_middles, K, indices);
  */

  /* TEST */
  LineCloud lines;
  vector<int> indices, origins;
  int K;
  getTestLines(poses, lines, origins, indices, K);
  /* END TEST */

  vector< PointCloud<PointXYZ> > clustersMiddles;
  vector<LineCloud> clustersLines;
  vector< vector<int> > clustersOrigins;
  getClusters(lines, indices, K, origins, clustersLines, clustersMiddles, clustersOrigins);

  printPoseGraph(poses, clustersLines, clustersOrigins);

  Visualizer3D vis;
  vis.addLines(lines);
  for(int i = 0; i < matches.line_cloud.size(); i++) {
    vis.addArrow(matches.line_cloud[i]);
  }
  vis.show();

  return EXIT_SUCCESS;
}
