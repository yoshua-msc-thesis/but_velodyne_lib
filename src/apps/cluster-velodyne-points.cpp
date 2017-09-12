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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process,
                     int &avg_cluster_size) {
  string pose_filename, sensor_poses_filename, skip_filename;

  po::options_description desc("Velodyne Points Clustering\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("cluster_size,c", po::value<int>(&avg_cluster_size)->default_value(1000), "Average cluster size.")
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

void cluster(const PointCloud<PointXYZI> &points, const vector<Eigen::Vector3f> &normals,
    const int K, vector<int> &indices) {
  cerr << "Clustering " << points.size() << " points and " << normals.size() << " normals into " << K << " clusters" << endl;

  cv::Mat data(points.size(), 6, CV_32F);
  for (int i = 0; i < points.size(); i++) {
    data.at<float>(i, 0) = points[i].x;
    data.at<float>(i, 1) = points[i].y;
    data.at<float>(i, 2) = points[i].z;
    data.at<float>(i, 3) = normals[i](0);
    data.at<float>(i, 4) = normals[i](1);
    data.at<float>(i, 5) = normals[i](2);
  }
  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1);
  int attempts = 1;

  cerr << "Kmeans started" << endl;
  cv::kmeans(data, K, labels, termination, attempts, cv::KMEANS_PP_CENTERS);
  cerr << "Kmeans finished" << endl;

  labels.copyTo(indices);
}

void getNormals(const PointCloud<PointXYZI> &middles,
    const PointCloud<PointXYZI> &original_points,
    vector<int> origins, PointCloud<PointXYZ> sensor_positions,
    vector<Eigen::Vector3f> &normals) {
  cerr << "Normal estimation" << endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(middles.makeShared());
  ne.setSearchSurface(original_points.makeShared());

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);

  ne.setRadiusSearch (0.1);

  pcl::PointCloud<pcl::Normal> cloud_normals;
  ne.compute (cloud_normals);

  for(int i = 0; i < middles.size(); i++) {
    Eigen::Vector4f n = cloud_normals[i].getNormalVector4fMap();
    PointXYZ p = sensor_positions[origins[i]];
    flipNormalTowardsViewpoint(middles[i], p.x, p.y, p.z, n);
    normals.push_back(n.head(3));
  }
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

void colorByClusters(const PointCloud<PointXYZI>::Ptr subsampled_cloud,
    const vector<int> &cluster_indices, PointCloud<PointXYZI> &sum_cloud) {
  pcl::search::KdTree<pcl::PointXYZI> index;
  index.setInputCloud(subsampled_cloud);
  vector<int> knn_indices(1);
  vector<float> distances(1);
  for(PointCloud<PointXYZI>::iterator query = sum_cloud.begin(); query < sum_cloud.end(); query++) {
    index.nearestKSearch(*query, 1, knn_indices, distances);
    int cluster = cluster_indices[knn_indices[0]];
    query->intensity = cluster;
    cout << cluster << endl;
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int avg_cluster_size;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames,
      avg_cluster_size)) {
    return EXIT_FAILURE;
  }

  VelodyneFileSequence sequence(filenames, calibration);

  vector<int> subsampled_origins;
  vector<Eigen::Vector3f> normals;

  PointCloud<PointXYZI> sum_cloud;
  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  for(int frame_i = 0; sequence.hasNext(); frame_i++) {
    VelodyneMultiFrame multiframe = sequence.getNext();
    PointCloud<PointXYZI>::Ptr joined(new PointCloud<PointXYZI>);
    multiframe.joinTo(*joined);
    transformPointCloud(*joined, *joined, poses[frame_i]);
    sum_cloud += *joined;
    subsample_cloud<PointXYZI>(joined, 0.08);
    *subsampled_cloud += *joined;
    vector<int> origins(joined->size(), frame_i);
    subsampled_origins.insert(subsampled_origins.end(), origins.begin(), origins.end());
  }

  getNormals(*subsampled_cloud, sum_cloud,
      subsampled_origins, Visualizer3D::posesToPoints(poses),
      normals);

  vector<int>::iterator origins_it = subsampled_origins.begin();
  PointCloud<PointXYZI>::iterator points_it = subsampled_cloud->begin();
  for(vector<Eigen::Vector3f>::iterator normals_it = normals.begin(); normals_it < normals.end();) {
    Eigen::Vector3f n = *normals_it;
    if(isfinite(n(0)) && isfinite(n(1)) && isfinite(n(2))) {
      normals_it++;
      origins_it++;
      points_it++;
    } else {
      normals_it = normals.erase(normals_it);
      points_it = subsampled_cloud->erase(points_it);
      origins_it = subsampled_origins.erase(origins_it);
    }
  }

  vector<int> indices;
  int K = subsampled_cloud->size()/avg_cluster_size;
  cluster(*subsampled_cloud, normals, K, indices);

  colorByClusters(subsampled_cloud, indices, sum_cloud);
  io::savePCDFileBinary("clusters.pcd", sum_cloud);

  return EXIT_SUCCESS;
}
