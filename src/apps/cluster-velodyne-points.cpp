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
                     int &clusters_count) {
  string pose_filename, sensor_poses_filename, skip_filename;

  po::options_description desc("Velodyne Points Clustering\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensors calibration file.")
    ("clusters_count,c", po::value<int>(&clusters_count)->default_value(1000), "Clusters count.")
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

inline float sq(const float x) {
  return x*x;
}

void getPlaneCoefficients(const Eigen::Vector3f &normal, const Eigen::Vector3f pt,
    float &a, float &b, float &c, float &d) {
  /*a = normal(0);
  b = normal(1);
  c = normal(2);
  d = -normal.dot(pt);*/

  float n1 = normal(0);
  float n2 = normal(1);
  float n3 = normal(2);
  float x1 = pt(0);
  float x2 = pt(1);
  float x3 = pt(2);
  float nx1 = n1*x1;
  float nx2 = n2*x2;
  float nx3 = n3*x3;

  float k = sqrt(sq(n1) + sq(n2) + sq(n3) + sq(nx1) + sq(nx2) + sq(nx3) + 2*nx1*(nx2+nx3) + 2*nx2*nx3);

  a = n1/k;
  b = n2/k;
  c = n3/k;

  d = -(a*x1 + b*x2 + c*x3);
}

void clusterKMeans(const PointCloud<PointXYZI> &points, const vector<Eigen::Vector3f> &normals,
    const int K, vector<int> &indices) {
  cerr << "Clustering " << points.size() << " points and " << normals.size() << " normals into " << K << " clusters using k-means" << endl;

  cv::Mat data(points.size(), 4, CV_32F);
  for (int i = 0; i < points.size(); i++) {
    getPlaneCoefficients(normals[i], points[i].getVector3fMap(),
        data.at<float>(i, 0),
        data.at<float>(i, 1),
        data.at<float>(i, 2),
        data.at<float>(i, 3));
  }
  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::TermCriteria termination(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1);
  int attempts = 1;

  cerr << "Kmeans started" << endl;
  cv::kmeans(data, K, labels, termination, attempts, cv::KMEANS_PP_CENTERS);
  cerr << "Kmeans finished" << endl;

  labels.copyTo(indices);
}

void clusterEM(const PointCloud<PointXYZI> &points, const vector<Eigen::Vector3f> &normals,
    const int K, vector<int> &indices, vector<float> &clusterProbs) {
  cerr << "Clustering " << points.size() << " points and " << normals.size() << " normals into " << K << " GMMs" << endl;

  cv::Mat data(points.size(), 4, CV_32F);
  for (int i = 0; i < points.size(); i++) {
    getPlaneCoefficients(normals[i], points[i].getVector3fMap(),
        data.at<float>(i, 0),
        data.at<float>(i, 1),
        data.at<float>(i, 2),
        data.at<float>(i, 3));
  }

  Ptr<EM> em_model = EM::create();
  em_model->setClustersNumber(K);
  em_model->setCovarianceMatrixType(EM::COV_MAT_GENERIC);
  em_model->setTermCriteria(TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 1000, 0.1));

  cv::Mat labels(points.size(), 1, CV_32SC1);
  cv::Mat probs(points.size(), K, CV_64FC1);
  em_model->trainEM(data, noArray(), labels, probs);

  labels.copyTo(indices);

  clusterProbs.resize(indices.size());
  for(int i = 0; i < indices.size(); i++) {
    clusterProbs[i] = probs.at<double>(i, indices[i]);
  }
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
    const vector<int> &cluster_indices, const vector<float> &cluster_probs, PointCloud<PointXYZI> &sum_cloud) {
  pcl::search::KdTree<pcl::PointXYZI> index;
  index.setInputCloud(subsampled_cloud);
  vector<int> knn_indices(1);
  vector<float> distances(1);
  for(PointCloud<PointXYZI>::iterator query = sum_cloud.begin(); query < sum_cloud.end(); query++) {
    index.nearestKSearch(*query, 1, knn_indices, distances);
    int cluster = cluster_indices[knn_indices[0]];
    query->intensity = cluster;
    cout << cluster << " " << cluster_probs[knn_indices[0]] << endl;
  }
}

int main(int argc, char** argv) {

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  SensorsCalibration calibration;
  int clusters_count;

  if(!parse_arguments(argc, argv,
      poses, calibration, filenames,
      clusters_count)) {
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
  vector<float> probabilities;
  clusterEM(*subsampled_cloud, normals, clusters_count, indices, probabilities);

  colorByClusters(subsampled_cloud, indices, probabilities, sum_cloud);
  io::savePCDFileBinary("clusters.pcd", sum_cloud);

  return EXIT_SUCCESS;
}
