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
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/GlobalOptimization.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ml/kmeans.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

template <typename T>
void load_vector_from_file(const string &filename, vector<T> &output) {
  ifstream file(filename.c_str());
  T element;
  while(file >> element) {
    output.push_back(element);
  }
}

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<int> &cluster_labels,
                     vector<string> &clouds_to_process) {
  string pose_filename, cluster_labels_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("cluster_labels,c", po::value<string>(&cluster_labels_filename)->required(), "Cluster labels of points.")
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
  load_vector_from_file(cluster_labels_filename, cluster_labels);

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

template <typename PointT>
void printPoseGraph(const std::vector<Eigen::Affine3f> &poses,
    const SensorsCalibration &calibration,
    const std::vector< pcl::PointCloud<PointT> > &points_clusters,
    const std::vector< std::vector<Origin> > &origins_clusters,
    const std::vector< std::vector<cv::DMatch> > &matches_clusters) {

  static const cv::Mat POSES_COVARIANCE = cv::Mat::eye(6, 6, CV_32FC1)*0.1;
  static const cv::Mat LANDMARK_COVARIANCE = cv::Mat::eye(3, 3, CV_32FC1)*0.1;
  static const int SENSORS = 2;

  for(int pi = 1; pi < poses.size(); pi++) {
    Eigen::Affine3f delta_pose = poses[pi-1].inverse() * poses[pi];
    std::cout << PoseGraphEdge(pi-1, pi, delta_pose.matrix(), POSES_COVARIANCE) << std::endl;
  }
  for(int si = 1; si < SENSORS; si++) {
    for(int pi = 0; pi < poses.size(); pi++) {
      std::cout << PoseGraphEdge(pi, pi+si*poses.size(), calibration.ofSensor(si).matrix(), POSES_COVARIANCE) << std::endl;
    }
  }
  int new_vertex = poses.size()*calibration.sensorsCount();
  for(int ci = 0; ci < points_clusters.size(); ci++) {
    const pcl::PointCloud<PointT> &points = points_clusters[ci];
    const std::vector<Origin> &origins = origins_clusters[ci];
    const std::vector<cv::DMatch> &matches = matches_clusters[ci];
    for(std::vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
      Origin o1 = origins[m->trainIdx];
      Origin o2 = origins[m->queryIdx];
      PointT pt1 = transformPoint(points[m->trainIdx], (poses[o1.pose_id]*calibration.ofSensor(o1.sensor_id)).inverse());
      PointT pt2 = transformPoint(points[m->queryIdx], (poses[o2.pose_id]*calibration.ofSensor(o2.sensor_id)).inverse());
      std::cout << PoseToLandmarkGraphEdge(o1.edgeIdx(poses.size()), new_vertex, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << std::endl;
      std::cout << PoseToLandmarkGraphEdge(o2.edgeIdx(poses.size()), new_vertex, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << std::endl;
      new_vertex++;
    }
  }
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<int> cluster_labels;
  vector<string> pcd_files;
  SensorsCalibration calibration;
  if(!parse_arguments(argc, argv,
      poses, calibration, cluster_labels, pcd_files)) {
    return EXIT_FAILURE;
  }

  int clusters_count = *max_element(cluster_labels.begin(), cluster_labels.end()) + 1;

  vector< PointCloud<PointXYZI> > points_clusters(clusters_count);
  vector< vector<Origin> > origins_clusters(clusters_count);
  VelodyneFileSequence file_sequence(pcd_files, calibration);
  int past_frames_points = 0;
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    if(frame_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << frame_i << std::endl << std::flush;
      break;
    }
    PointCloud<PointXYZI> cloud;
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.joinTo(cloud);
    transformPointCloud(cloud, cloud, poses[frame_i]);

    int sensor_i = 0;
    int past_sensors_points = 0;
    for(int i = 0; i < cloud.size(); i++) {
      if(multiframe.clouds[sensor_i]->size() <= i-past_sensors_points) {
        past_sensors_points += multiframe.clouds[sensor_i]->size();
        sensor_i++;
      }
      int label = cluster_labels[past_frames_points+i];
      points_clusters[label].push_back(cloud[i]);
      origins_clusters[label].push_back(Origin(frame_i, sensor_i));
    }
    past_frames_points += cloud.size();
  }

  /*Visualizer3D vis;
  for(vector< PointCloud<PointXYZI> >::iterator cluster = points_clusters.begin(); cluster < points_clusters.end(); cluster++) {
    PointCloud<PointXYZI>::Ptr sub_cluster(new PointCloud<PointXYZI>);
    *sub_cluster += *cluster;
    subsample_cloud<PointXYZI>(sub_cluster, 0.1);
    vis.addPointCloud(*sub_cluster);
  }
  vis.show();*/

  vector< vector<cv::DMatch> > matches_clusters(clusters_count);
  for(int ci = 0; ci < points_clusters.size(); ci++) {
    const PointCloud<PointXYZI> &cluster = points_clusters[ci];

    Eigen::Vector3f centroid;
    Eigen::Matrix3f covaraince;
    computeCentroidAndCovariance(cluster, centroid, covaraince);

    vector<float> ev;
    getEigenvalues(covaraince, ev);

    if(ev[1] > 2*ev[0] && ev[2] < 2*ev[1]) {
      PointCloud<PointXYZI>::Ptr projectedCluster(new PointCloud<PointXYZI>);
      reduceWeakestDimension(cluster, covaraince, *projectedCluster);
      getClosestMatches<PointXYZI>(projectedCluster, 0.01, matches_clusters[ci]);
    }
  }

  printPoseGraph(poses, calibration, points_clusters, origins_clusters, matches_clusters);

  return EXIT_SUCCESS;
}
