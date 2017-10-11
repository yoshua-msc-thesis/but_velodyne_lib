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
#include <but_velodyne/common.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

bool DONE = false;
bool IGNORE = false;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     float &inlier_tolerance, float &normal_dist_weight,
                     string &normals_filename, string &indices_filename,
                     vector<string> &clouds_to_process) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("normals_filename,n", po::value<string>(&normals_filename)->required(), "Normals filename.")
    ("indices_filename,i", po::value<string>(&indices_filename)->required(), "Indices filename.")
    ("inlier_tolerance,t", po::value<float>(&inlier_tolerance)->default_value(0.2), "Inlier tolerance.")
    ("normal_dist_weight,w", po::value<float>(&normal_dist_weight)->default_value(0.5), "Weight of normals angular distance.")
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

  if(!sensor_poses_filename.empty()) {
    calibration = SensorsCalibration(sensor_poses_filename);
  } else {
    calibration = SensorsCalibration();
  }

  return true;
}

void printPoseGraphPrefix(const std::vector<Eigen::Affine3f> &poses) {
  static const cv::Mat POSES_COVARIANCE = cv::Mat::eye(6, 6, CV_32FC1)*0.1;
  for(int pi = 1; pi < poses.size(); pi++) {
    Eigen::Affine3f delta_pose = poses[pi-1].inverse() * poses[pi];
    std::cout << PoseGraphEdge(pi-1, pi, delta_pose.matrix(), POSES_COVARIANCE) << std::endl;
  }
}

template <typename PointT>
void printPoseGraphMatches(const std::vector<Eigen::Affine3f> &poses,
    const pcl::PointCloud<PointT> &points,
    const std::vector<Origin> &origins,
    const std::vector<cv::DMatch> &matches) {

  static const cv::Mat LANDMARK_COVARIANCE = cv::Mat::eye(3, 3, CV_32FC1)*0.001;

  static int new_vertex = poses.size();
  for(std::vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
    Origin o1 = origins[m->trainIdx];
    Origin o2 = origins[m->queryIdx];
    PointT pt1 = transformPoint(points[m->trainIdx], poses[o1.pose_id].inverse());
    PointT pt2 = transformPoint(points[m->queryIdx], poses[o2.pose_id].inverse());
    std::cout << PoseToLandmarkGraphEdge(o1.pose_id, new_vertex, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << std::endl;
    std::cout << PoseToLandmarkGraphEdge(o2.pose_id, new_vertex, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << std::endl;
    new_vertex++;
  }
}

void spreadInliers(const vector< vector<int> > &impact,
    PointIndices::ConstPtr subsampled_inliers, PointIndices::Ptr sum_inliers) {
  for(vector<int>::const_iterator subsampled_inlier = subsampled_inliers->indices.begin();
      subsampled_inlier < subsampled_inliers->indices.begin();
      subsampled_inlier++) {
    for(vector<int>::const_iterator sum_inlier = impact[*subsampled_inlier].begin();
        sum_inlier < impact[*subsampled_inlier].end();
        sum_inlier++) {
      sum_inliers->indices.push_back(*sum_inlier);
    }
  }
}

void processInliers(PointCloud<PointXYZI>::Ptr subsampled_cloud,
    PointCloud<PointXYZI>::ConstPtr sum_cloud,
    vector< vector<int> > &impact,
    PointIndices::ConstPtr inliers,
    PointCloud<PointXYZI>::Ptr plane) {
  vector<bool> delete_mask(subsampled_cloud->size(), false);
  for(vector<int>::const_iterator i = inliers->indices.begin(); i < inliers->indices.end(); i++) {
    delete_mask[*i] = true;
    for(vector<int>::const_iterator sum_i = impact[*i].begin(); sum_i < impact[*i].end(); sum_i++) {
      plane->push_back(sum_cloud->at(*sum_i));
    }
  }

  PointCloud<PointXYZI>::iterator subsampled_it = subsampled_cloud->begin();
  vector< vector<int> >::iterator impact_it = impact.begin();
  for(vector<bool>::iterator d = delete_mask.begin(); d < delete_mask.end(); d++) {
    if(*d) {
      subsampled_it = subsampled_cloud->erase(subsampled_it);
      impact_it = impact.erase(impact_it);
    } else {
      subsampled_it++;
      impact_it++;
    }
  }
}

void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
  if(event.keyDown()) {
    if(event.getKeySym() == "d") {
      DONE = true;
    } else if(event.getKeySym() == "i") {
      IGNORE = true;
    }

  }
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<string> pcd_files;
  SensorsCalibration calibration;
  string normals_filename, indices_filename;
  float inlier_tolerance, normal_dist_weight;
  if(!parse_arguments(argc, argv,
      poses, calibration, inlier_tolerance, normal_dist_weight,
      normals_filename, indices_filename,
      pcd_files)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZI>::Ptr sum_cloud(new PointCloud<PointXYZI>);
  vector<Origin> sum_origins;
  VelodyneFileSequence file_sequence(pcd_files, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {

    PointCloud<PointXYZI> cloud;
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.joinTo(cloud);
    transformPointCloud(cloud, cloud, poses[frame_i]);
    *sum_cloud += cloud;

    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      vector<Origin> origins(multiframe.clouds[sensor_i]->size(), Origin(frame_i, sensor_i));
      sum_origins.insert(sum_origins.end(), origins.begin(), origins.end());
    }
  }

  cerr << "Loading normals ..." << endl;
  PointCloud<Normal>::Ptr subsampled_normals(new PointCloud<Normal>);
  io::loadPCDFile(normals_filename, *subsampled_normals);

  cerr << "Loading indices & subsampling ..." << endl;
  PointIndices::Ptr indices(new PointIndices);
  load_vector_from_file(indices_filename, indices->indices);
  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  extract_indices(sum_cloud, indices, *subsampled_cloud);
  vector<Origin> subsampled_origins;
  extract_indices(sum_origins, indices->indices, subsampled_origins);

  /*vis.keepOnlyClouds(0).setColor(150, 150, 150).addPointCloud(*subsampled_cloud);
  for(int i = 0; i < subsampled_cloud->size(); i+=1000) {
    Eigen::Vector3f start = subsampled_cloud->at(i).getVector3fMap();
    Eigen::Vector3f orient = subsampled_normals->at(i).getNormalVector3fMap();
    vis.addArrow(PointCloudLine(start, orient));
  }
  vis.show();*/

  /*vector< vector<int> > impact(subsampled_cloud->size());
  KdTreeFLANN<PointXYZI> subsampled_index;
  subsampled_index.setInputCloud(subsampled_cloud);
  for(int i = 0; i < sum_cloud->size(); i++) {
    vector<int> knn_indices(1);
    vector<float> distances(1);
    subsampled_index.nearestKSearch(sum_cloud->at(i), 1, knn_indices, distances);
    impact[knn_indices.front()].push_back(i);
  }*/

  SACSegmentationFromNormals<PointXYZI, Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_NORMAL_PLANE);
  seg.setMethodType(SAC_PROSAC);
  seg.setDistanceThreshold(inlier_tolerance);
  seg.setNormalDistanceWeight(normal_dist_weight);

  Visualizer3D vis;
  vis.getViewer()->registerKeyboardCallback(keyCallback);
  while(!DONE) {
    seg.setInputCloud(subsampled_cloud);
    seg.setInputNormals(subsampled_normals);
    PointIndices::Ptr subsampled_inliers(new PointIndices);
    ModelCoefficients coefficients;
    seg.segment(*subsampled_inliers, coefficients);

    PointCloud<PointXYZI>::Ptr plane(new PointCloud<PointXYZI>);
    extract_indices(subsampled_cloud, subsampled_inliers, *plane);
    vis.keepOnlyClouds(0).setColor(150, 150, 150).addPointCloud(*subsampled_cloud);
    vis.setColor(50, 200, 200).addPointCloud(*plane).show();
    extract_indices(subsampled_cloud, subsampled_inliers, *subsampled_cloud, false);
    extract_indices(subsampled_origins, indices->indices, subsampled_origins, false);

    if(!IGNORE) {
      Eigen::Vector3f centroid;
      Eigen::Matrix3f covaraince;
      computeCentroidAndCovariance(*plane, centroid, covaraince);

      PointCloud<PointXYZI>::Ptr projectedPlane(new PointCloud<PointXYZI>);
      reduceWeakestDimension(*plane, covaraince, *projectedPlane);

      vector<cv::DMatch> matches;
      getClosestMatches<PointXYZI>(projectedPlane, 0.1, matches);
      printPoseGraphMatches(poses, *sum_cloud, sum_origins, matches);
    } else {
      IGNORE = false;
    }
  }

  return EXIT_SUCCESS;
}
