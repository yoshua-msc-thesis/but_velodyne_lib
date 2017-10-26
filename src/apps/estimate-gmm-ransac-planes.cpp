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
#include <but_velodyne/Clustering.h>

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

bool IGNORE;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     float &gmm_prob_threshold,
                     float &inlier_tolerance, float &normal_dist_weight,
                     string &normals_filename, string &indices_filename, string &labeled_cloud_filename,
                     vector<string> &clouds_to_process) {
  string pose_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
    ("gmm_prob_threshold,g", po::value<float>(&gmm_prob_threshold)->default_value(0.99), "Gmm probability threshold.")
    ("inlier_tolerance,t", po::value<float>(&inlier_tolerance)->default_value(0.2), "Inlier tolerance.")
    ("normal_dist_weight,w", po::value<float>(&normal_dist_weight)->default_value(0.5), "Weight of normals angular distance.")
    ("sensor_poses,s", po::value<string>(&sensor_poses_filename)->default_value(""), "Sensor poses (calibration).")
    ("normals_filename,n", po::value<string>(&normals_filename)->required(), "Normals filename.")
    ("indices_filename,i", po::value<string>(&indices_filename)->required(), "Indices filename.")
    ("clusters_filename,c", po::value<string>(&labeled_cloud_filename)->required(), "Labels by clustering filename.")
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

void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
  if(event.keyDown()) {
    if(event.getKeySym() == "i") {
      IGNORE = true;
    }
  }
}

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<string> pcd_files;
  SensorsCalibration calibration;
  string normals_filename, indices_filename, labeled_cloud_filename;
  float inlier_tolerance, normal_dist_weight, gmm_prob_threshold;
  if(!parse_arguments(argc, argv,
      poses, calibration,
      gmm_prob_threshold,
      inlier_tolerance, normal_dist_weight,
      normals_filename, indices_filename, labeled_cloud_filename,
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

  cerr << "Loading indices, labels & subsampling ..." << endl;
  PointIndices::Ptr indices(new PointIndices);
  load_vector_from_file(indices_filename, indices->indices);
  PointCloud<PointXYZI>::Ptr subsampled_cloud(new PointCloud<PointXYZI>);
  extract_indices(sum_cloud, indices, *subsampled_cloud);
  vector<Origin> subsampled_origins;
  extract_indices(sum_origins, indices->indices, subsampled_origins);

  PointCloud<LabeledPoint>::Ptr labeled_subsampled_cloud(new PointCloud<LabeledPoint>);
  io::loadPCDFile(labeled_cloud_filename, *labeled_subsampled_cloud);

  vector<int> labels(labeled_subsampled_cloud->size());
  int expected_planes = -1;
  for(int i = 0; i < labels.size(); i++) {
    labels[i] = labeled_subsampled_cloud->at(i).label;
    expected_planes = MAX(expected_planes, labels[i]+1);
  }
  vector< PointIndices::Ptr > indices_clusters;
  invert_indices(labels, indices_clusters);

  SACSegmentationFromNormals<PointXYZI, Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_NORMAL_PLANE);
  seg.setMethodType(SAC_PROSAC);
  seg.setDistanceThreshold(inlier_tolerance);
  seg.setNormalDistanceWeight(normal_dist_weight);

  printPoseGraphPrefix(poses);

  Visualizer3D vis;

  vector< PointCloud<LabeledPoint> > clusters(expected_planes);
  for(int i = 0; i < labeled_subsampled_cloud->size(); i++) {
    LabeledPoint pt = labeled_subsampled_cloud->at(i);
    clusters[pt.label].push_back(pt);
  }
  vis.addPointClouds(clusters).show();
  vis.keepOnlyClouds(0);

  vis.getViewer()->registerKeyboardCallback(keyCallback);
  for(int plane_i = 0; plane_i < expected_planes; plane_i++) {

    for(vector<int>::iterator i = indices_clusters[plane_i]->indices.begin(); i < indices_clusters[plane_i]->indices.end();) {
      if(labeled_subsampled_cloud->at(*i).prob > gmm_prob_threshold) {
        i++;
      } else {
        i = indices_clusters[plane_i]->indices.erase(i);
      }
    }

    PointCloud<PointXYZI>::Ptr points_cluster(new PointCloud<PointXYZI>);
    extract_indices(subsampled_cloud, indices_clusters[plane_i], *points_cluster);

    PointCloud<Normal>::Ptr normals_cluster(new PointCloud<Normal>);
    extract_indices(subsampled_normals, indices_clusters[plane_i], *normals_cluster);

    vector<Origin> origins_clusters;
    extract_indices(subsampled_origins, indices_clusters[plane_i]->indices, origins_clusters);

    seg.setInputCloud(points_cluster);
    seg.setInputNormals(normals_cluster);
    PointIndices::Ptr plane_inliers(new PointIndices);
    ModelCoefficients coefficients;
    seg.segment(*plane_inliers, coefficients);

    PointCloud<PointXYZI>::Ptr plane(new PointCloud<PointXYZI>);
    extract_indices(points_cluster, plane_inliers, *plane);
    vector<Origin> plane_origins;
    extract_indices(subsampled_origins, plane_inliers->indices, plane_origins);

    IGNORE = false;
    vis.keepOnlyClouds(0).setColor(150, 150, 150).addPointCloud(*points_cluster);
    vis.addPointCloud(*plane).show();

    if(!IGNORE) {
      Eigen::Vector3f centroid;
      Eigen::Matrix3f covaraince;
      computeCentroidAndCovariance(*plane, centroid, covaraince);

      PointCloud<PointXYZI>::Ptr projectedPlane(new PointCloud<PointXYZI>);
      reduceWeakestDimension(*plane, covaraince, *projectedPlane);

      vector<cv::DMatch> matches;
      getClosestMatches<PointXYZI>(projectedPlane, 0.1, matches);
      printPoseGraphMatches(poses, *plane, plane_origins, matches);
    }
  }

  return EXIT_SUCCESS;
}
