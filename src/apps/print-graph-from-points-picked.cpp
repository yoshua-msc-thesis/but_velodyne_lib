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
#include <pcl/common/geometry.h>

#include <cv.h>
#include <cxeigen.hpp>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

float SUBSAMPLE_RATE = 0.01;
//float SUBSAMPLE_RATE = 1.0;
float QUERY_TO_TRAIN_RATE = 0.1;

bool parse_arguments(int argc, char **argv,
                     vector<Eigen::Affine3f> &poses,
                     SensorsCalibration &calibration,
                     vector<string> &clouds_to_process) {
  string pose_filename, cluster_labels_filename, sensor_poses_filename;

  po::options_description desc("Global optimization: print pose graph\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("pose_file,p", po::value<string>(&pose_filename)->required(), "KITTI poses file.")
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

class GraphFromPickedPoints {
public:
  GraphFromPickedPoints(const PointCloud<PointXYZI> &points_, const vector<Origin> &origins_,
      const vector<Eigen::Affine3f> &poses_, const SensorsCalibration &calibration_) :
    points(points_), origins(origins_), poses(poses_), calibration(calibration_),
    new_vertex(0), finish(false) {
    visualizer.getViewer()->setBackgroundColor(0.0, 0.0, 0.0);
    visualizer.getViewer()->registerAreaPickingCallback(&GraphFromPickedPoints::pickPointsCallback, *this);
    visualizer.getViewer()->registerKeyboardCallback(&GraphFromPickedPoints::keyCallback, *this);
    printPosesEdges();
  }

  bool run(void) {
    preserved_indices.resize(points.size());
    for(int i = 0; i < preserved_indices.size(); i++) {
      preserved_indices[i] = i;
    }
    indices_to_delete.clear();

    visualizeData();
    visualizer.show();

    cerr << "Generating landmark edges" << endl;
    PointCloud<PointXYZI> points_picked;
    copyPointCloud(points, preserved_indices, points_picked);
    PointCloud<PointXYZI>::Ptr projectedCluster(new PointCloud<PointXYZI>);
    Eigen::Vector3f centroid;
    Eigen::Matrix3f covariance;
    computeCentroidAndCovariance(points_picked, centroid, covariance);
    reduceWeakestDimension(points_picked, covariance, *projectedCluster);
    vector<cv::DMatch> matches;
    //getClosestMatches<PointXYZI>(projectedCluster, QUERY_TO_TRAIN_RATE, matches);
    getClosestMatches(points_picked, *projectedCluster, matches);
    cerr << matches.size() << " landmark edges found" << endl;

    printLandmarkEdges(matches);
    visualizer.addMatches(matches, points_picked, points_picked).show();
    visualizer.getViewer()->removeAllShapes();

    return finish;
  }

protected:

  void getClosestMatches(const PointCloud<PointXYZI> &points_picked,
      const PointCloud<PointXYZI> &points_projected, vector<cv::DMatch> &matches) {
    for(int i = 0; i < points_picked.size(); i++) {
      for(int j = 0; j < i; j++) {
        float original_distance = geometry::distance(points_picked[i], points_picked[j]);
        float projected_distance = geometry::distance(points_projected[i], points_projected[j]);
        if(original_distance > 10*projected_distance) {
          matches.push_back(cv::DMatch(i, j, original_distance));
        }
      }
    }
  }

  void visualizeData() {
    PointCloud<PointXYZRGB>::Ptr points_to_vis(new PointCloud<PointXYZRGB>);
    points_to_vis->resize(preserved_indices.size());
    for(int i = 0; i < preserved_indices.size(); i++) {
      const PointXYZI &pt = points[preserved_indices[i]];
      PointXYZRGB &pt_rgb = points_to_vis->at(i);
      copyXYZ(pt, pt_rgb);
      pt_rgb.r = pt_rgb.g = pt_rgb.b = pt.intensity*255;
    }
    for(vector<int>::iterator i = indices_to_delete.begin(); i < indices_to_delete.end(); i++) {
      PointXYZRGB &pt_rgb = points_to_vis->at(*i);
      pt_rgb.r = 200;
      pt_rgb.g = pt_rgb.b = 50;
    }
    visualizer.keepOnlyClouds(0).addColorPointCloud(points_to_vis);
  }

  void pickPointsCallback(const pcl::visualization::AreaPickingEvent& event, void*) {
    vector<int> indices;
    if(event.getPointsIndices(indices_to_delete)) {
      visualizeData();
    } else {
      indices_to_delete.clear();
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "d") {
        for(vector<int>::iterator di = indices_to_delete.begin(); di < indices_to_delete.end(); di++) {
          preserved_indices[*di] = -1;
        }
        indices_to_delete.clear();
        for(vector<int>::iterator pi = preserved_indices.begin(); pi < preserved_indices.end();) {
          if(*pi < 0) {
            pi = preserved_indices.erase(pi);
          } else {
            pi++;
          }
        }
        visualizeData();
      } else if(event.getKeySym() == "f") {
        finish = true;
      }
    }
  }

  void printPosesEdges() {
    for(int pi = 1; pi < poses.size(); pi++) {
      Eigen::Affine3f delta_pose = poses[pi-1].inverse() * poses[pi];
      std::cout << PoseGraphEdge(pi-1, pi, delta_pose.matrix(), POSES_COVARIANCE) << std::endl;
    }
    new_vertex = poses.size();
  }

  void printLandmarkEdges(const vector<cv::DMatch> &matches) {
    cerr << "Printing matches (" << matches.size() << ")" << endl;
    for(std::vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
      int i1 = preserved_indices[m->trainIdx];
      int i2 = preserved_indices[m->queryIdx];
      Origin o1 = origins[i1];
      Origin o2 = origins[i2];
      Eigen::Affine3f t1 = poses[o1.pose_id];
      Eigen::Affine3f t2 = poses[o2.pose_id];
      PointXYZI pt1 = transformPoint(points[i1], t1.inverse());
      PointXYZI pt2 = transformPoint(points[i2], t2.inverse());
      std::cout << PoseToLandmarkGraphEdge(o1.pose_id, new_vertex, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << std::endl;
      std::cout << PoseToLandmarkGraphEdge(o2.pose_id, new_vertex, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << std::endl;
      new_vertex++;
    }
  }

  void printPosesEdgesWithCalibration() {
    for(int pi = 1; pi < poses.size(); pi++) {
      Eigen::Affine3f delta_pose = poses[pi-1].inverse() * poses[pi];
      std::cout << PoseGraphEdge(pi-1, pi, delta_pose.matrix(), POSES_COVARIANCE) << std::endl;
    }
    for(int si = 1; si < SENSORS; si++) {
      for(int pi = 0; pi < poses.size(); pi++) {
        std::cout << PoseGraphEdge(pi, pi+si*poses.size(), calibration.ofSensor(si).matrix(), POSES_COVARIANCE) << std::endl;
      }
    }
    new_vertex = poses.size()*calibration.sensorsCount();
  }

  void printPosesEdgesWithCalibration(const vector<cv::DMatch> &matches) {
    cerr << "Printing matches (" << matches.size() << ")" << endl;
    for(std::vector<cv::DMatch>::const_iterator m = matches.begin(); m < matches.end(); m++) {
      int i1 = preserved_indices[m->trainIdx];
      int i2 = preserved_indices[m->queryIdx];
      Origin o1 = origins[i1];
      Origin o2 = origins[i2];
      Eigen::Affine3f t1 = poses[o1.pose_id]*calibration.ofSensor(o1.sensor_id);
      Eigen::Affine3f t2 = poses[o2.pose_id]*calibration.ofSensor(o2.sensor_id);
      PointXYZI pt1 = transformPoint(points[i1], t1.inverse());
      PointXYZI pt2 = transformPoint(points[i2], t2.inverse());
      std::cout << PoseToLandmarkGraphEdge(o1.edgeIdx(poses.size()), new_vertex, pt1.x, pt1.y, pt1.z, LANDMARK_COVARIANCE) << std::endl;
      std::cout << PoseToLandmarkGraphEdge(o2.edgeIdx(poses.size()), new_vertex, pt2.x, pt2.y, pt2.z, LANDMARK_COVARIANCE) << std::endl;
      new_vertex++;
    }
  }

private:
  const PointCloud<PointXYZI> points;
  const vector<Origin> origins;
  const vector<Eigen::Affine3f> poses;
  const SensorsCalibration calibration;
  Visualizer3D visualizer;
  vector<int> indices_to_delete;
  vector<int> preserved_indices;
  int new_vertex;
  bool finish;

  static const cv::Mat POSES_COVARIANCE;
  static const cv::Mat LANDMARK_COVARIANCE;
  static const int SENSORS = 2;
};

const cv::Mat GraphFromPickedPoints::POSES_COVARIANCE = cv::Mat::eye(6, 6, CV_32FC1)*0.1;
const cv::Mat GraphFromPickedPoints::LANDMARK_COVARIANCE = cv::Mat::eye(3, 3, CV_32FC1)*0.1;

int main(int argc, char** argv) {

  vector<Eigen::Affine3f> poses;
  vector<string> pcd_files;
  SensorsCalibration calibration;
  if(!parse_arguments(argc, argv,
      poses, calibration, pcd_files)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZI> sum_cloud;
  vector<Origin> sum_origins;
  VelodyneFileSequence file_sequence(pcd_files, calibration);
  for (int frame_i = 0; file_sequence.hasNext(); frame_i++) {
    if(frame_i >= poses.size()) {
      std::cerr << "No remaining pose for cloud: " << frame_i << std::endl << std::flush;
      break;
    }

    PointCloud<PointXYZI> cloud;
    VelodyneMultiFrame multiframe = file_sequence.getNext();
    multiframe.subsample(SUBSAMPLE_RATE);
    multiframe.joinTo(cloud);
    transformPointCloud(cloud, cloud, poses[frame_i]);
    sum_cloud += cloud;

    for(int sensor_i = 0; sensor_i < calibration.sensorsCount(); sensor_i++) {
      for(int pt_i = 0; pt_i < multiframe.clouds[sensor_i]->size(); pt_i++) {
        sum_origins.push_back(Origin(frame_i, sensor_i));
      }
    }
  }

  GraphFromPickedPoints graph_printer(sum_cloud, sum_origins, poses, calibration);
  while(!graph_printer.run())
    ;

  return EXIT_SUCCESS;
}
