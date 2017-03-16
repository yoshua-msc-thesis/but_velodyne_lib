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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

class KeypointId {
public:
  string frame_id;
  int pt_id;

  KeypointId(const string frame_id_, int pt_id_) :
    frame_id(frame_id_), pt_id(pt_id_) {
  }
};

void findMatches(const PointCloud<PointXYZ>::Ptr sum_cloud,
    const vector<KeypointId> &keypoints_idx,
    const map<string, float> &frame_trajectories,
    map<string, Eigen::Affine3f> &frame_poses,
    vector<cv::DMatch> &matches,
    float radius) {
  KdTreeFLANN<PointXYZ> kdTree;
  kdTree.setInputCloud(sum_cloud);

  vector<int> trg_indices;
  vector<float> distances;
  radius *= radius; // because radiusSearch returns squared distance

  float done = 0;
  for(int src_i = 0; src_i < sum_cloud->size(); src_i++) {
    kdTree.radiusSearch(sum_cloud->at(src_i), radius, trg_indices, distances);
    string src_frame = keypoints_idx[src_i].frame_id;
    float src_trajectory = frame_trajectories.find(src_frame)->second;
    for(vector<int>::iterator trg_i = trg_indices.begin(); trg_i < trg_indices.end(); trg_i++) {
      string trg_frame = keypoints_idx[*trg_i].frame_id;
      float trg_trajectory = frame_trajectories.find(trg_frame)->second;
      //cerr << src_trajectory << " " << trg_trajectory << endl;
      float poses_diff = (frame_poses[src_frame].translation()-frame_poses[trg_frame].translation()).norm();
      if(src_trajectory-trg_trajectory > 2*poses_diff) {
        matches.push_back(cv::DMatch(src_i, *trg_i, 0.0));
        break;
      }
    }
    trg_indices.clear();
    distances.clear();
    if(src_i%(sum_cloud->size()/10) == 0) {
      cerr << done << "%" << endl;
      done += 10;
    }
  }
  cerr << "Found " << matches.size() << " matches" << endl;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " -p <poses> <point-cloud>+" << endl;
    return 1;
  }

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  for (int i = 1; i < argc; i++) {
    cerr << argv[i] << endl;
    if (strcmp(argv[i], "-p") == 0 && (i < argc - 1)) {
      i++;
      poses = KittiUtils::load_kitti_poses(argv[i]);
    } else {
      filenames.push_back(string(argv[i]));
    }
  }

  if(poses.empty()) {
    cerr << "Missing pose file. Usage: " << argv[0] << " -p <poses> <point-cloud>+" << endl;
    return 1;
  }

  PointCloud<PointXYZ> cloud;
  PointCloud<PointXYZ>::Ptr sum_cloud(new PointCloud<PointXYZ>);
  vector<KeypointId> keypoints_idx;
  map<string, float> frame_trajectories;
  map<string, Eigen::Affine3f> frame_poses;
  float trajectory = 0;
  for(int i = 0; i < filenames.size(); i++) {

    std::cerr << "Processing keypoints from file: " << filenames[i] << std::endl << std::flush;
    pcl::io::loadPCDFile(filenames[i], cloud);

    transformPointCloud(cloud, cloud, poses[i]);
    *sum_cloud += cloud;

    if(i > 0) {
      trajectory += (poses[i].translation() - poses[i-1].translation()).norm();
    }
    frame_trajectories[filenames[i]] = trajectory;
    frame_poses[filenames[i]] = poses[i];

    for(int j = 0; j < cloud.size(); j++) {
      keypoints_idx.push_back(KeypointId(filenames[i], j));
    }
  }

  vector<cv::DMatch> matches;
  findMatches(sum_cloud, keypoints_idx, frame_trajectories, frame_poses, matches, 1.0);

  cerr << "Found " << matches.size() << " matches." << endl;

  for(vector<cv::DMatch>::iterator m = matches.begin(); m < matches.end(); m++) {
    cout << keypoints_idx[m->queryIdx].frame_id << " " << keypoints_idx[m->queryIdx].pt_id << " " <<
        keypoints_idx[m->trainIdx].frame_id << " " << keypoints_idx[m->trainIdx].pt_id << endl;
  }

  /*Visualizer3D()
    .setColor(200, 200, 200).addPointCloud(*sum_cloud)
    .addMatches(matches, *sum_cloud, *sum_cloud)
    .show();*/

  return EXIT_SUCCESS;
}
