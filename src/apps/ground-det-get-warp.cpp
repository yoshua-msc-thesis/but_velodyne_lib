/*
 * Velodyne data labeling for training of ground detector.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 02/08/2016
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
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>

#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/GroundDetectionDataGenerator.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

void showWarp(const Mat &warp, string out_filename) {
  Mat arrows = Mat::zeros(warp.rows, warp.cols, CV_8UC1);
  for(int r = 0; r < VelodynePointCloud::VELODYNE_RINGS_COUNT; r++) {
    for(int p = 0; p < PolarGridOfClouds::getPolarBins(); p++) {
      if(r % 20 == 0 && p % 20 == 0) {
	Vec2i dx_dy = warp.at<Vec2i>(r, p);
	Point pt(p, r);
	Point ptd(p+dx_dy.val[0], r+dx_dy.val[1]);
	arrowedLine(arrows, pt, ptd, Scalar(255));
      }
    }
  }
  imwrite(out_filename, arrows);
}

int main(int argc, char** argv) {
  if(argc != 5) {
    cerr << "Invalid arguments, expected: <source-cloud> <target-cloud> <both-poses-file> <output-warp.yaml.gz>" << endl;
    return EXIT_FAILURE;
  }
  string out_yaml_file = argv[4];
  VelodynePointCloud source_cloud, target_cloud;
  VelodynePointCloud::fromFile(argv[1], source_cloud);
  VelodynePointCloud::fromFile(argv[2], target_cloud);
  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[3]);

  Eigen::Matrix4f delta_pose = poses[0].matrix().inverse() * poses[1].matrix();
  Eigen::Matrix4f delta_pose_inv = delta_pose.inverse();

  /*Visualizer3D().setColor(100, 100, 100).addPointCloud(target_cloud)
      .setColor(0, 0, 255).addPointCloud(source_cloud)
      .setColor(255, 0, 0).addPointCloud(target_cloud, delta_pose).show();*/

  PolarGridOfClouds::POLAR_SUPERBINS = 360;
  PolarGridOfClouds::BIN_SUBDIVISION = 1;
  PolarGridOfClouds::Ptr source_polar_grid = PolarGridOfClouds(source_cloud).summarize();
  PolarGridOfClouds::Ptr target_polar_grid = PolarGridOfClouds(target_cloud).summarize();

  vector<CellId> index;
  PointCloud<PointXYZ>::Ptr source_summarized(new PointCloud<PointXYZ>);
  for(int r = 0; r < VelodynePointCloud::VELODYNE_RINGS_COUNT; r++) {
    for(int p = 0; p < PolarGridOfClouds::getPolarBins(); p++) {
      CellId cellId(p, r);
      const VelodynePointCloud &cell = source_polar_grid->at(cellId);
      if(!cell.empty()) {
	PointXYZ pt = PointXYZIRtoPointXYZ(cell.front());
	source_summarized->push_back(pt);
	index.push_back(cellId);
      }
    }
  }

  PointCloud<PointXYZ>::Ptr source_summarized_transformed(new PointCloud<PointXYZ>);
  transformPointCloud(*source_summarized, *source_summarized_transformed, delta_pose_inv);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(source_summarized_transformed);

  Mat warp(VelodynePointCloud::VELODYNE_RINGS_COUNT, PolarGridOfClouds::getPolarBins(), CV_32SC2);
  for(int r = 0; r < VelodynePointCloud::VELODYNE_RINGS_COUNT; r++) {
    for(int p = 0; p < PolarGridOfClouds::getPolarBins(); p++) {
      CellId cellId(p, r);
      const VelodynePointCloud &cell = target_polar_grid->at(cellId);
      int dx, dy;
      if(!cell.empty()) {
	std::vector<int> pointIdx(1);
	std::vector<float> pointDist(1);
	PointXYZ pt = PointXYZIRtoPointXYZ(cell.front());
	int found = kdtree.nearestKSearch(pt, 1, pointIdx, pointDist);
	if(pointIdx.front() < index.size()) {
	  CellId source_cell = index[pointIdx.front()];
	  dx = source_cell.polar - p;
	  dy = source_cell.ring - r;
	} else {
	  static int count = 0;
	  cerr << "index: " << pointIdx.front() << "; found pts: " << found << "; indexes: " << index.size() << "; dist: " << pointDist.front()
	      << "; count: " << count++ << endl;
	  cerr << pt << endl;
	  dx = dy = 0;
	}
      } else {
	dx = dy = 0;
      }
      warp.at<Vec2i>(r, p) = Vec2i(dx, dy);
    }
  }

  showWarp(warp, out_yaml_file + ".png");

  FileStorage fs(out_yaml_file, FileStorage::WRITE);
  fs << "warp" << warp;

  return EXIT_SUCCESS;
}
