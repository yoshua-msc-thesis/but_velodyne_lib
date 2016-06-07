/*
 * Velodyne data labeling for training of ground detector.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 19/09/2014
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

#include <Eigen/Eigenvalues>

#include <boost/program_options.hpp>
#include <boost/math/distributions/normal.hpp>

#include <cv.h>
#include <opencv2/highgui.hpp>

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

void read_lables(const string ann_filename,
		 vector<int> &ann_labels) {
  ifstream ann_file(ann_filename.c_str());
  int label;
  int labels_read = 0;
  while(ann_file >> label) {
    labels_read++;
    ann_labels.push_back(label);
  }
  cerr << "Read " << labels_read << " labels" << endl;
}

PointCloud<PointXYZRGB>::Ptr color_by_labels(const VelodynePointCloud &cloud,
		 const vector<int> &ann_labels) {
  PointCloud<PointXYZRGB>::Ptr color_cloud(new PointCloud<PointXYZRGB>);
  for(int i = 0; i < cloud.size(); i++) {
    PointXYZRGB pt_color;
    switch(ann_labels[i]) {
      case 3:
      case 5:
	pt_color.r = 255; pt_color.g = pt_color.b = 0; break;
      case 4:
	pt_color.g = 255; pt_color.r = pt_color.b = 0; break;
      default:
	pt_color.b = 255; pt_color.r = pt_color.g = 0;
    }
    pt_color.x = cloud[i].x;
    pt_color.y = cloud[i].y;
    pt_color.z = cloud[i].z;
    color_cloud->push_back(pt_color);
  }
  return color_cloud;
}

int main(int argc, char** argv) {
  if(argc != 4) {
    cerr << "Invalid arguments, expected: <input-cloud> <input-ann> <output-yaml-file>" << endl;
    return EXIT_FAILURE;
  }
  string cloud_filename = argv[1];
  string ann_filename = argv[2];
  string out_yaml_file = argv[3];

  VelodynePointCloud new_cloud;
  VelodynePointCloud::fromFile(cloud_filename, new_cloud);

  vector<int> ann_labels;
  read_lables(ann_filename, ann_labels);

  PointCloud<PointXYZRGB>::Ptr colored_cloud = color_by_labels(new_cloud, ann_labels);
  io::savePCDFile(out_yaml_file + ".colored.pcd", *colored_cloud, true);

  GroundDetectionDataGenerator data_generator(new_cloud);

  map<string, Mat> data;
  data["y"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::Y);
  data["range"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::RANGE);
  data["intensity"] = data_generator.getMatrixOf(GroundDetectionDataGenerator::INTENSITY);
  data_generator.getGroundLabelsFromAnn(ann_labels, data["ground_labels"]);

  FileStorage storage(out_yaml_file, FileStorage::WRITE);
  for(map<string, Mat>::iterator m = data.begin(); m != data.end(); m++) {
      storage << m->first << m->second;
      Mat equalized;
      normalize(m->second, equalized, 0.0, 255.0, NORM_MINMAX);
      equalized.convertTo(equalized, CV_8UC1);
      imwrite(out_yaml_file + "." + m->first + ".png", equalized);
  }

  return EXIT_SUCCESS;
}
