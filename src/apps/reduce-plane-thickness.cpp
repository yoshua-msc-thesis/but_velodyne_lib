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
#include <but_velodyne/Clustering.h>
#include <but_velodyne/NormalsEstimation.h>
#include <but_velodyne/point_types.h>
#include <but_velodyne/EigenUtils.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     string &input_fn, float &plane_tolerance,
                     string &output_fn) {

  po::options_description desc("Optimization of the wall thickness\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_points,i", po::value<string>(&input_fn)->required(), "Input pcd file.")
    ("plane_tolerance,t", po::value<float>(&plane_tolerance)->default_value(0.1), "Plane tolerance.")
    ("output_cloud,o", po::value<string>(&output_fn)->required(), "Output pcd file.")
  ;
  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);

  if (vm.count("help")) {
      std::cerr << desc << std::endl;
      return false;
  }
  try {
      po::notify(vm);
  } catch(std::exception& e) {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  return true;
}

void split_into_plane_clusters(const PointCloud<LabeledPoint> &input,
    vector< PointCloud<LabeledPoint> > &clusters) {
  int K = -1;
  for(PointCloud<LabeledPoint>::const_iterator p = input.begin(); p < input.end(); p++) {
    K = max(K, (int)p->label+1);
  }
  clusters.resize(K);
  for(PointCloud<LabeledPoint>::const_iterator p = input.begin(); p < input.end(); p++) {
    clusters[p->label].push_back(*p);
  }
}

void remove_nonplanar(vector< PointCloud<LabeledPoint> > &clusters, PointCloud<LabeledPoint> &non_plane) {
  for(vector< PointCloud<LabeledPoint> >::iterator c = clusters.begin(); c < clusters.end(); /*none*/) {
    float avg_prob = 0.0;
    for(PointCloud<LabeledPoint>::iterator p = c->begin(); p < c->end(); p++) {
      avg_prob += p->prob;
    }
    avg_prob /= c->size();

    Eigen::Vector3f centroid;
    Eigen::Matrix3f covaraince;
    computeCentroidAndCovariance(*c, centroid, covaraince);
    vector<float> ev;
    getEigenvalues(covaraince, ev);

    if(avg_prob > 0.98 && (ev[2] < 3*ev[1] && ev[1] > 2*ev[0])) {
      c++;
    } else {
      non_plane += *c;
      c = clusters.erase(c);
    }
  }
}

void reduce_thickness(PointCloud<PointXYZI>::Ptr points, float plane_tolerance) {
  SACSegmentation<PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(SACMODEL_PLANE);
  seg.setMethodType(SAC_PROSAC);
  seg.setDistanceThreshold(plane_tolerance);
  seg.setInputCloud(points);

  PointIndices::Ptr subsampled_inliers(new PointIndices);
  ModelCoefficients::Ptr coefficients(new ModelCoefficients);
  seg.segment(*subsampled_inliers, *coefficients);

  ProjectInliers<PointXYZI> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setInputCloud(points);
  proj.setModelCoefficients(coefficients);
  proj.filter(*points);
}

int main(int argc, char** argv) {

  string input_fn, output_fn;
  float plane_tolerance;

  if(!parse_arguments(argc, argv,
      input_fn, plane_tolerance,
      output_fn)) {
    return EXIT_FAILURE;
  }

  PointCloud<LabeledPoint> in_cloud, non_plane;
  io::loadPCDFile(input_fn, in_cloud);

  vector< PointCloud<LabeledPoint> > clusters;
  split_into_plane_clusters(in_cloud, clusters);

  remove_nonplanar(clusters, non_plane);

  PointCloud<PointXYZI> out_cloud;
  copyPointCloud(non_plane, out_cloud);
  for(vector< PointCloud<LabeledPoint> >::iterator c = clusters.begin(); c < clusters.end(); c++) {
    PointCloud<PointXYZI>::Ptr xyzi_points(new PointCloud<PointXYZI>);
    copyPointCloud(*c, *xyzi_points);
    reduce_thickness(xyzi_points, plane_tolerance);
    out_cloud += *xyzi_points;
  }

  io::savePCDFileBinary(output_fn, out_cloud);

  return EXIT_SUCCESS;
}
