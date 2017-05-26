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

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

void normalizeMinMax(const PointCloud<PointXYZI> &in, PointCloud<PointXYZI> &out,
    float skiprate, float expected_min, float expected_max) {
  vector<float> intensities;
  for (pcl::PointCloud<PointXYZI>::const_iterator pt = in.begin(); pt < in.end(); pt++) {
    intensities.push_back(pt->intensity);
  }
  sort(intensities.begin(), intensities.end());
  float min_intensity = intensities[intensities.size()*skiprate];
  float max_intensity = intensities[intensities.size()*(1-skiprate)];

  out.resize(in.size());
  for (int i = 0; i < in.size(); i++) {
    copyXYZ(in[i], out[i]);
    out[i].intensity = (in[i].intensity-min_intensity)/(max_intensity-min_intensity) * (expected_max-expected_min) + expected_min;
    out[i].intensity = MIN(MAX(out[i].intensity, 0.0), 1.0);
  }
}

class HorizontalAligner {
public:
  HorizontalAligner(Visualizer3D &vis_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_) :
    vis(vis_), in_cloud(cloud_),
    vis_cloud(new pcl::PointCloud<pcl::PointXYZRGB>),
    picked_indices(new pcl::PointIndices),
    plane_horizontal_corr(Eigen::Affine3f::Identity()) {
    vis.getViewer()->setBackgroundColor(0, 0, 0);
    vis.getViewer()->addCoordinateSystem(5);
    *vis_cloud += *in_cloud;
    pcl::compute3DCentroid(*in_cloud, centroid);
  }

  Eigen::Affine3f run() {
    setVisData();
    vis.getViewer()->registerAreaPickingCallback(&HorizontalAligner::planePointsPicker, *this);
    vis.show();
    return plane_horizontal_corr;
  }

  void planePointsPicker(const pcl::visualization::AreaPickingEvent& event, void*) {
    event.getPointsIndices(picked_indices->indices);
    findLargestPlane();

    Eigen::Vector3f expected_y(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    if(expected_y(1) < 0) {
      // preserve Y up/down orientation
      expected_y *= -1.0;
    }
    Eigen::Vector3f expected_z(0, expected_y(2), -expected_y(1));
    getTransformationFromTwoUnitVectorsAndOrigin(expected_y, expected_z, centroid.head(3), plane_horizontal_corr);

    setVisData();
  }

  const pcl::ModelCoefficients& getCoefficients() const {
    return coefficients;
  }

protected:
  void findLargestPlane() {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(in_cloud);
    seg.setIndices(picked_indices);
    seg.segment(plane_indices, coefficients);
    if (plane_indices.indices.size() == 0) {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
      return;
    }
  }

  void setVisData() {
    for(int i = 0; i < in_cloud->size(); i++) {
      PointXYZRGB &vis_pt = vis_cloud->at(i);
      PointXYZRGB &in_pt = in_cloud->at(i);
      vis_pt.r = in_pt.r;
      vis_pt.g = in_pt.g;
      vis_pt.b = in_pt.b;
      copyXYZ(in_pt, vis_pt);
    }
    for(vector<int>::iterator i = plane_indices.indices.begin(); i < plane_indices.indices.end(); i++) {
      PointXYZRGB &vis_pt = vis_cloud->at(*i);
      vis_pt.r = vis_pt.b = 0;
      vis_pt.g = 200;
    }
    vis.keepOnlyClouds(0).addColorPointCloud(vis_cloud, plane_horizontal_corr.matrix());
  }
private:
  Visualizer3D &vis;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, vis_cloud;
  pcl::PointIndices plane_indices;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr picked_indices;
  Eigen::Vector4f centroid;
  Eigen::Affine3f plane_horizontal_corr;
};

bool parse_arguments(int argc, char **argv,
  string &input_cloud, string &output_prefix, int &snapshots_count) {

  po::options_description desc("Horizontal alignment and defile of point cloud\n"
      "======================================\n"
      " * Allowed options");
  desc.add_options()
    ("help,h", "produce help message")
    ("input_cloud,i", po::value<string>(&input_cloud)->required(), "Input point cloud")
    ("output_prefix,o", po::value<string>(&output_prefix)->required(), "Output prefix .pcd (for cloud and snapshots)")
    ("snapshots_count,s", po::value<int>(&snapshots_count)->default_value(10), "Snapshots per 360deg rotation.")
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
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl << std::endl << desc
        << std::endl;
    return false;
  }
  return true;
}


int main(int argc, char** argv) {

  string input_cloud, output_prefix;
  int snapshots_count;
  if(!parse_arguments(argc, argv, input_cloud, output_prefix, snapshots_count)) {
    return EXIT_FAILURE;
  }

  PointCloud<PointXYZI> cloud;
  pcl::io::loadPCDFile(input_cloud, cloud);
  normalizeMinMax(cloud, cloud, 0.01, 0.1, 1.0);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud = Visualizer3D::colorizeCloud(cloud, true);

  Visualizer3D vis;
  HorizontalAligner aligner(vis, rgb_cloud);
  Eigen::Affine3f t = aligner.run();
  KittiUtils::printPose(cout, t.matrix());

  transformPointCloud(*rgb_cloud, *rgb_cloud, t);
  io::savePCDFileBinary(output_prefix, *rgb_cloud);

  vis.keepOnlyClouds(0).addColorPointCloud(rgb_cloud).show();

  if(snapshots_count > 0) {
    float resolution = 2*M_PI / snapshots_count;
    for(int i = 0; i < snapshots_count; i++) {
      Eigen::Affine3f R = getTransformation(0, 0, 0, 0, resolution*i, 0);
      stringstream ss;
      ss << output_prefix << "." << i << ".png";
      vis.keepOnlyClouds(0).addColorPointCloud(rgb_cloud, R.matrix()).saveSnapshot(ss.str());
    }
  }

  return EXIT_SUCCESS;
}
