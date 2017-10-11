/*
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

#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace pcl;
using namespace but_velodyne;

typedef PointXYZ PointType;
typedef PointCloud<PointType> Cloud;
typedef Cloud::iterator It;

float TOLERANCE = 0.05;

float pointPlaneDistance(const PointType &pt, const ModelCoefficients &coeff) {
  float a = coeff.values[0];
  float b = coeff.values[1];
  float c = coeff.values[2];
  float d = coeff.values[3];
  return fabs(pt.x*a + pt.y*b + pt.z*c + d) / sqrt(a*a + b*b + c*c);
}

int main(int argc, char** argv) {

  Visualizer3D vis;
  for (int i = 1; i < argc; i++) {
    PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
    PointCloud<PointType> plane, rest;
    io::loadPCDFile(argv[i], *cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(TOLERANCE);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, coefficients);

    if (inliers->indices.size() == 0) {
      PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    pcl::ExtractIndices<PointType> eifilter;
    eifilter.setIndices(inliers);
    eifilter.setInputCloud(cloud);
    eifilter.filter(plane);
    eifilter.setNegative(true);
    eifilter.filter(rest);

    vector<float> distances(plane.size());
    float average = 0;
    for(int i = 0; i < plane.size(); i++) {
      distances[i] = pointPlaneDistance(plane[i], coefficients);
      average += distances[i];
    }
    average /= distances.size();
    float stdev = 0;
    for(int i = 0; i < plane.size(); i++) {
      float diff = distances[i] - average;
      stdev += pow(diff, 2);
    }
    stdev = sqrt(stdev/distances.size());
    cout << stdev << endl;

    vis.keepOnlyClouds(0)
        .setColor(0, 255, 0).addPointCloud(plane)
        .setColor(0, 0, 255).addPointCloud(rest)
        .show();
  }

  return EXIT_SUCCESS;
}
