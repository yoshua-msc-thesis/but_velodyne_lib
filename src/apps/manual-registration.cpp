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
#include <but_velodyne/CollarLinesRegistration.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>
#include <but_velodyne/Termination.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;
namespace po = boost::program_options;

class ManualRegistration {
public:
  ManualRegistration(const VelodynePointCloud &src_cloud_, const VelodynePointCloud &trg_cloud_,
      const Eigen::Affine3f &init_transform_) :
    estimated_transform(init_transform_) {

    src_cloud = Visualizer3D::colorizeCloud(src_cloud_, 255, 0, 0);
    trg_cloud = Visualizer3D::colorizeCloud(trg_cloud_, 0, 0, 255);

    vector<int> indices;
    removeNaNFromPointCloud(*src_cloud, *src_cloud, indices);
    split_idx = src_cloud->size();
    removeNaNFromPointCloud(*trg_cloud, *trg_cloud, indices);

    PolarGridOfClouds src_polar(src_cloud_);
    PolarGridOfClouds trg_polar(src_cloud_);

    CollarLinesFilter filter(params.linesPerCellPreserved);
    src_lines = LineCloud::Ptr(new LineCloud(src_polar, params.linesPerCellGenerated, filter));
    trg_lines = LineCloud::Ptr(new LineCloud(trg_polar, params.linesPerCellGenerated, filter));
    params.targetError = 0.001;

    pclVis = visualizer.getViewer();
    pclVis->registerPointPickingCallback(&ManualRegistration::pickPointCallback, *this);
    pclVis->registerKeyboardCallback(&ManualRegistration::keyCallback, *this);

/*    src_indices.push_back(11646);
    src_indices.push_back(13055);
    src_indices.push_back(43138);
    src_indices.push_back(39780);
    trg_indices.push_back(94203);
    trg_indices.push_back(95829);
    trg_indices.push_back(15491);
    trg_indices.push_back(12093);*/
  }

  Eigen::Affine3f run() {
    setDataToVisualizer();
    visualizer.show();
    return estimated_transform;
  }

protected:

  void pickPointCallback(const pcl::visualization::PointPickingEvent& event, void*) {
    int idx = event.getPointIndex();
    if (idx == -1)
      return;

    if(idx < split_idx) {
      if(src_indices.size() < trg_indices.size()) {
        src_indices.push_back(idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked source point but expected target - ignoring\n");
      }
    } else {
      if(src_indices.size() == trg_indices.size()) {
        trg_indices.push_back(idx-split_idx);
        setDataToVisualizer();
      } else {
        PCL_WARN("Clicked target point but expected source - ignoring\n");
      }
    }
  }

  void setDataToVisualizer() {
    PointCloud<PointXYZRGB>::Ptr sum_cloud(new PointCloud<PointXYZRGB>);
    *sum_cloud += *src_cloud;
    PointCloud<PointXYZRGB> trg_cloud_transformed;
    transformPointCloud(*trg_cloud, trg_cloud_transformed, estimated_transform);
    *sum_cloud += trg_cloud_transformed;
    pclVis->removeAllShapes();
    pclVis->removeAllPointClouds();
    visualizer.addColorPointCloud(sum_cloud);
    for(int i = 0; i < src_indices.size(); i++) {
      visualizer.addArrow(PointCloudLine(src_cloud->at(src_indices[i]),
                                         trg_cloud_transformed[trg_indices[i]]));
    }
    if(src_indices.size() < trg_indices.size()) {
      pclVis->addSphere(trg_cloud_transformed[trg_indices.back()], 0.1, "sphere");
    }
  }

  void keyCallback(const pcl::visualization::KeyboardEvent &event, void*) {
    if(event.keyDown()) {
      if(event.getKeySym() == "s") {
        PCL_INFO("Running transformation estimation using SVD ...\n");
        estimateTransform();
        setDataToVisualizer();
      } else if(event.getKeySym() == "u") {
        if(src_indices.size() < trg_indices.size()) {
          trg_indices.pop_back();
        } else {
          src_indices.pop_back();
        }
        setDataToVisualizer();
      } else if(event.getKeySym() == "a") {
        runAutomaticRegistration();
        setDataToVisualizer();
      }
    }
  }

  void estimateTransform() {
    if(src_indices.size() > 2) {
      pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> tfe;
      Eigen::Matrix4f t;
      tfe.estimateRigidTransformation(*trg_cloud, trg_indices, *src_cloud, src_indices, t);
      estimated_transform = Eigen::Affine3f(t);
    } else {
      PCL_WARN("Unable to estimate transformation with less than 3 matches - skipping.\n");
    }
  }

  void runAutomaticRegistration() {
    PCL_INFO("Running automatic transformation estimation using CLS ...\n");

    Termination termination(params.minIterations, params.maxIterations, params.maxTimeSpent,
                            params.significantErrorDeviation, params.targetError);
    CollarLinesRegistration icl_fitting(*src_lines, *trg_lines,
                                        registration_params, estimated_transform.matrix());
    while(!termination()) {
      float error = icl_fitting.refine();
      termination.addNewError(error);
      estimated_transform = Eigen::Affine3f(icl_fitting.getTransformation());
    }
  }

private:
  PointCloud<PointXYZRGB>::Ptr src_cloud, trg_cloud;
  int split_idx;
  Visualizer3D visualizer;
  pcl::visualization::PCLVisualizer::Ptr pclVis;
  vector<int> src_indices;
  vector<int> trg_indices;

  Eigen::Affine3f estimated_transform;

  LineCloud::Ptr src_lines, trg_lines;
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;
};

int main(int argc, char** argv) {

  string src_filename, trg_filename;

  if(argc != 4) {
    cerr << "ERROR, expected arguments: [src_cloud.pcd] [target_cloud.pcd] [init_poses.txt]" << endl;
    return EXIT_FAILURE;
  }

  VelodynePointCloud src_cloud, trg_cloud;
  VelodynePointCloud::fromFile(argv[1], src_cloud, true);
  VelodynePointCloud::fromFile(argv[2], trg_cloud, true);

  vector<Eigen::Affine3f> poses = KittiUtils::load_kitti_poses(argv[3]);

  ManualRegistration registration(src_cloud, trg_cloud, poses[0].inverse()*poses[1]);
  Eigen::Affine3f t = registration.run();
  KittiUtils::printPose(cout, Eigen::Matrix4f::Identity());
  KittiUtils::printPose(cout, t.matrix());

  return EXIT_SUCCESS;
}