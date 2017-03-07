/*
 * line-fitting-sample-processor.cpp
 *
 *  Created on: 21.4.2015
 *      Author: ivelas
 */

#include <cstdlib>
#include <cstdio>
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/visualization/common/common.h>

#include <cv.h>

#include "but_velodyne/VelodynePointCloud.h"
#include "but_velodyne/EigenUtils.h"
#include "but_velodyne/CollarLinesRegistration.h"

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

const float DEG_TO_RAD = M_PI/180;

float getArgument(PointXYZ from, PointXYZ dir, bool &isWithinCube) {
    float tx, ty, tz;
    tx = (1 - from.x) / dir.x;
    ty = (1 - from.y) / dir.y;
    tz = (1 - from.z) / dir.z;
    isWithinCube = MIN(tx-from.x, MIN(ty-from.y, tz-from.z)) > 0.0;
    return MIN(tx, MIN(ty, tz));
}

void noise(PointXYZIR &pt) {
  pt.x += cv::theRNG().gaussian(.2e-2);
  pt.y += cv::theRNG().gaussian(.2e-2);
  pt.z += cv::theRNG().gaussian(.2e-2);
}

void generate(PointXYZ origin, VelodynePointCloud &cloud) {
  int RINGS = VelodynePointCloud::VELODYNE_RINGS_COUNT/2;
  int POINTS = 1000;
  float VERT_ANGLE_MIN = -10;
  float VERT_ANGLE_MAX = 80;

  for(int r = 0; r < RINGS; r++) {
    float phi = (VERT_ANGLE_MAX - VERT_ANGLE_MIN)/RINGS*r + VERT_ANGLE_MIN;
    PointXYZ dir;
    dir.z = sin(phi*DEG_TO_RAD);
    for(int p = 0; p < POINTS; p++) {
      float theta = 360.0/POINTS * p;
      dir.x = sin(theta*DEG_TO_RAD);
      dir.y = cos(theta*DEG_TO_RAD);

      bool isWithin;
      float t = getArgument(origin, dir, isWithin);
      if(isWithin) {
        PointXYZIR pt;
        pt.x = dir.x*t + origin.x;
        pt.z = dir.y*t + origin.y;        // y <-> z
        pt.y = dir.z*t + origin.z;
        pt.ring = r;
        pt.intensity = 1.0;
        noise(pt);
        cloud.push_back(pt);
      }
    }
  }
}

class SnapName {
public:
  SnapName() : counter(0) {
  }

  string get() {
    stringstream ss;
    ss << "snap" << setw(3) << setfill('0') << counter << ".png";
    counter++;
    return ss.str();
  }

private:
  int counter;
};

/**
 * for d in *; do pushd $d/velodyne; ./line-fitting $(ls *.bin | sort | xargs); popd;  done | tee output.txt
 */
int main(int argc, char** argv)
{
  VelodynePointCloud source, target;
  PointXYZ origin(0, 0, 0.5);
  generate(origin, source);
  float dx = 0.01;
  float dy = 0.01;
  float dz = 0.1;
  origin.x += dx;
  origin.y += dy;
  origin.z += dz;
  generate(origin, target);
  Eigen::Affine3f t = getTransformation(dx, dy-0.3, dz, 0.01, 0.4, 0.2);
  transformPointCloud(target, target, t);

  SnapName nameGen;

  PolarGridOfClouds source_polar(source);
  PolarGridOfClouds target_polar(target);
  CollarLinesFilter filter(1);
  LineCloud source_lines(source_polar, 2, filter);
  LineCloud target_lines(target_polar, 2, filter);

  float axis_line_color[] = {.8, .8, .8};
  float src_line_color[] = {.0, .5, .8};
  float trg_line_color[] = {.8, .1, .1};
  uchar src_cloud_color[] = {src_line_color[2]*255, src_line_color[1]*255, src_line_color[0]*255};
  uchar trg_cloud_color[] = {trg_line_color[2]*255, trg_line_color[1]*255, trg_line_color[0]*255};

  Visualizer3D vis;
  vis.getViewer()->removeAllPointClouds();
  vis.getViewer()->removeAllShapes();
  vis.getViewer()->removeCoordinateSystem();
  vis.getViewer()->setBackgroundColor(0, 0, 0);
  visualization::Camera camera;
  camera.clip[0] = 0.108209;
  camera.clip[1] = 3.13248;
  camera.focal[0] = 0.626394;
  camera.focal[1] = 0.778356;
  camera.focal[2] = 0.312092;
  camera.pos[0] = 0.333445;
  camera.pos[1] = 0.206981;
  camera.pos[2] = -0.811689;
  camera.view[0] = 0.196155;
  camera.view[1] = -0.893794;
  camera.view[2] = 0.403306;
  //vis.getViewer()->setCameraParameters(camera);
  vis.getViewer()->setCameraPosition(camera.pos[0], camera.pos[1], camera.pos[2],
				     camera.view[0], camera.view[1], camera.view[2]);
  //vis.getViewer()->setCameraPosition(camera.pos[0], camera.pos[1], camera.pos[2]);
  PointXYZ corner(1, 1, 1);
  PointXYZ edgeUp = corner; edgeUp.y -= 1;
  PointXYZ edgeLeft = corner; edgeLeft.x -= 1.5;
  PointXYZ edgeRight = corner; edgeRight.z -= 1.5;
  vis.addLine(PointCloudLine(corner, edgeUp), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.addLine(PointCloudLine(corner, edgeLeft), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.addLine(PointCloudLine(corner, edgeRight), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source).show();

  float dyaw = 0.05;
  float total_yaw = 0.0;
  Eigen::Vector4f centroid;
  compute3DCentroid(source, centroid);
  Eigen::Affine3f R =
	getTransformation(centroid.x(), centroid.y(), centroid.z(), 0, 0, 0)*
	getTransformation(0, 0, 0, 0, dyaw, 0)*
	getTransformation(-centroid.x(), -centroid.y(), -centroid.z(), 0, 0, 0);
  VelodynePointCloud source_defile;
  source_defile += source;
  while(total_yaw < 2*M_PI) {
    total_yaw += dyaw;
    corner = transformPoint(corner, R);
    edgeUp = transformPoint(edgeUp, R);
    edgeLeft = transformPoint(edgeLeft, R);
    edgeRight = transformPoint(edgeRight, R);
    transformPointCloud(source_defile, source_defile, R);

    vis.getViewer()->removeAllPointClouds();
    vis.getViewer()->removeAllShapes();
    vis.addLine(PointCloudLine(corner, edgeUp), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeLeft), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeRight), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source_defile).saveSnapshot(nameGen.get());
  }

  vis.saveSnapshot(nameGen.get());
  vis.setPointSize(1);
  vis.getViewer()->removeAllPointClouds();
  vis.getViewer()->removeAllShapes();
  vis.addLine(PointCloudLine(corner, edgeUp), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.addLine(PointCloudLine(corner, edgeLeft), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.addLine(PointCloudLine(corner, edgeRight), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
  vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source).saveSnapshot(nameGen.get());
  vis.addLines(source_lines.getLines(), src_line_color[0], src_line_color[1], src_line_color[2]).saveSnapshot(nameGen.get());
  vis.setPointSize(2);
  vis.setColor(trg_cloud_color[0], trg_cloud_color[1], trg_cloud_color[2]).addPointCloud(target).saveSnapshot(nameGen.get());
  vis.getViewer()->removeAllPointClouds();
  vis.setPointSize(1);
  vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source);
  vis.setColor(trg_cloud_color[0], trg_cloud_color[1], trg_cloud_color[2]).addPointCloud(target);
  vis.addLines(target_lines.getLines(), trg_line_color[0], trg_line_color[1], trg_line_color[2]).saveSnapshot(nameGen.get());

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  for(int i = 0; i < 15; i++) {

    CollarLinesRegistration::Parameters clsParams(CollarLinesRegistration::NO_THRESHOLD,
						  CollarLinesRegistration::NO_WEIGHTS);
    CollarLinesRegistration icl_fitting(source_lines, target_lines,
					clsParams, transformation);

    for(int sampling_it = 0; sampling_it < i+1; sampling_it++) {
      icl_fitting.refine();
    }
    transformation = icl_fitting.getTransformation();

    LineCloud target_lines_transformed;
    target_lines.transform(transformation, target_lines_transformed);

    vis.getViewer()->removeAllPointClouds();
    vis.getViewer()->removeAllShapes();
    vis.addLine(PointCloudLine(corner, edgeUp), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeLeft), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeRight), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source)
        .setColor(trg_cloud_color[0], trg_cloud_color[1], trg_cloud_color[2]).addPointCloud(target, transformation)
        .addLines(source_lines.getLines(), src_line_color[0], src_line_color[1], src_line_color[2])
        .addLines(target_lines_transformed.getLines(), trg_line_color[0], trg_line_color[1], trg_line_color[2]).saveSnapshot(nameGen.get());
  }

  transformPointCloud(target, target, transformation);
  target_lines.transform(transformation);
  total_yaw = 0.0;
  while(total_yaw < 2*M_PI) {
    total_yaw += dyaw;
    corner = transformPoint(corner, R);
    edgeUp = transformPoint(edgeUp, R);
    edgeLeft = transformPoint(edgeLeft, R);
    edgeRight = transformPoint(edgeRight, R);
    transformPointCloud(source, source, R);
    transformPointCloud(target, target, R);
    source_lines.transform(R.matrix());
    target_lines.transform(R.matrix());

    vis.getViewer()->removeAllPointClouds();
    vis.getViewer()->removeAllShapes();
    vis.addLine(PointCloudLine(corner, edgeUp), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeLeft), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.addLine(PointCloudLine(corner, edgeRight), axis_line_color[0], axis_line_color[1], axis_line_color[2]);
    vis.setColor(src_cloud_color[0], src_cloud_color[1], src_cloud_color[2]).addPointCloud(source)
        .setColor(trg_cloud_color[0], trg_cloud_color[1], trg_cloud_color[2]).addPointCloud(target)
        .addLines(source_lines.getLines(), src_line_color[0], src_line_color[1], src_line_color[2])
        .addLines(target_lines.getLines(), trg_line_color[0], trg_line_color[1], trg_line_color[2]).saveSnapshot(nameGen.get());
  }
  return EXIT_SUCCESS;
}
