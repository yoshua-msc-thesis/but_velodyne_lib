// file: example4-velodyne.c
//
// Given a log file, extract velodyne measurements within a specified
// time window.  Measurements are projected into the local frame and 
// written to stdout as a point cloud.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>

#include "config.h"
#include "config_util.h"
#include "eventlog.h"
#include "lcmtypes_velodyne_t.h"
#include "lcmtypes_pose_t.h"
#include "small_linalg.h"
#include "velodyne.h"

using namespace but_velodyne;
using namespace velodyne_pointcloud;
using namespace std;

#ifndef VISUALIZATION
#define VISUALIZATION 0
#endif

void pose_to_eigen(double *array_pose, Eigen::Matrix4f &eigen_pose) {
	for (int i = 0; i < 16; i++) {
		eigen_pose(i / 4, i % 4) = array_pose[i];
	}
}

float compute_median_hor_angle(const VelodynePointCloud &cloud) {
	vector<float> angles;
	for (VelodynePointCloud::const_iterator pt = cloud.begin(); pt < cloud.end();
			pt++) {
		angles.push_back(VelodynePointCloud::horizontalAngle(pt->z, pt->x));
	}
	sort(angles.begin(), angles.end());
	return angles[angles.size() / 2];
}

void printPose(const Eigen::Matrix4f &t) {
	for (int i = 0; i < 12; i++) {
		cout << t(i / 4, i % 4) << " ";
	}
	cout << endl;
}

int main(int argc, char **argv) {
	if (argc != 3) {
		fprintf(stderr, "usage: example4-velodyne <logfile> <output-dir>\n");
		return 1;
	}

	lcm_eventlog_t *log = lcm_eventlog_create(argv[1], "r");
	if (!log) {
		fprintf(stderr, "error opening log file\n");
		return 1;
	}

	Config *config = config_parse_default();
	if (!config) {
		fprintf(stderr, "couldn't find config file\n");
		return 1;
	}

	string out_dir = argv[2];

	// load the velodyne sensor calibration
	velodyne_calib_t *vcalib = velodyne_calib_create();

	// read the first timestamp of the log file
	lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(log);
	int64_t first_timestamp = event->timestamp;

	lcmtypes_pose_t last_pose;
	memset(&last_pose, 0, sizeof(last_pose));

#if VISUALIZATION != 0
	Visualizer3D vis;
	VelodynePointCloud vis_cloud;
#endif
	double velodyne_to_local[] =
			{ 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1 };
	VelodynePointCloud sum_cloud;
	Eigen::Matrix4f first_seq_pose, first_cloud_pose;
	bool first_seq_pose_set = false;
	float last_med_angle = 360;
	int cloud_index = -1;
	int64_t last_pose_timestamp;
	bool pose_was_observed = false;
	while (1) {
		// release the last event
		lcm_eventlog_free_event(event);

		// read an event
		event = lcm_eventlog_read_next_event(log);

		if (!event)
			break;

		// always keep track of the current pose
		if (!strcmp(event->channel, "POSE")) {
			if (last_pose.utime)
				lcmtypes_pose_t_decode_cleanup(&last_pose);
			last_pose_timestamp = event->timestamp;

			lcmtypes_pose_t_decode(event->data, 0, event->datalen, &last_pose);
			// compute the velodyne-to-local transformation matrix
			//
			// This is an approximation because we're using the last recorded
			// pose.  A more accurate projection might be to project the
			// vehicle's pose forward based on its last measured velocity.
			config_util_sensor_to_local_with_pose(config, "VELODYNE",
					velodyne_to_local, &last_pose);
			pose_was_observed = true;
		}

		if (!strcmp(event->channel, "VELODYNE")) {
			if (!pose_was_observed) {
				continue;
			}

			Eigen::Matrix4f global_pose;
			pose_to_eigen(velodyne_to_local, global_pose);

			if (!first_seq_pose_set) {
				first_cloud_pose = first_seq_pose = global_pose;
				first_seq_pose_set = true;
			}

			// parse the LCM packet into a velodyne data packet.
			lcmtypes_velodyne_t vel;
			lcmtypes_velodyne_t_decode(event->data, 0, event->datalen, &vel);

			// parse the velodyne data packet
			velodyne_decoder_t vdecoder;
			velodyne_decoder_init(vcalib, &vdecoder, vel.data, vel.datalen);

			// project each sample in the velodyne data packet into the local
			// frame
			velodyne_sample_t vsample;
			VelodynePointCloud cloud;
			while (!velodyne_decoder_next(vcalib, &vdecoder, &vsample)) {
				if (vsample.range < 0.01) {
					continue;
				}

				PointXYZIR point;
				point.x = vsample.xyz[0];
				point.y = vsample.xyz[1];
				point.z = vsample.xyz[2];
				point.intensity = vsample.intensity;
				point.ring = VelodyneSpecification::RINGS - 1 - vsample.logical;
				cloud.push_back(point);
			}

			cloud.setImageLikeAxisFromDarpa();
			Eigen::Matrix4f from_darpa_axis = cloud.getAxisCorrection();
			float med_angle = compute_median_hor_angle(cloud);

			if (med_angle - last_med_angle > 180) {
				if (cloud_index >= 0) {
					pcl::io::savePCDFileBinary(out_dir + "/" + KittiUtils::getKittiFrameName(cloud_index, ".pcd"), sum_cloud);
					Eigen::Matrix4f pose = from_darpa_axis.inverse().eval()
							* (first_seq_pose.inverse().eval() * first_cloud_pose)
							* from_darpa_axis;
					printPose(pose);
#if VISUALIZATION != 0
					if(cloud_index%10 == 0) {
						VelodynePointCloud tmp_cloud;
						pcl::transformPointCloud(sum_cloud, tmp_cloud, pose);
						vis_cloud += tmp_cloud;
						vis.keepOnlyClouds(0).addCloudColoredByRing(vis_cloud).show();
					}
#endif
				}
				sum_cloud.clear();
				cloud_index++;
				first_cloud_pose = global_pose;
				cerr << "Cloud " << cloud_index << " at " << event->timestamp
						<< " transformed by pose acquired at " << last_pose_timestamp << endl;
			}
			pcl::transformPointCloud(cloud, cloud,
					from_darpa_axis.inverse().eval() * first_cloud_pose.inverse().eval() * global_pose * from_darpa_axis);
			sum_cloud += cloud;
			last_med_angle = med_angle;

			lcmtypes_velodyne_t_decode_cleanup(&vel);
		}
	}
	// release NULL ?? lcm_eventlog_free_event(event);

	lcm_eventlog_destroy(log);

	return 0;
}
