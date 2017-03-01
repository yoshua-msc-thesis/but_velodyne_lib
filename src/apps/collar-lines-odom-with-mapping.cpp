/*
 * Odometry estimation by collar line segments of Velodyne scan.
 *
 * Published in:
 * 	Velas, M. Spanel, M. Herout, A.: Collar Line Segments for
 * 	Fast Odometry Estimation from Velodyne Point Clouds, ICRA 2016
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 21/04/2015
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

#include <boost/program_options.hpp>
#include <boost/circular_buffer.hpp>

#include <cv.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PoseGraphEdge.h>
#include <but_velodyne/CollarLinesRegistrationPipeline.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

class CollarLinesRegistrationPipelineWithMapping {
public:
  CollarLinesRegistrationPipeline::Parameters params;
  CollarLinesRegistration::Parameters registration_params;

  CollarLinesRegistrationPipelineWithMapping(MoveEstimator &estimator, ostream &graph_file,
      CollarLinesRegistrationPipeline::Parameters pipeline_params_, CollarLinesRegistration::Parameters registration_params_) :
    estimation(estimator), filter(pipeline_params_.linesPerCellPreserved), cumulated_transformation(Eigen::Matrix4f::Identity()),
    pose_index(0), graph_file(graph_file), history(pipeline_params_.historySize),
    params(pipeline_params_), registration_params(registration_params_) {
  }

  Eigen::Matrix4f runRegistration(const VelodynePointCloud &target_cloud) {
    PolarGridOfClouds target_polar_grid(target_cloud);
    LineCloud::Ptr target_line_cloud(new LineCloud(target_polar_grid, params.linesPerCellGenerated, filter));

    Eigen::Matrix4f transformation;
    if(!history.empty()) {
      LineCloud::Ptr current_map = buildMap();
      transformation = registerLineClouds(*current_map, *target_line_cloud, getPrediction());
      estimation.addMeassurement(transformation);
    } else {
      transformation = Eigen::Matrix4f::Identity();
    }
    updateHistory(target_line_cloud, transformation);

    PoseGraphEdge edge(pose_index, ++pose_index, transformation);
    graph_file << edge << endl;

    return transformation;
  }

  void output(const Eigen::Matrix4f &transformation) {
    cumulated_transformation = cumulated_transformation * transformation;
    KittiUtils::printPose(std::cout, cumulated_transformation);
  }

protected:
  void updateHistory(const LineCloud::Ptr target_polar_grid, Eigen::Matrix4f transformation) {
    for(int i = 0; i < history.size(); i++) {
      history[i].update(transformation);
    }
    history.push_front(HistoryRecord<LineCloud>(target_polar_grid, pose_index));
  }

  Eigen::Matrix4f getPrediction() {
    Eigen::Matrix4f prediction = estimation.predict();
    cerr << "Prediction:" << std::endl << prediction << std::endl << std::endl;
    return prediction;
  }

  LineCloud::Ptr buildMap(void) {
    LineCloud::Ptr current_map(new LineCloud);
    for(int i = 0; i < history.size(); i++) {
      *current_map += *(history[i].getGridCloud());
    }
    return current_map;
  }

  Eigen::Matrix4f registerLineClouds(const LineCloud &source,
                                     const LineCloud &target,
                                     const Eigen::Matrix4f &initial_transformation) {
    Termination termination(params.minIterations, params.maxIterations, params.maxTimeSpent,
                            params.significantErrorDeviation, params.targetError);
    Eigen::Matrix4f transformation = initial_transformation;
    while(!termination()) {
      CollarLinesRegistration icl_fitting(source, target,
                                          registration_params, transformation);
      float error = icl_fitting.refine();
      termination.addNewError(error);
      transformation = icl_fitting.getTransformation();
    }
    return transformation;
  }

private:
  MoveEstimation estimation;
  CollarLinesFilter filter;
  Eigen::Matrix4f cumulated_transformation;
  Stopwatch stopwatch;

  boost::circular_buffer< HistoryRecord<LineCloud> > history;

  int pose_index;
  ostream &graph_file;
};


bool parse_arguments(int argc, char **argv,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     boost::shared_ptr<MoveEstimator> &estimator,
                     vector<string> &clouds_to_process);

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

	CollarLinesRegistration::Parameters registration_parameters;
	CollarLinesRegistrationPipeline::Parameters pipeline_parameters;
	vector<string> clouds_to_process;
        boost::shared_ptr<MoveEstimator> estimator;

	if(!parse_arguments(argc, argv, registration_parameters, pipeline_parameters, estimator, clouds_to_process)) {
	  return EXIT_FAILURE;
	}

	boost::filesystem::path first_cloud(clouds_to_process.front());
    string output_path;
    if (first_cloud.has_parent_path()) {
        output_path = first_cloud.parent_path().string();
    } else {
        output_path =  boost::filesystem::current_path().string();
    }
    string graph_filename = output_path + "/poses.graph";
	ofstream graph_file(graph_filename.c_str());
	if (!graph_file.is_open()) {
		perror(graph_filename.c_str());
		exit(1);
	}

	CollarLinesRegistrationPipelineWithMapping registration(
			*estimator, graph_file,
			pipeline_parameters, registration_parameters);

	VelodynePointCloud target_cloud;
	for (int i = 0; i < clouds_to_process.size(); i++) {

		string filename = clouds_to_process[i];
		BUT_VELODYNE_LOG << "KITTI file: " << filename << endl << flush;
		if (filename.find(".pcd") != string::npos) {
			io::loadPCDFile(filename, target_cloud);
		} else {
			VelodynePointCloud::fromKitti(filename, target_cloud);
		}

		Eigen::Matrix4f t = registration.runRegistration(target_cloud);
		EigenUtils::saveMatrix(filename + string(".transform"), t);
		registration.output(t);
	}

	return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv,
                     CollarLinesRegistration::Parameters &registration_parameters,
                     CollarLinesRegistrationPipeline::Parameters &pipeline_parameters,
                     boost::shared_ptr<MoveEstimator> &estimator,
                     vector<string> &clouds_to_process) {
  bool use_kalman;
  int linear_estimator;
  string init_poses;

  po::options_description desc("Collar Lines Registration of Velodyne scans\n"
      "======================================\n"
      " * Reference(s): Velas et al, ICRA 2016\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("matching_threshold", po::value<CollarLinesRegistration::Threshold>(&registration_parameters.distance_threshold)->default_value(registration_parameters.distance_threshold),
          "How the value of line matching threshold is estimated (mean/median/... of line pairs distance). Possible values: MEDIAN_THRESHOLD|MEAN_THRESHOLD|NO_THRESHOLD")
      ("line_weightning", po::value<CollarLinesRegistration::Weights>(&registration_parameters.weighting)->default_value(registration_parameters.weighting),
          "How the weights are assigned to the line matches - prefer vertical lines, close or treat matches as equal. Possible values: DISTANCE_WEIGHTS|VERTICAL_ANGLE_WEIGHTS|NO_WEIGHTS")
      ("shifts_per_match", po::value<int>(&registration_parameters.correnspPerLineMatch)->default_value(registration_parameters.correnspPerLineMatch),
          "[Experimental] How many shift vectors (for SVD) are generated per line match - each is amended by small noise")
      ("shifts_noise_sigma", po::value<float>(&registration_parameters.lineCorrenspSigma)->default_value(registration_parameters.lineCorrenspSigma),
          "[Experimental] Deviation of noise generated for shift vectors (see above)")
      ("lines_per_bin_generated,g", po::value<int>(&pipeline_parameters.linesPerCellGenerated)->default_value(pipeline_parameters.linesPerCellGenerated),
          "How many collar lines are generated per single polar bin")
      ("lines_per_bin_preserved,p", po::value<int>(&pipeline_parameters.linesPerCellPreserved)->default_value(pipeline_parameters.linesPerCellPreserved),
          "How many collar lines are preserved per single polar bin after filtering")
      ("min_iterations", po::value<int>(&pipeline_parameters.minIterations)->default_value(pipeline_parameters.minIterations),
          "Minimal number of registration iterations (similar to ICP iterations)")
      ("max_iterations", po::value<int>(&pipeline_parameters.maxIterations)->default_value(pipeline_parameters.maxIterations),
          "Maximal number of registration iterations")
      ("max_time_for_registration", po::value<int>(&pipeline_parameters.maxTimeSpent)->default_value(pipeline_parameters.maxTimeSpent),
          "Maximal time for registration [sec]")
      ("target_error", po::value<float>(&pipeline_parameters.targetError)->default_value(pipeline_parameters.targetError),
          "Minimal error (average distance of line matches) causing termination of registration")
      ("significant_error_deviation", po::value<float>(&pipeline_parameters.significantErrorDeviation)->default_value(pipeline_parameters.significantErrorDeviation),
          "If standard deviation of error from last N=min_iterations iterations if below this value - registration is terminated")
      ("history_size,m", po::value<int>(&pipeline_parameters.historySize)->default_value(pipeline_parameters.historySize),
          "How many previous frames are used for registration (multi-view CLS-M approach described in the paper)")
      ("linear_estimator", po::value<int>(&linear_estimator)->default_value(0),
              "Use last N frames for linear odometry prediction - can not be combined with kalman_estimator switch or init_poses_estimator")
      ("init_poses_estimator", po::value<string>(&init_poses)->default_value(""),
              "Use precomputed poses as a prediction - can not be combined with kalman_estimator switch or linear_estimator")
      ("kalman_estimator", po::bool_switch(&use_kalman), "Use Kalman filter instead of linear predictor or precomputed poses for estimation of odometry")
   ;

    po::variables_map vm;
    po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
    po::store(parsed, vm);
    clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

    if (vm.count("help") || clouds_to_process.size() < 1)
    {
        std::cerr << desc << std::endl;
        return false;
    }

    try
    {
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
        return false;
    }

    if(use_kalman?1:0 + (linear_estimator > 0)?1:0 + init_poses.empty()?0:1 > 1) {
        std::cerr << "Unable to use multiple predictors at the same time!" << std::endl;
        std::cerr << desc << std::endl;
        return false;
    } else if(use_kalman) {
      estimator.reset(new KalmanMoveEstimator(1e-5, 1e-4, 1.0));
    } else if(!init_poses.empty()) {
      estimator.reset(new PosesToInitEstimator(init_poses));
    } else {
      estimator.reset(new LinearMoveEstimator(linear_estimator > 0 ? linear_estimator : 3));
    }

    return true;
}
