#include <fstream>
#include <iostream>
#include <cstdlib>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/program_options.hpp>

#include <but_velodyne/Stopwatch.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/GroundDetectionDataGenerator.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace but_velodyne;
namespace po = boost::program_options;

bool parse_arguments(int argc, char **argv,
                     ofstream &out_file,
                     vector<string> &files_to_process) {
  string out_filename;
  po::options_description desc("Ground detection by pretrained CNN.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("output,o", po::value<string>(&out_filename)->default_value(""),
	  "Output file of ground probabilities")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line (argc, argv, desc);
  po::store (parsed, vm);
  files_to_process = po::collect_unrecognized (parsed.options,
						po::include_positional);

  if (vm.count ("help") || files_to_process.size () < 1) {
    std::cerr << desc << std::endl;
    return false;
  }

  try {
    po::notify (vm);
  }
  catch (std::exception& e) {
    std::cerr << "Error: " << e.what () << std::endl << std::endl << desc
	<< std::endl;
    return false;
  }

  if(!out_filename.empty()) {
    out_file.open(out_filename.c_str());
    if(!out_file.is_open()) {
      perror(out_filename.c_str());
      return false;
    }
  }

  return true;
}

int main (int argc, char **argv) {
  vector<string> files_to_process;
  ofstream out_file;
  if(!parse_arguments(argc, argv, out_file, files_to_process)) {
    return EXIT_FAILURE;
  }

  for(vector<string>::iterator featFile = files_to_process.begin(); featFile < files_to_process.end(); featFile++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(*featFile, cloud);

    GroundDetectionDataGenerator data_generator(cloud);

    FileStorage fs("/home/ivelas/workspace/but_velodyne_lib/scripts/histogram.yaml", FileStorage::READ);
    Mat probGround;
    fs["histogram"] >> probGround;

    const vector<CellId> &indicies = data_generator.getIndices();
    PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
    for(int i = 0; i < cloud.size(); i++) {
      uchar r, g, b;
      CellId index = indicies[i];
      float prob = probGround.at<float>(index.ring, index.polar);
      Visualizer3D::colorizeIntensity(prob, r, g, b);
      VelodynePoint pt = cloud[i];
      PointXYZRGB colored_pt(r, g, b);
      colored_pt.x = pt.x;
      colored_pt.y = pt.y;
      colored_pt.z = pt.z;
      colored_cloud->push_back(colored_pt);
      if(out_file.is_open()) {
	out_file << prob << endl;
      }
    }
  }

  return EXIT_SUCCESS;
}
