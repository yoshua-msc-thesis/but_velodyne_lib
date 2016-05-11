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
                     string &proto_file, string &model_file,
                     vector<string> &files_to_process) {
  po::options_description desc("Ground detection by pretrained CNN.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("prototxt", po::value<string>(&proto_file)->required(),
          "File of net prototype")
      ("caffemodel", po::value<string>(&model_file)->required(),
          "Pretrained caffe model")
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

  return true;
}

int main (int argc, char **argv) {
  vector<string> files_to_process;
  string modelTxt, modelBin;
  if(!parse_arguments(argc, argv, modelTxt, modelBin, files_to_process)) {
    return EXIT_FAILURE;
  }

  Ptr<dnn::Importer> importer;
  try {
    importer = dnn::createCaffeImporter (modelTxt, modelBin);
  } catch (const cv::Exception &err) {
    std::cerr << err.msg << std::endl;
  }
  if (!importer) {
    std::cerr << "Can't load network by using the following files: " << std::endl;
    std::cerr << "prototxt:   " << modelTxt << std::endl;
    std::cerr << "caffemodel: " << modelBin << std::endl;
    return EXIT_FAILURE;
  }

  dnn::Net net;
  importer->populateNet (net);
  importer.release ();                     //We don't need importer anymore

  Visualizer3D visualizer;
  for(vector<string>::iterator featFile = files_to_process.begin(); featFile < files_to_process.end(); featFile++) {
    VelodynePointCloud cloud;
    VelodynePointCloud::fromFile(*featFile, cloud);

    GroundDetectionDataGenerator data_generator(cloud);
    vector<Mat> feature_channels(3);
    feature_channels[0] = data_generator.getMatrixOf(GroundDetectionDataGenerator::RANGE);
    feature_channels[1] = data_generator.getMatrixOf(GroundDetectionDataGenerator::Y);
    feature_channels[2] = data_generator.getMatrixOf(GroundDetectionDataGenerator::INTENSITY);

    Stopwatch stopwatch;
    Mat features;
    merge(feature_channels, features);
    dnn::Blob inputBlob = dnn::Blob (features);  //Convert Mat to dnn::Blob image batch
    net.setBlob (".data", inputBlob);        //set the network input
    net.forward ();                          //compute output
    dnn::Blob probBlob = net.getBlob("prob");   //gather output of "prob" layer
    cerr << "forward pass took: " << stopwatch.elapsed() << "s" << endl;

    Mat probNotGround = probBlob.getPlane(0, 0);
    Mat probIsGround = probBlob.getPlane(0, 1);
    Mat ones = Mat::ones(probIsGround.size(), CV_32F);

    Mat probGround = (ones + probIsGround - probNotGround) * 0.5;

    const vector<CellId> &indicies = data_generator.getIndices();
    PointCloud<PointXYZRGB>::Ptr colored_cloud(new PointCloud<PointXYZRGB>);
    for(int i = 0; i < cloud.size(); i++) {
      uchar r, g, b;
      CellId index = indicies[i];
      Visualizer3D::colorizeIntensity(probGround.at<float>(index.ring, index.polar), r, g, b);
      PointXYZIR pt = cloud[i];
      PointXYZRGB colored_pt(r, g, b);
      colored_pt.x = pt.x;
      colored_pt.y = pt.y;
      colored_pt.z = pt.z;
      colored_cloud->push_back(colored_pt);
    }
    visualizer.keepOnlyClouds(0).addColorPointCloud(colored_cloud).show();
  }

  return EXIT_SUCCESS;
}
