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

const int HISTORY_SIZE = 5;
const int BATCH_SIZE = 5;
const int JOINED_FRAMES = 3;
const int FEATURES = 3;

bool parse_arguments(int argc, char **argv,
                     string &proto_file, string &model_file,
		     bool &visualize, ofstream &out_file,
                     vector<string> &files_to_process) {
  string out_filename;
  po::options_description desc("Odometry estimation by pretrained CNN.\n"
      "======================================\n"
//      " * Reference(s): Velas et al, ???? 201?\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("prototxt,p", po::value<string>(&proto_file)->required(),
          "File of net prototype")
      ("caffemodel,m", po::value<string>(&model_file)->required(),
          "Pretrained caffe model")
      ("visualize,v", po::bool_switch(&visualize),
	  "Run visualization")
      ("output,o", po::value<string>(&out_filename)->default_value(""),
	  "Output file of ground probabilities")
  ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line (argc, argv, desc);
  po::store (parsed, vm);
  files_to_process = po::collect_unrecognized (parsed.options,
						po::include_positional);

  if (vm.count ("help") || files_to_process.size () < HISTORY_SIZE+JOINED_FRAMES-1) {
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
  string modelTxt, modelBin;
  bool visualization;
  ofstream out_file;
  if(!parse_arguments(argc, argv, modelTxt, modelBin, visualization, out_file, files_to_process)) {
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

  boost::shared_ptr<Visualizer3D> visualizer;
  if(visualization) {
    visualizer.reset(new Visualizer3D);
  }

  for(vector<string>::iterator firstCloud = files_to_process.begin(); firstCloud+(HISTORY_SIZE+JOINED_FRAMES-2) < files_to_process.end(); firstCloud++) {

    GroundDetectionDataGenerator::Parameters data_generator_default_params;
    vector<Mat> feature_channels(FEATURES*JOINED_FRAMES*BATCH_SIZE*HISTORY_SIZE,
				 Mat::zeros(VelodyneSpecification::RINGS, data_generator_default_params.polar_bins, CV_32FC1));

    for(int cloud_i = 0; cloud_i < HISTORY_SIZE+JOINED_FRAMES-1; cloud_i++) {
      VelodynePointCloud cloud;
      VelodynePointCloud::fromFile(*(firstCloud+cloud_i), cloud);
      GroundDetectionDataGenerator data_generator(cloud);
      vector<Mat> feature(FEATURES);
      feature[0] = data_generator.getMatrixOf(GroundDetectionDataGenerator::RANGE);
      feature[1] = data_generator.getMatrixOf(GroundDetectionDataGenerator::Y);
      feature[2] = data_generator.getMatrixOf(GroundDetectionDataGenerator::INTENSITY);

      for(int fi = 0; fi < JOINED_FRAMES; fi++) {
	int history_i = cloud_i-fi;
	int slot_i = JOINED_FRAMES-1-fi;
	if(0 <= history_i && history_i < HISTORY_SIZE && 0 <= slot_i && slot_i < JOINED_FRAMES) {
	  int i = FEATURES*JOINED_FRAMES*BATCH_SIZE*history_i;
	  i += FEATURES*slot_i;
	  for(int ch = 0; ch < FEATURES; ch++) {
	    feature_channels[i+ch] = feature[ch];
	    /*cerr << "cloud_i: " << cloud_i << ", fi: " << fi << ", history_i: " << history_i
		<< ", slot_i: " << slot_i << ", ch: " << ch << ", i+ch: " << i+ch << endl;*/
	  }
	}
      }
    }

    Stopwatch stopwatch;
    Mat features;
    merge(feature_channels, features);
    dnn::Blob inputBlob = dnn::Blob::fromImages(features, FEATURES*JOINED_FRAMES);  //Convert Mat to dnn::Blob image batch
    net.setBlob (".data", inputBlob);        //set the network input
    net.forward ();                          //compute output
    dnn::Blob probBlob = net.getBlob("full_conv");   //gather output of "prob" layer
    cerr << "forward pass took: " << stopwatch.elapsed() << "s" << endl;

    cerr << "dims: [" << probBlob.num() << ", " << probBlob.channels() << ", " << probBlob.rows() << ", " << probBlob.cols() << "]" << endl;
    for(int n = 0; n < probBlob.num(); n++) {
      cout << n << ":[";
      for(int ch = 0; ch < probBlob.channels(); ch++) {
	Mat pose = probBlob.getPlane(n, ch);
	cout << pose.at<float>(0) << ", ";
      }
      cout << "]" << endl << flush;
    }

    // TODO output odometry

    if(visualization) {
      // TODO visualize
    }
  }

  return EXIT_SUCCESS;
}
