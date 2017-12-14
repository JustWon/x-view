/**
 * File: streetview.cpp
 * Date: December 2017
 * Author: Abel Gawel
 * Description: DBoW2 on StreetView
 */

#include <iostream>
#include <vector>
#include<fstream>
#include<chrono>
#include<iomanip>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#if CV_MAJOR_VERSION == 2
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#endif

#if CV_MAJOR_VERSION == 3
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#endif

// Eigen
#include <Eigen/Dense>


using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

bool parseVectorOfDoubles(const std::string& input,
                          std::vector<double>* output);
void loadFeatures(vector<vector<cv::Mat > > &features, std::string path);
void loadWaypoints(const std::string path, Eigen::Matrix3Xd* waypoints);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testDatabase(const vector<vector<cv::Mat> > &features_database,
                  const vector<vector<cv::Mat> > &features_query,
                  std::vector<QueryResults>* ret);

// number of training images
const int NIMAGES = 70;
const int NSTEP = 1;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait() {
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main() {
  vector<vector<cv::Mat > > features_db, features_query;
  Eigen::Matrix3Xd waypoints;

  std::string db_path = "/home/johnny/segnet/datasets/streetview/RGB_PNG/forward/";
  std::string query_path = "/home/johnny/segnet/datasets/streetview/RGB_PNG/backward/";
  std::string waypoint_path = "/home/johnny/segnet/datasets/streetview/";
  std::string out_path = "/tmp/";
  //Load database features (Forward run)
  loadFeatures(features_db, db_path);
  // Load query features (Backward run)
  loadFeatures(features_query, query_path);
  // Load the waypoints.
  loadWaypoints(waypoint_path, &waypoints);

  wait();
  std::vector<QueryResults> ret;
  testDatabase(features_db, features_query, &ret);

  // Save all results to file.
  Eigen::Matrix<double, NIMAGES, 8> results;
  for (int i = 0; i < NIMAGES; ++i) {
    results(i, 0) = waypoints(0, i);
    results(i, 1) = waypoints(1, i);
    results(i, 2) = waypoints(2, i);
    results(i, 3) = waypoints(0, ret[i][0].Id);
    results(i, 4) = waypoints(1, ret[i][0].Id);
    results(i, 5) = waypoints(2, ret[i][0].Id);
    results(i, 6) = ret[i][0].Score;
    results(i, 7) = 0;
  }

  std::cout << "Writing to file." << std::endl;
  std::ofstream file(out_path + "dbow_streetview.txt");
    if (file.is_open()) {
      file << results;
    }

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features, std::string path) {
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < NIMAGES * NSTEP; i = i + NSTEP) {
    std::cout << "Feature " << i << "/" << NIMAGES * NSTEP << "." << std::endl;
    stringstream ss;
    ss << setfill('0') << setw(3) << i+1;
    std::string filename = path + ss.str() + ".png";

    cv::Mat image = cv::imread(filename, 0);
    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(image, mask, keypoints, descriptors);

    features.push_back(vector<cv::Mat >());
    changeStructure(descriptors, features.back());
  }
}

void loadWaypoints(const std::string path, Eigen::Matrix3Xd* waypoints) {
  waypoints->resize(3, NIMAGES);
  std::string filename_wp = path + "waypoints_forward.txt";

  std::ifstream import_file_wp(filename_wp, std::ios::in);

  std::vector<double> parsed_doubles;

  // Parse camera 0.
  std::string line;
  int line_number = 0;
  if (import_file_wp.is_open()) {
    for (int i = 0; i < NIMAGES * NSTEP; ++i) {
      getline(import_file_wp, line);
      if (i % NSTEP == 0) {
        parseVectorOfDoubles(line, &parsed_doubles);
        (*waypoints)(0, line_number) = parsed_doubles[0];
        (*waypoints)(1, line_number) = parsed_doubles[1];
        (*waypoints)(2, line_number) = parsed_doubles[2];

        line_number++;
      }
    }
    import_file_wp.close();
  }
}

bool parseVectorOfDoubles(const std::string& input,
                          std::vector<double>* output) {
  output->clear();
  // Parse the line as a stringstream for comma-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ',');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception& exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out) {
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i) {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<cv::Mat> > &features_db,
                  const vector<vector<cv::Mat> > &features_query,
                  std::vector<QueryResults>* ret) {
  cout << "Creating a small database..." << endl;

  ret->resize(NIMAGES);
  // load the vocabulary from disk
  OrbVocabulary voc("small_voc.yml.gz");

  OrbDatabase db(voc, false, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < NIMAGES; i++) {
    db.add(features_db[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

//  QueryResults ret;
  for(int i = 0; i < NIMAGES; i++) {
    db.query(features_query[i], (*ret)[i], 4);

    // ret[0] is always the same image in this case, because we added it to the
    // database. ret[1] is the second best match.
  }

  cout << endl;

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  cout << "Saving database..." << endl;
  db.save("small_db.yml.gz");
  cout << "... done!" << endl;

  // once saved, we can load it again
  cout << "Retrieving database once again..." << endl;
  OrbDatabase db2("small_db.yml.gz");
  cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------


