/**
 * File: synthia.cpp
 * Date: August 2017
 * Author: Abel Gawel
 * Description: DBoW2 on SYNTHIA
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
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

// Eigen
#include <Eigen/Dense>


using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

bool parseVectorOfDoubles(const std::string& input,
                          std::vector<double>* output);
void testVocCreation(const vector<vector<cv::Mat > > &features);
void loadFeatures(vector<vector<cv::Mat > > &features, std::string path);
void loadWaypoints(const std::string path, Eigen::Matrix3Xd* waypoints);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testDatabase(const vector<vector<cv::Mat> > &features_database,
                  const vector<vector<cv::Mat> > &features_query, std::vector<QueryResults>* ret);

const int NIMAGES = 900;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
  vector<vector<cv::Mat > > features_db, features_query;
  Eigen::Matrix3Xd waypoints;

  std::string db_path = "/media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER/RGB/Stereo_Left/Omni_F/";
  std::string query_path = "/media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER/RGB/Stereo_Right/Omni_F/";
  std::string waypoint_path = "/media/johnny/082e0614-ce4c-45cc-abc5-dbde7ae882bb/SYNTHIA/Video_sequences/SYNTHIA-SEQS-04-SUMMER/CameraParams/Stereo_Left/Omni_F/";
  std::string out_path = "/tmp/";
  //Load database features (Forward run)
  loadFeatures(features_db, db_path);

  testVocCreation(features_db);
  // Load query features (Backward run)
  loadFeatures(features_query, query_path);
  // Load the waypoints.
  loadWaypoints(waypoint_path, &waypoints);

  wait();

  std::vector<QueryResults> ret;
  testDatabase(features_db, features_query, &ret);

  // Save all results to file.a similar
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
  std::ofstream file(out_path + "dbow_synthia_ff.txt");
    if (file.is_open())
    {
      file << results;
    }

  return 0;
}

void testVocCreation(const vector<vector<cv::Mat > > &features)
{
  // branching factor and depth levels
  const int k = 9;
  const int L = 3;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  OrbVocabulary voc(k, L, weight, score);

  cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;
  for(int i = 0; i < NIMAGES; i++)
  {
    voc.transform(features[i], v1);
    for(int j = 0; j < NIMAGES; j++)
    {
      voc.transform(features[j], v2);

      double score = voc.score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    }
  }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features, std::string path)
{
  features.clear();
  features.reserve(NIMAGES);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < NIMAGES; ++i)
  {
    std::cout << "Feature " << i << "/" << NIMAGES << "." << std::endl;
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
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
  for(int i = 0; i < NIMAGES; ++i) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    std::string filename = path + ss.str() + ".txt";

    Eigen::Vector3d waypoint;

    std::vector<double> parsed_doubles;
    std::string line;
    std::ifstream import_file_wp(filename, std::ios::in);
    std::getline(import_file_wp, line);
    parseVectorOfDoubles(line, &parsed_doubles);
    (*waypoints)(0,i) = parsed_doubles[13];
    (*waypoints)(1,i) = parsed_doubles[14];
    (*waypoints)(2,i) = parsed_doubles[15];
  }
}

bool parseVectorOfDoubles(const std::string& input,
                          std::vector<double>* output) {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
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

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<cv::Mat > > &features_db, const vector<vector<cv::Mat > > &features_query, std::vector<QueryResults>* ret)
{
  cout << "Creating a small database..." << endl;

  ret->resize(NIMAGES);
  // load the vocabulary from disk
  cout << "Loading Voc" << endl;
  OrbVocabulary voc("small_voc.yml.gz");
  cout << "Voc loaded" << endl;

  OrbDatabase db(voc, false, 0); // false = do not use direct index
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < NIMAGES; i++)
  {
    db.add(features_db[i]);
  }

  cout << "... done!" << endl;

  cout << "Database information: " << endl << db << endl;

  // and query the database
  cout << "Querying the database: " << endl;

  for(int i = 0; i < NIMAGES; i++)
  {
    db.query(features_query[i], (*ret)[i], 4);
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


