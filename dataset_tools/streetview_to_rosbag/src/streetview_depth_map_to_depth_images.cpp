#include <iostream>
#include <vector>
#include <cstring>
#include <queue>
#include <dirent.h>
#include <sys/stat.h>
#include <regex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

//  Adapted from https://github.com/patriciogonzalezvivo/ofxStreetView
//  Created by Patricio Gonzalez Vivo on 3/10/14.
//  Adapted for X-View by Abel Gawel on 05/12/17.

#include "streetview_to_rosbag/base64.h"
#include <zlib.h>

struct DepthMapPlane
{
  float x, y, z;
  float d;
};

struct Link
{
  float yaw_deg;
  std::string pano_id;
};

bool numeric_string_compare(const std::string& s1,
                            const std::string& s2) {
  std::string::const_iterator it1 = s1.begin(), it2 = s2.begin();
  std::string::const_iterator rit1 = s1.end(), rit2 = s2.end();
  // Find beginning of number.
  --rit1;
  --rit2;
  while (!std::isdigit(*rit1)) {
    --rit1;
  }
  it1 = rit1;
  while (std::isdigit(*it1)) {
    --it1;
  }
  ++it1;
  while (!std::isdigit(*rit2)) {
    --rit2;
  }
  it2 = rit2;
  while (std::isdigit(*it2)) {
    --it2;
  }
  ++it2;

  if (rit1 - it1 == rit2 - it2) {
    while (it1 != rit1 && it2 != rit2) {
      if (*it1 > *it2) {
        return true;
      } else if (*it1 < *it2) {
        return false;
      }
      ++it1;
      ++it2;
    }
    if (*it1 > *it2) {
      return true;
    } else {
      return false;
    }
  } else if (rit1 - it1 > rit2 - it2) {
    return true;
  } else {
    return false;
  }
}

int getFileLists(const std::string& initial_path,
                 bool sortlexical, const std::string& extension_filter,
                 std::vector<std::string>* imagepaths) {
  struct dirent* dir;

  std::vector<std::string> sub_directories;
  sub_directories.push_back(initial_path);

  std::queue<std::string> directories_to_explore;
  directories_to_explore.push(initial_path);

  while (!directories_to_explore.empty()) {
    std::string path = directories_to_explore.front();
    directories_to_explore.pop();
    DIR* d = opendir(path.c_str());
    if (d == NULL) {
      closedir(d);
      return 1;
    }
    while ((dir = readdir(d))) {  // NOLINT
      if (strcmp(dir->d_name, ".") == 0 || strcmp(dir->d_name, "..") == 0) {
        continue;
      }
      struct stat st;
      char filename[512];
      snprintf(filename, sizeof(filename), "%s/%s", path.c_str(),
               dir->d_name);
      lstat(filename, &st);
      if (S_ISDIR(st.st_mode)) {
        const std::string subfolder = path + "/" + dir->d_name;
        directories_to_explore.push(subfolder);
        sub_directories.push_back(subfolder);
      }
    }
    closedir(d);
  }

  for (const std::string& folder : sub_directories) {
    struct dirent* subdir;
    DIR* d = opendir(folder.c_str());
    if (d == NULL) {
      closedir(d);
      return 1;
    }

    while ((subdir = readdir(d))) {  // NOLINT
      if (strcmp(subdir->d_name, ".") == 0 ||
          strcmp(subdir->d_name, "..") == 0) {
        continue;
      }
      if (!std::regex_match(subdir->d_name,
                            std::regex("(.*)(" + extension_filter + ")"))) {
        continue;
      }
      if (subdir == NULL) {
        break;
      }

      imagepaths->push_back(folder + "/" + subdir->d_name);
    }
    closedir(d);
  }
  if (sortlexical) {
    sort(imagepaths->begin(), imagepaths->end());  // Normal lexical sort.
  } else {
    sort(imagepaths->begin(), imagepaths->end(),
         // Sorts strictly by the number in the file name
         std::bind(&numeric_string_compare, std::placeholders::_2,
                   std::placeholders::_1));
  }
  return 0;
}

int main(int argc, char* args[])
{
  try {
    xercesc_3_2::XMLPlatformUtils::Initialize();
  } catch (const xercesc_3_2::XMLException& toCatch) {
    char* message = xercesc_3_2::XMLString::transcode(toCatch.getMessage());
    std::cout << "Error during initialization! :\n" << message << "\n";
    xercesc_3_2::XMLString::release(&message);
    return 1;
  }
  xercesc_3_2::XercesDOMParser* parser = new xercesc_3_2::XercesDOMParser();
  parser->setValidationScheme(xercesc_3_2::XercesDOMParser::Val_Always);
  parser->setDoNamespaces(true);  // optional

  xercesc_3_2::ErrorHandler* errHandler =
      (xercesc_3_2::ErrorHandler*) new xercesc_3_2::HandlerBase();
  parser->setErrorHandler(errHandler);

  std::vector<std::string> filenames;

  getFileLists("/home/johnny/segnet/datasets/streetview/Depth/XML/", true, "xml",
               &filenames);

  std::vector<unsigned char> depthmapIndices;
  std::vector<DepthMapPlane> depthmapPlanes;

  for (size_t i = 0u; i < filenames.size(); ++i) {

    const char * xmlFile = filenames[i].c_str();
    try {
      parser->parse(xmlFile);
    } catch (const xercesc_3_2::XMLException& toCatch) {
      char* message = xercesc_3_2::XMLString::transcode(toCatch.getMessage());
      std::cout << "Exception message is: \n" << message << "\n";
      xercesc_3_2::XMLString::release(&message);
      return -1;
    } catch (const xercesc_3_2::DOMException& toCatch) {
      char* message = xercesc_3_2::XMLString::transcode(toCatch.msg);
      std::cout << "Exception message is: \n" << message << "\n";
      xercesc_3_2::XMLString::release(&message);
      return -1;
    } catch (...) {
      std::cout << "Unexpected Exception \n";
      return -1;
    }

    char* m_OptionA;
    char* m_OptionB;
    XMLCh* TAG_root = xercesc_3_2::XMLString::transcode("panorama");
    XMLCh* TAG_model = xercesc_3_2::XMLString::transcode("model");
    XMLCh* TAG_depth_map = xercesc_3_2::XMLString::transcode("depth_map");
    XMLCh* ATTR_OptionA = xercesc_3_2::XMLString::transcode("image_width");
    XMLCh* ATTR_OptionB = xercesc_3_2::XMLString::transcode("region");

    xercesc_3_2::DOMDocument* xmlDoc = parser->getDocument();
    xercesc_3_2::DOMElement* elementRoot = xmlDoc->getDocumentElement();
    xercesc_3_2::DOMNodeList* children = elementRoot->getChildNodes();
    const XMLSize_t nodeCount = children->getLength();

    std::cout << "node count " << nodeCount << std::endl;
    for (XMLSize_t xx = 0; xx < nodeCount; ++xx) {
      xercesc_3_2::DOMNode* currentNode = children->item(xx);
      if (currentNode->getNodeType() &&  // true is not NULL
          currentNode->getNodeType() == xercesc_3_2::DOMNode::ELEMENT_NODE)  // is element
      {
        // Found node which is an Element. Re-cast node as element
        xercesc_3_2::DOMElement* currentElement =
            dynamic_cast<xercesc_3_2::DOMElement*>(currentNode);
        std::cout << "current tag: "
            << xercesc_3_2::XMLString::transcode(currentElement->getTagName())
        << std::endl;

        // Get depth data.
        if (xercesc_3_2::XMLString::equals(currentElement->getTagName(),
                                           TAG_model)) {
          xercesc_3_2::DOMNodeList* model_children =
              currentElement->getChildNodes();
          xercesc_3_2::DOMNode* depth_node = model_children->item(0);
          xercesc_3_2::DOMElement* depth_element =
              dynamic_cast<xercesc_3_2::DOMElement*>(depth_node);

          std::cout << "should be depth tag: "
              << xercesc_3_2::XMLString::transcode(depth_element->getTagName())
          << std::endl;

          std::string depth_map_base64;
          char* depth_map_char = xercesc_3_2::XMLString::transcode(
              depth_element->getTextContent());
          depth_map_base64 = depth_map_char;
          xercesc_3_2::XMLString::release(&depth_map_char);

          int mapWidth, mapHeight, maxDistance;
          mapWidth = 512;
          mapHeight = 256;
          maxDistance = 200;
          std::vector<Link> links;

          if (depth_map_base64 != "") {
            //Decode base64
            std::vector<unsigned char> depth_map_compressed(
                depth_map_base64.length());
            int compressed_length = decode_base64(&depth_map_compressed[0],
                                                  &depth_map_base64[0]);
            std::cout << "compressed_length " << compressed_length << std::endl;

            //Uncompress data with zlib
            //TODO: decompress in a loop so we can accept any size
            unsigned long length = 512 * 256 + 5000;
            std::vector<unsigned char> depth_map(length);
            int zlib_return = uncompress(&depth_map[0], &length,
                                         &depth_map_compressed[0],
                                         compressed_length);
            if (zlib_return != Z_OK)
              throw "zlib decompression of the depth map failed";

            //Load standard data
            const int headersize = depth_map[0];
            const int numPanos = depth_map[1] | (depth_map[2] << 8);
            mapWidth = depth_map[3] | (depth_map[4] << 8);
            mapHeight = depth_map[5] | (depth_map[6] << 8);
            const int panoIndicesOffset = depth_map[7];

            if (headersize != 8 || panoIndicesOffset != 8)
              throw "Unexpected depth map header";

            //Load depthMapIndices
            depthmapIndices = std::vector<unsigned char>(mapHeight * mapWidth);
            std::memcpy(&depthmapIndices[0], &depth_map[panoIndicesOffset],
                        mapHeight * mapWidth);

            //Load depthMapPlanes
            depthmapPlanes = std::vector < DepthMapPlane > (numPanos);
            std::memcpy(&depthmapPlanes[0],
                        &depth_map[panoIndicesOffset + mapHeight * mapWidth],
                        numPanos * sizeof(struct DepthMapPlane));
            std::cout << "planes " << depthmapPlanes.size() << std::endl;
            std::cout << "indices " << depthmapIndices.size() << std::endl;
          }

          cv::Mat out_image_forward(128, 128, CV_8UC1, 255);
          cv::Mat out_image_backward(128, 128, CV_8UC1, 255);

          // Convert depth map into a grayscale image.
          cv::Mat current_image(256, 512, CV_8UC1, 255);

          for (int x = 0; x < mapWidth; x++) {
            for (int y = 0; y < mapHeight; y++) {
              int planeId = depthmapIndices[y * mapWidth + x];
              if (planeId > 0) {
                double rad_azimuth = x / (double) (mapWidth - 1.0f) * 2 * M_PI;
                double rad_elevation = y / (double) (mapHeight - 1.0f) * M_PI;

                double xc = sin(rad_elevation) * sin(rad_azimuth);
                double yc = sin(rad_elevation) * cos(rad_azimuth);
                double zc = cos(rad_elevation);

                DepthMapPlane plane = depthmapPlanes[planeId];
                float dist = std::abs(
                    plane.d / (xc * plane.x + yc * plane.y + zc * plane.z));
                if (dist >= 100.0) {
                  current_image.at < uchar > (y, x) = 255u;
                } else {
                  current_image.at < uchar > (y, x) = dist * 255.0 / 100.0;
                }
              }
            }
          }

          cv::Rect myROI_forward(192, 64, 128, 128);
          cv::Rect myROI_backward_right(0, 64, 64, 128);
          cv::Rect myROI_backward_left(448, 64, 64, 128);
          cv::Mat back_right(64, 128, CV_8UC1, 255);
          cv::Mat back_left(64, 128, CV_8UC1, 255);
          back_left = current_image(myROI_backward_left);
          back_right = current_image(myROI_backward_right);
          cv::hconcat(back_left, back_right, out_image_backward);
          out_image_forward = current_image(myROI_forward);

          cv::Mat resized_image_forward(480, 480, CV_8UC1, 255);
          cv::Mat resized_image_backward(480, 480, CV_8UC1, 255);
          cv::resize(out_image_forward, resized_image_forward, cv::Size(), 3.75, 3.75);
          cv::resize(out_image_backward, resized_image_backward, cv::Size(), 3.75, 3.75);

          // Crop image.
          cv::Rect myROI(0, 60, 480, 360);
          cv::Rect myROI2(0, 60, 480, 360);

          // Crop the full image to that image contained by the rectangle myROI
          // Note that this doesn't copy the data
          cv::Mat cropped_image_forward = resized_image_forward(myROI);
          cv::Mat cropped_image_backward = resized_image_backward(myROI2);

          std::string path_out = "/home/johnny/segnet/datasets/streetview/Depth/";

          std::cout << filenames[i] << std::endl;

          char buffer[20];
          sprintf(buffer, "%03d", int(i + 1));

          cv::imwrite( path_out + "forward/" + std::string(buffer) + ".png", cropped_image_forward);
          cv::imwrite( path_out + "backward/" + std::string(buffer) + ".png", cropped_image_backward);

        }
      }
    }
  }

  std::cout << ": parse OK" << std::endl;

  delete parser;
  delete errHandler;
  return 0;
}
