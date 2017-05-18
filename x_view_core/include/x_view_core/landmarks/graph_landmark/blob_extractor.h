#ifndef X_VIEW_BLOB_EXTRACTOR_H
#define X_VIEW_BLOB_EXTRACTOR_H

#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>
#include <x_view_core/landmarks/graph_landmark/blob.h>

#include <opencv2/core/core.hpp>

#include <opencvblobslib/blob.h>

#include <vector>

namespace x_view {

/**
* \brief This class provides methods to extract blobs from an image.
*/
class BlobExtractor {

 public:

  /**
   * \brief Computes the blobs from the image passed as argument and
   * generates a ImageBlobs datastructure containing all the detected blobs.
   * \param image Image to be analyzed for blob detection.
   * \param dilate_and_erode boolean specifying if the input image must be
   * dilated and erosed prior to blob extraction.
   * \param min_blob_size Blobs containing a number of pixels smaller than
   * this argument are not added to the returned ImageBlobs datastructure.
   * \return ImageBlobs datastructure containing all detected blobs.
   */
  static ImageBlobs findBlobsWithContour(const cv::Mat& image,
                                         bool dilate_and_erode,
                                         int min_blob_size);

};

}

#endif //X_VIEW_BLOB_EXTRACTOR_H
