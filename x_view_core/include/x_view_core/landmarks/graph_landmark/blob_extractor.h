#ifndef X_VIEW_BLOB_EXTRACTOR_H
#define X_VIEW_BLOB_EXTRACTOR_H

#include <x_view_core/landmarks/graph_landmark/blob.h>
#include <x_view_core/landmarks/graph_landmark/graph_landmark_types.h>

#include <opencv2/core/core.hpp>
#include <opencvblobslib/blob.h>

#include <vector>

#ifdef X_VIEW_DEBUG
#define X_VIEW_NUM_THREADS_BLOB_EXTRACTION 1
#else
#define X_VIEW_NUM_THREADS_BLOB_EXTRACTION 4
#endif

namespace x_view {

/**
 * \brief Parameters used by the blob extractor to extract blobs from the
 * passed image.
 */
struct BlobExtractorParams {

  /**
   * \brief The minimum blob size can be either given as an absolute integer
   * value representing the number of pixels, or by a value relative to the
   * number of pixels in the image.
   */
    enum class MIN_BLOB_SIZE_TYPE {
    RELATIVE_TO_IMAGE_SIZE,
    ABSOLUTE
  };

  /**
   * \brief This struct computes the minimum blob size based on the
   * parameters set in the BlobExtractorParams.
   */
  struct BlobSizeFiltering {

    /**
     * \brief Default constructor will filter out all blobs with a number of
     * pixels smaller than 1% of of the total number of pixels in the image.
     */
    BlobSizeFiltering()
        : type_(MIN_BLOB_SIZE_TYPE::RELATIVE_TO_IMAGE_SIZE),
          n_min_pixels_(0.01) {}

    MIN_BLOB_SIZE_TYPE type_;
    /// \brief If type_ is RELATIVE_TO_IMAGE_SIZE, this represents the
    /// fraction of pixels relative to the total number of pixels in the
    /// image. Otherwise, if type_ is ABSOLUTE, this represents the minimum
    /// number of pixels a blob must have in order to be extracted.
    float n_min_pixels_;

    /**
     * \brief Computes the minimum number of pixels a blob must have in order
     * to be extracted from the image.
     * \param image_size Size of the image being analyzed for blob extraction.
     * \return Minimum number of pixels a blob must have in order to be
     * extracted from the image.
     */
    int minimumBlobSize(const cv::Size& image_size) const;
  };

  /**
   * \brief Default constructor which does not perform any dilation and
   * erosion operation on the image, works with X_VIEW_NUM_THREADS_BLOB_EXTRACTION
   * threads and uses a default blob filtering struct.
   */
  BlobExtractorParams()
      : dilate_and_erode_(false),
        num_dilate_reps_(1),
        num_erode_reps_(1),
        blob_size_filtering_(BlobSizeFiltering()),
        num_threads_(X_VIEW_NUM_THREADS_BLOB_EXTRACTION) {
  }

  /// \brief Boolean indicating if a dilation and erosion procedure must be
  /// applied to the input image before blob extraction.
  bool dilate_and_erode_;
  /// \brief Number of dilation repetitions to be applied to the image.
  int num_dilate_reps_;
  /// \brief Number of erosion repetitions to be applied to the image.
  int num_erode_reps_;
  /// \brief Filtering parameter determining which blobs are extracted and
  /// which not.
  BlobSizeFiltering blob_size_filtering_;
  /// \brief Number of threads to be used for blob extraction.
  int num_threads_;

  /**
   * \brief Computes the minimum number of pixels a blob must have in order
   * to be extracted from the image.
   * \param image_size Size of the image being analyzed for blob extraction.
   * \return Minimum number of pixels a blob must have in order to be
   * extracted from the image.
   */
  int minimumBlobSize(const cv::Size& image_size) const {
    return blob_size_filtering_.minimumBlobSize(image_size);
  }
};

/**
* \brief This class provides methods to extract blobs from an image.
*/
class BlobExtractor {

 public:

  /**
   * \brief Computes the blobs from the image passed as argument and
   * generates a ImageBlobs datastructure containing all the detected blobs.
   * \param image Image to be analyzed for blob detection.
   * \param params Parameters to be used for blob extraction.
   * \return ImageBlobs datastructure containing all detected blobs.
   */
  static ImageBlobs findBlobsWithContour(const cv::Mat& image,
                                         const BlobExtractorParams& params =
                                         BlobExtractorParams());

};

}

#undef X_VIEW_NUM_THREADS_BLOB_EXTRACTION

#endif //X_VIEW_BLOB_EXTRACTOR_H
