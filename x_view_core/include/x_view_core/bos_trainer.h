#ifndef X_VIEW_BOS_TRAINER_H
#define X_VIEW_BOS_TRAINER_H

#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace x_view {

/**
 * \brief Class training a Bag of Semantic vocabulary
 */
class BoSTrainer : public cv::BOWKMeansTrainer {

 public:
  /**
   * \brief Initializes a BoS trainer with the following parameters
   * \param clusterCount number of clusters/semantic-words to train
   * \param termcrit termination criteria
   * \param attempts attempts to perform k-means
   * \param flags how to initialize the k-means algorithm
   */
  BoSTrainer(int clusterCount,
             const cv::TermCriteria& termcrit = cv::TermCriteria(),
             int attempts = 3,
             int flags = cv::KMEANS_PP_CENTERS);

  virtual ~BoSTrainer();

  /**
   * \brief Adds a semantic lanmdark to the set of descriptors to be used
   * during training
   * \param semanticLandmarkPtr At runtime this must be a BoS object
   */
  void add( const SemanticLandmarkPtr& semanticLandmarkPtr );

};

}

#endif //X_VIEW_BOS_TRAINER_H
