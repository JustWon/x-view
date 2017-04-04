#include <x-view_core/semantic_factory.h>

#include <x-view_core/x-view_semantics.h>
#include <x-view_core/bos.h>
#include <x-view_core/semantic_graph.h>

#include <exception>

namespace x_view {

    void XViewSemanticFactory::createSemanticLandmark(const cv::Mat &image, const SE3 &pose,
                                                      std::unique_ptr<XViewSemantics> &landmark) {
        CHECK(this->semanticLandmarkType_ >= 0 &&
              this->semanticLandmarkType_ < SEMANTIC_LANDMARK_TYPE::NUM_SEMANTIC_LANDMARK_TYPES);

        switch (this->semanticLandmarkType_) {
            case SEMANTIC_LANDMARK_TYPE::BOS : {
                landmark.reset(new BoS(image, pose));
                break;
            }
            case SEMANTIC_LANDMARK_TYPE::GRAPH : {
                landmark.reset(new SemanticGraph(image, pose));
                break;
            }
            case SEMANTIC_LANDMARK_TYPE::UNDEFINED_SEMANTIC_LANDMARK_TYPE : {
                throw std::runtime_error("Make sure to set a valid landmark type in the landmark factory");
            }
            default : {
                throw std::runtime_error("Unrecognized semantic landmark type in semantic landmark factory");
            }
        }
    }

}

