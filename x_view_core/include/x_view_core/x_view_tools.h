#ifndef X_VIEW_X_VIEW_TOOLS_H
#define X_VIEW_X_VIEW_TOOLS_H

#include <x_view_core/features/graph.h>
#include <x_view_core/x_view_types.h>

#include <opencv2/core/core.hpp>

#include <random>

namespace x_view {


// ************************* Image manipulation ******************************//
/**
 * \brief Interpret two consecutive bytes as an integer.
 * \param b1 First byte.
 * \param b2 Second byte.
 * \return Integer corresponding to the two bytes passed as argument.
 */
int twoBytesToInt(const unsigned char b1, const unsigned char b2);

/**
 * \brief Interprets two consecutive bytes pointed by the passed argument as
 * an integer.
 * \param b Pointer to the bytes.
 * \return Integer corresponding to the two consecutive bytes pointed by the
 * passed argument.
 */
int twoBytesToInt(const unsigned char* b);

/**
 * \brief Extract a single channel from an image.
 * \param image Input image composed of multiple channels.
 * \param channel Desired channel to be extracted.
 * \return New image containing only the channel passed as argument.
 */
cv::Mat extractChannelFromImage(const cv::Mat& image, const int channel);

// ******************************* Utility ***********************************//
class padded_int {
 public:
  padded_int(const int value, const int pad = 0, const char fill = '0');

  const std::string& str() const;

 private:
  const int value_;
  const int pad_;
  const char fill_;
  std::string str_;
};

std::string operator + (const std::string& l, const padded_int& r);
std::string operator + (const padded_int& l, const std::string& r);

const std::string formatSE3(const SE3& se3, const std::string& indent = "",
                            const int precision = Eigen::StreamPrecision);

/// \brief Generates different colors for different semantic labels.
const cv::Scalar getColorFromSemanticLabel(const int semantic_label);

// ******************************* Logging ***********************************//
/**
 * \brief Stringification macros used to transform preprocessor strings into
 * c++ strings.
 */
#define X_VIEW_XSTR(s) X_VIEW_STR(s)
#define X_VIEW_STR(s)  #s

/**
 * @brief Returns a string containing the absolute path to the X_View root
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_ROOT_DIR=..."
 */
const std::string& getRootDirectory();

/**
 * @brief Returns a string containing the absolute path to the X_View output
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_OUT_DIR=..."
 */
const std::string& getOutputDirectory();

/**
 * @brief Returns a string containing the absolute path to the X_View log
 * directory specified in the CMakeLists.txt file as "-DX_VIEW_LOG_DIR=..."
 */
const std::string& getLogDirectory();

/**
 * \brief Sets up parameters for logging such that log files are written to
 * the directory specified by "-DX_VIEW_LOG_DIR=..." in the CMakeLists.txt file.
 * \param argv Arguments received in the main() function.
 */
void setupLogging(char** argv);

/**
 * \brief Makes sure all log are written to the corresponding files.
 */
void finalizeLogging();

// **************************** Graph modifiers ******************************//

/**
 * \brief Adds a new generated VertexProperty to the graph pointed by the
 * passed argument. The newly generated vertex is linked towards
 * link_to_n_vertices existing vertices of the graph.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 * \param index Index to associate to the newly added vertex.
 * \param link_to_n_vertices The added vertex is linked to link_to_n_vertices
 * randomly chosen vertices of the graph. This ensure that the new graph
 * consists of a single connected component.
 */
void addRandomVertexToGraph(Graph* graph, std::mt19937& rng,
                            const int index, const int link_to_n_vertices);

/**
 * \brief Adds a new generated EdgeProperty to the graph pointed by the
 * passed argument. The edge is defined by randomly selecting two different
 * vertices of the graph passed as argument, and is added to the graph only
 * if the resulting edge does not already exist.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator.
 */
void addRandomEdgeToGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes a random vertex from the graph pointed by the passed argument.
 * This function makes sure that removing the vertex from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomVertexFromGraph(Graph* graph, std::mt19937& rng);

/**
 * \brief Removes a random edge from the graph pointed by the passed argument.
 * This function makes sure that removing the edge from the graph
 * does not create two disconnected components.
 * \param graph Pointer to the graph to be modified.
 * \param rng Instance of mersenne twister random number generator
 */
void removeRandomEdgeFromGraph(Graph* graph, std::mt19937& rng);

}

#endif //X_VIEW_X_VIEW_TOOLS_H
