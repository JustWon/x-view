#ifndef X_VIEW_X_VIEW_TOOLS_H
#define X_VIEW_X_VIEW_TOOLS_H

#include <opencv2/core/core.hpp>

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

}

#endif //X_VIEW_X_VIEW_TOOLS_H
