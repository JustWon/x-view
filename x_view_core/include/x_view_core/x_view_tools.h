#ifndef X_VIEW_X_VIEW_TOOLS_H
#define X_VIEW_X_VIEW_TOOLS_H

#include <opencv2/core/core.hpp>

namespace x_view {

// Utility function to convert two consecutive bytes into a 16bits unsigned
// int value. This function assumes little endiannes of the system

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

}

#endif //X_VIEW_X_VIEW_TOOLS_H
