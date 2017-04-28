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

}

#endif //X_VIEW_X_VIEW_TOOLS_H
