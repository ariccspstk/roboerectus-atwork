#ifndef ARICC_2D_VISION_IMAGE_UTILS_
#define ARICC_2D_VISION_IMAGE_UTILS_

#include <opencv2/opencv.hpp>

namespace aricc_2d_vision
{
  cv::Rect boundingRectOfMaskImage(const cv::Mat& image);
}

#endif
