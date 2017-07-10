#include "aricc_2d_vision/image_utils.h"

namespace aricc_2d_vision
{
  cv::Rect boundingRectOfMaskImage(const cv::Mat& image)
  {
    int min_x = image.cols;
    int min_y = image.rows;
    int max_x = 0;
    int max_y = 0;
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if (image.at<uchar>(j, i) != 0) {
          min_x = std::min(min_x, i);
          min_y = std::min(min_y, j);
          max_x = std::max(max_x, i);
          max_y = std::max(max_y, j);
        }
      }
    }
    
    return cv::Rect(min_x, min_y, std::max(max_x - min_x, 0), std::max(max_y - min_y, 0));
  }
}
