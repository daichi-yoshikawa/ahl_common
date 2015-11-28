/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#ifndef __CV_WRAPPER_UTILS_HPP
#define __CV_WRAPPER_UTILS_HPP

#include <opencv2/opencv.hpp>

namespace cv_wrapper
{

  bool isEmpty(const cv::Mat& img);
  bool imshow(const std::string& name, const cv::Mat& img);
  char waitKey(double interval);
  void closing(const cv::Mat& src, cv::Mat& dst, unsigned int erodes, unsigned int dilates);
  void convertRGBToHSV(const cv::Mat& src, std::vector<cv::Mat>& hsv);
  void extractDesignatedArea   (cv::Mat& gray, cv::Mat& dst, int min, int max);
  void extractDesignatedHueArea(cv::Mat& hue,  cv::Mat& dst, int min, int max);
  void extractDesignatedSatArea(cv::Mat& sat,  cv::Mat& dst, int min, int max);
  void extractDesignatedValArea(cv::Mat& val,  cv::Mat& dst, int min, int max);
  void extractDesignatedHSVArea(std::vector<cv::Mat>& hsv, cv::Mat& dst,
                                int h_min, int h_max,
                                int s_min, int s_max,
                                int v_min, int v_max,
                                bool labeling, int erode_num, int dilate_num);
  void labeling(cv::Mat& src, cv::Mat& dst);

}

#endif /* __CV_WRAPPER_UTILS_HPP */
