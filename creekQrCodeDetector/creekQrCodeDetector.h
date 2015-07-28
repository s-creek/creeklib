// -*- c++ -*-

#ifndef CREEK_QR_CODE_DETECTOR_H
#define CREEK_QR_CODE_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace creek
{
  class creekQrCodeDetector
  {
  public:
    creekQrCodeDetector(int in_requiredChildNum=5);
    inline void set(int in_requiredChildNum) { m_requiredChildNum = in_requiredChildNum; }

    bool detectQrCode(cv::Mat &in_src, double in_th=0.7);
    void drawFinderPattern(cv::Mat &in_src);
    inline cv::Mat getQrImage() { return m_out; }


  private:
    bool detectFinderPattern(double in_th=0.70);
    double minAreaRatio(std::vector<int> &in_contIndex);
    void cropImage(cv::Mat &in_src);

    int m_requiredChildNum;
    cv::Mat m_gray, m_edge, m_out;

    std::vector< std::vector<cv::Point> > m_contours;
    std::vector< cv::Vec4i > m_hierarchy;
    std::vector< std::vector<int> > m_index;
  };
};

#endif
