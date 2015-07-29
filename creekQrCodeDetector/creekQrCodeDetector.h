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
    void drawFinderPattern(cv::Mat &in_src, int num=-1);
    inline cv::Mat getQrImage() { return m_out; }


  private:
    bool detectFinderPattern(double in_th=0.70);
    bool cropImage(cv::Mat &in_src);
    double minAreaRatio(std::vector<int> &in_contIndex);
    void align();

    float distance(cv::Point2f &P, cv::Point2f &Q);
    float cross(cv::Point2f &v1, cv::Point2f &v2);

    int m_requiredChildNum;
    cv::Mat m_gray, m_edge, m_out;

    std::vector< std::vector<cv::Point> > m_contours;
    std::vector< cv::Vec4i > m_hierarchy;
    std::vector< std::vector<int> > m_index;  // finder pattern index (3 x requiredChildNum)
  };
};

#endif
