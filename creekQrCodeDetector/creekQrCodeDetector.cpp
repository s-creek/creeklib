#include "creekQrCodeDetector.h"

using namespace creek;

creekQrCodeDetector::creekQrCodeDetector(int in_requiredChildNum)
  : m_requiredChildNum(in_requiredChildNum),
    m_out(256, 256, CV_8UC1)
{
}


bool creekQrCodeDetector::detectQrCode(cv::Mat &in_src, double in_th)
{
  cv::cvtColor(in_src, m_gray, CV_BGR2GRAY);
  cv::Canny(m_gray, m_edge, 100, 200, 3, true);

  if( !detectFinderPattern(in_th) ) {
    return false;
  }
  cropImage(in_src);
  return true;
}


bool creekQrCodeDetector::detectFinderPattern(double in_th)
{
  cv::findContours( m_edge, m_contours, m_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );

  m_index.clear();
  for(int i=0; i<m_hierarchy.size(); i++) {
    int k=i, c=0;
    
    // find child end
    while(m_hierarchy[k][2] != -1) {
      k = m_hierarchy[k][2];
      c++;
    }
    if( m_hierarchy[k][0] != -1 )
      c = 0;

    // check child num
    if( c >= m_requiredChildNum ) {
      // create index (temporary)
      std::vector<int> tmp;
      for(int j=0; j<m_requiredChildNum; j++) {
	tmp.push_back(k);
	k = m_hierarchy[k][3];
      }

      // check duplication
      if( std::find(m_index.begin(), m_index.end(), tmp) == m_index.end() ) {
	// check rectangle
	if( minAreaRatio(tmp) > in_th )
	  m_index.push_back(tmp);
      }
    }
  }
  return (m_index.size() == 3);
}


double creekQrCodeDetector::minAreaRatio(std::vector<int> &in_contIndex)
{
  double min=1.0;

  for(int i=0; i<in_contIndex.size(); i++) {
    int index=in_contIndex[i];
    cv::RotatedRect rect = cv::minAreaRect( m_contours[index] );
    double rect_area = rect.size.width * rect.size.height;
    double cont_area = cv::contourArea( m_contours[index] );

    double ratio=cont_area/rect_area;
    if( min>ratio ) min=ratio;
  }
  return min;
}


void creekQrCodeDetector::drawFinderPattern(cv::Mat &in_src)
{
  std::vector< cv::Scalar > colors = {cv::Scalar(0, 255, 0),
				      cv::Scalar(0, 0, 255),
				      cv::Scalar(200, 0, 200),
				      cv::Scalar(255, 0, 0),
				      cv::Scalar(0, 200, 200),
				      cv::Scalar(200, 200, 0),
				      cv::Scalar(0, 0, 100),
				      cv::Scalar(0, 100, 0),
				      cv::Scalar(100, 0, 0)};

  for(int i=0; i<m_index.size(); i++) {
    std::vector<int> tmp = m_index[i];
    for(int j=0; j<tmp.size(); j++) {
      cv::drawContours(in_src, m_contours, tmp[j], colors[j], 2, CV_AA, m_hierarchy, 0);
    }
  }
}


void creekQrCodeDetector::cropImage(cv::Mat &in_src)
{
  std::vector<cv::Point> contour;
  for(int i=0; i<3; i++) {
    int index = m_index[i].back();
    contour.insert(contour.end(), m_contours[index].begin(), m_contours[index].end());
  }
  

  int mode=1;
  if(mode==0) {
    cv::RotatedRect rect = cv::minAreaRect( contour );
    cv::Point2f center = rect.center;
    cv::Size2f  size = rect.size;

    float max = std::max(size.width, size.height)*1.04;
    float x = center.x - max/2.0;  if(x<0) x = 0;
    float y = center.y - max/2.0;  if(y<0) y = 0;
    cv::Mat crop(in_src, cv::Rect(x, y, max, max));
    cv::resize(crop, m_out, m_out.size(), 0, 0, cv::INTER_LINEAR);
  }
  else if(mode==1) {
    cv::RotatedRect rect = cv::minAreaRect( contour );
    
    cv::Point2f center = rect.center;
    cv::Mat rot = cv::getRotationMatrix2D( center, rect.angle, 1.0 );
    cv::Mat tmp;
    cv::warpAffine(in_src, tmp, rot, in_src.size());

    cv::Size2f  size = rect.size;
    float max = std::max(size.width, size.height)*1.1;
    float x = center.x - max/2.0;  if(x<0) x = 0;
    float y = center.y - max/2.0;  if(y<0) y = 0;

    cv::Mat crop(tmp, cv::Rect(x, y, max, max));
    cv::resize(crop, m_out, m_out.size(), 0, 0, cv::INTER_LINEAR);
  }
  else {
    cv::Rect rect = cv::boundingRect( contour );
    cv::Point2f center;
    center.x = rect.x + rect.width/2.0;
    center.y = rect.y + rect.height/2.0;

    float max = std::max(rect.width, rect.height)*1.04;
    float x = center.x - max/2.0;  if(x<0) x = 0;
    float y = center.y - max/2.0;  if(y<0) y = 0;
    cv::Mat crop(in_src, cv::Rect(x, y, max, max));
    cv::resize(crop, m_out, m_out.size(), 0, 0, cv::INTER_LINEAR);
  }
}
