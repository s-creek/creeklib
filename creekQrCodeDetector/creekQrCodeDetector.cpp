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
  if( !cropImage(in_src) ) {
    return false;
  }
  align();
  return true;
}


void creekQrCodeDetector::drawFinderPattern(cv::Mat &in_src, int num)
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

  int size = m_index.size();
  if( num > 0 ) {
    size = std::min(size, num);
  }
  for(int i=0; i<size; i++) {
    std::vector<int> tmp = m_index[i];
    for(int j=0; j<tmp.size(); j++) {
      cv::drawContours(in_src, m_contours, tmp[j], colors[j], 2, CV_AA, m_hierarchy, 0);
    }
  }
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


bool creekQrCodeDetector::cropImage(cv::Mat &in_src)
{
  std::vector<cv::Point> contour;
  for(int i=0; i<3; i++) {
    int index = m_index[i].back();
    contour.insert(contour.end(), m_contours[index].begin(), m_contours[index].end());
  }
  
  
  float x, y, max;
  cv::Mat crop, tmp;
  tmp = in_src.clone();


  int mode=-1;
  float scale=1.04;
  if( mode==0 ) {
    cv::RotatedRect rect = cv::minAreaRect( contour );
    
    cv::Mat rot = cv::getRotationMatrix2D( rect.center, rect.angle, 1.0 );
    cv::warpAffine(in_src, tmp, rot, in_src.size());

    max = std::max(rect.size.width, rect.size.height)*scale;
    x = rect.center.x - max/2.0;  if(x<0) x = 0;
    y = rect.center.y - max/2.0;  if(y<0) y = 0;
  }
  else {
    cv::Rect rect = cv::boundingRect( contour );
    
    cv::Point2f center;
    center.x = rect.x + rect.width/2.0;
    center.y = rect.y + rect.height/2.0;

    max = std::max(rect.width, rect.height)*scale;
    x = center.x - max/2.0;  if(x<0) x = 0;
    y = center.y - max/2.0;  if(y<0) y = 0;
  }


  try {
    crop = cv::Mat(tmp, cv::Rect(x, y, max, max));
    cv::resize(crop, m_out, m_out.size(), 0, 0, cv::INTER_LINEAR);
  }
  catch(cv::Exception) {
    return false;
  }
  
  return true;
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


void creekQrCodeDetector::align()
{
  // calc center of mass
  std::vector<cv::Moments> mu(m_index.size());
  std::vector<cv::Point2f> mc(m_index.size());

  for(int i=0; i<m_index.size(); i++) {
    int index = m_index[i].back();
    mu[i] = cv::moments(m_contours[index]);
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }


  // each distance
  enum {A, B, C};
  double dAB, dBC, dCA;
  dAB = distance(mc[A], mc[B]);
  dBC = distance(mc[B], mc[C]);
  dCA = distance(mc[C], mc[A]);
  std::cout << "AB = " << dAB << ",  BC = " << dBC << ",  CA = " << dCA << std::endl;


  // finder index
  //
  //  0------1
  //  |
  //  |
  //  2
  //
  // align
  std::vector< std::vector<int> > old_index(m_index.begin(), m_index.end());
  if ( dAB > dBC && dAB > dCA )
    {
      cv::Point2f CA, CB;
      CA = mc[A] - mc[C];
      CB = mc[B] - mc[C];

      m_index[0] = old_index[C];
      if( cross(CA, CB) > 0 ) {
	m_index[1] = old_index[A];
	m_index[2] = old_index[B];
      }
      else {
	m_index[1] = old_index[B];
	m_index[2] = old_index[A];
      }
    }
  else if ( dCA > dAB && dCA > dBC )
    {
      cv::Point2f BA, BC;
      BA = mc[A] - mc[B];
      BC = mc[C] - mc[B];

      m_index[0] = old_index[B];
      if( cross(BA, BC) > 0 ) {
	m_index[1] = old_index[A];
	m_index[2] = old_index[C];
      }
      else {
	m_index[1] = old_index[C];
	m_index[2] = old_index[A];
      }
    }
  else if ( dBC > dAB && dBC > dCA )
    {
      cv::Point2f AB, AC;
      AB = mc[B] - mc[A];
      AC = mc[C] - mc[A];

      m_index[0] = old_index[A];
      if( cross(AB, AC) > 0 ) {
	m_index[1] = old_index[B];
	m_index[2] = old_index[C];
      }
      else {
	m_index[1] = old_index[C];
	m_index[2] = old_index[B];
      }
    }
}


float creekQrCodeDetector::distance(cv::Point2f &P, cv::Point2f &Q)
{
  return sqrt(pow(abs(P.x - Q.x),2) + pow(abs(P.y - Q.y),2)); 
}


float creekQrCodeDetector::cross(cv::Point2f &v1, cv::Point2f &v2)
{
  return v1.x*v2.y - v1.y*v2.x;
}
