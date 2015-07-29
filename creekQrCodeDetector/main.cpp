#include <iostream>
#include <creekQrCodeDetector.h>

int main( int argc, char **argv )
{
  if( argc < 2 )
    return 0;

  cv::Mat src = cv::imread(argv[1]);
  cv::Mat tmp = src.clone();

  creek::creekQrCodeDetector dec;
  if( !dec.detectQrCode(src, 0.7) )
    return 0;
  dec.drawFinderPattern(tmp, 2);

  cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("finder", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  cv::imshow("src", src);
  cv::imshow("finder", tmp);
  cv::imshow("QR", dec.getQrImage());

  cv::waitKey(0);

  return 0;
}