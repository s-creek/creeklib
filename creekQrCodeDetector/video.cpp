#include <iostream>
#include <creekQrCodeDetector.h>

int main()
{
  cv::VideoCapture video(0);
  cv::Mat src, finder;

  creek::creekQrCodeDetector dec;
  dec.debug(true);
  dec.set(5);

  cv::namedWindow("src", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("finder", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
  cv::namedWindow("QR", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

  int key(0);
  while(key != 'q') {
    video >> src;
    if( src.empty() )
      return 0;
    finder = src.clone();

    if( dec.detectQrCode(src, 0.5) )
      dec.drawFinderPattern(finder);


    cv::imshow("src", src);
    cv::imshow("finder", finder);
    cv::imshow("QR", dec.getQrImage());

    key = cv::waitKey(100) & 255;
  }
  return 0;
}
