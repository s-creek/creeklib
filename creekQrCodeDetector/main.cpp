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


  int key(0), num(0);
  while(key != 'q') {
    num++;
    tmp = src.clone();
    dec.drawFinderPattern(tmp, num);
    std::cout << "finder index ( 0 : " << (num-1) << " )" << std::endl;
    num = num%3;

    cv::imshow("src", src);
    cv::imshow("finder", tmp);
    cv::imshow("QR", dec.getQrImage());

    key = cv::waitKey(0) & 255;
  }
  return 0;
}
