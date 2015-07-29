#include <iostream>
#include <creekQrCodeDetector.h>
#include <zbar.h>

int main()
{
  cv::VideoCapture video(0);
  cv::Mat src, finder, gray;

  creek::creekQrCodeDetector dec;
  //dec.debug(true);
  //dec.set(5);

  zbar::Image m_zbar(256, 256, "Y800", dec.getQrImage().data, 256*256);
  zbar::ImageScanner m_scanner;
  m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);


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


    cv::cvtColor(src, gray, CV_BGR2GRAY);
    m_zbar.set_size(gray.cols, gray.rows);
    m_zbar.set_data(gray.data, gray.total());
    if( m_scanner.scan(m_zbar) != 0 ) {
      for(zbar::Image::SymbolIterator symbol = m_zbar.symbol_begin(); symbol != m_zbar.symbol_end(); ++symbol) {
	std::cout << "type = " << symbol->get_type_name() << ",  type = " << symbol->get_type() << ",  data" << symbol->get_data() << std::endl;
   
      }
    }


    cv::imshow("src", src);
    cv::imshow("finder", finder);
    cv::imshow("QR", dec.getQrImage());

    key = cv::waitKey(100) & 255;
  }
  return 0;
}
