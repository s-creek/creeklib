#include "StreamSocketClient.hpp"
#include <iostream>
#include "unistd.h"

int main()
{
  creek::StreamSocketClient ssc;
  ssc.setHeader("[client]");
  ssc.connect("localhost", 1280);  

  ssc.set(std::cout);
  ssc.set(std::cerr);
  ssc.set(std::clog);

  //ssc.connect("localhost", 1280);  


  sleep(1);
  std::cout << "cout" << std::endl;
  sleep(1);
  std::cerr << "cerr" << std::endl;// << "aaaaaaaaaa\n" << std::endl;
  sleep(1);
  std::clog << "clog" << std::endl;
  sleep(1);
  std::clog << "yahooooooooooo\ngoooooooooogle\n";


  ssc.close();
  sleep(1);

  std::cout << "cout" << std::endl;
  std::cerr << "cerr" << std::endl;
  std::clog << "clog" << std::endl;
}
