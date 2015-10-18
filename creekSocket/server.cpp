#include <SocketServer.h>
#include <iostream>

int main()
{
  creek::SocketServer *server = new creek::SocketServer();
  if( !server->init(1280) ) {
    std::cerr << "init error" << std::endl;
    return 0;
  }

  if( !server->start() ) {
    std::cerr << "active error" << std::endl;
  }
 
  std::string st;
  std::cin >> st;

  if( !server->stop() ) {
    std::cout << "stop error" << std::endl;
  }
  
  return 0;
}
