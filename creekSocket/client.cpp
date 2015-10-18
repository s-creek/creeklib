#include <SocketClient.h>
#include <iostream>
#include <sstream>
#include "unistd.h"

int main()
{
  creek::SocketClient *client1 = new creek::SocketClient();
  if( !client1->connect("localhost", 1280) ) {
    std::cerr << "client1 : connect error" << std::endl;
    return 0;
  }


  creek::SocketClient *client2 = new creek::SocketClient();
  if( !client2->connect("localhost", 1280) ) {
    std::cerr << "client2 : connect error" << std::endl;
    return 0;
  }
  

  sleep(1);


  std::string data;
  for(int i=0; i<4; i++) {
    std::stringstream ss1;
    ss1 << i;
    data = ss1.str();
    std::cout << "send data = " << data << std::endl;
    if( !client1->send(data) ) {
      std::cerr << "client1 : send error" << std::endl;
    }
    sleep(1);


    for(int j=0; j<2; j++) {
      std::stringstream ss2;
      ss2 << i << ", " << j;
      data = ss2.str();
      std::cout << "send data = " << data << std::endl;
      if( !client2->send(data) ) {
    	std::cerr << "client2 : send error" << std::endl;
      }
      sleep(1);
    }
  }


  client1->close();
  sleep(5);


  data = "aaaaaaaa\nbbbbbbbb\ncccccccccc";
  std::cout << "send data = " << data << std::endl;
  if( !client2->send(data) ) {
    std::cerr << "client2 : send error" << std::endl;
  }
  

  sleep(1);
  client2->close();
  

  std::cout << "client end" << std::endl;
  return 0;
}
