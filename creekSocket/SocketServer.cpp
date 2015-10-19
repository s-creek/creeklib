#include "SocketServer.h"
#include "ThreadBase.hpp"

#include <cstring>
#include <iostream>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>

using namespace creek;
#define MAX_BUFFER_SIZE 512

SocketServer::SocketServer()
  : m_socket(-1),
    m_client(-1),
    m_active(false)
{
  //std::cout << "SocketServer::SocketServer()" << std::endl;
}


SocketServer::~SocketServer()
{
  this->stop();
  //std::cout << "SocketServer::~SocketServer()" << std::endl;
}


bool SocketServer::init(unsigned short in_portNum)
{
  m_active = false;


  // ソケットの作成
  m_socket = socket(AF_INET, SOCK_STREAM, 0);


  // ソケットのバインド
  struct sockaddr_in addr;
  addr.sin_port        = htons(in_portNum);
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if( bind(m_socket, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
    return false;
  }


  //int val = 1;
  //ioctl(m_socket, FIONBIO, &val);


  // 接続の許可（回線数の指定）
  listen(m_socket, 5);


  //std::cout << "SocketServer::init()" << std::endl;
  return true;
}


void SocketServer::clear()
{
  for( std::vector<SocketServer*>::iterator iter = m_receiver.begin(); iter != m_receiver.end(); ++iter)
    delete *iter;
  m_receiver.clear();

  //std::cout << "SocketServer::clear()" << std::endl;
}


bool SocketServer::start()
{
  if( !ThreadBase::start() )
    return false;

  m_active = true;

  //std::cout << "SocketServer::start()" << std::endl;
  return true;
}


bool SocketServer::stop()
{
  m_active = false;

  // socket のクローズ
  ::close(m_client);
  ::close(m_socket);
    
  m_client = -1;
  m_socket = -1;

  clear();


  // thread の停止
  if( !ThreadBase::stop() )
    return false;

  //std::cout << "SocketServer::stop()" << std::endl;
  return true;
}


void SocketServer::run()
{
  while(m_active) {
    //
    // 通信待機
    //
    if(m_client < 0) {

      // check active receiver
      for( std::vector<SocketServer*>::iterator iter = m_receiver.begin(); iter != m_receiver.end(); ) {
	if( ! (*iter)->isActive() ) {
	  delete *iter;
	  iter = m_receiver.erase(iter);
	  //std::cout << "remove" << std::endl;
	}
	else {
	  ++iter;
	}
      }


      // create new client
      struct sockaddr_in clientAddr;
      int len = sizeof(clientAddr);
      int client;
      client = accept(m_socket, (struct sockaddr*) &clientAddr, (socklen_t*) &len);


      // create new receiver
      if( client > -1 ) {
	SocketServer *receiver = new SocketServer();
	receiver->setClient(client);
	if( receiver->start() ) {
	  std::cout << "create new receiver. client id = " << client << std::endl;
	  m_receiver.push_back(receiver);
	}
	else {
	  std::cerr << "failed to create new receiver" << std::endl;
	  delete receiver;
	}
      }
    }
    //
    // 受信
    //
    else {
      char data[MAX_BUFFER_SIZE+1];
      int bufsize = recv(m_client, &data[0], MAX_BUFFER_SIZE, 0);
  
      if( bufsize >= MAX_BUFFER_SIZE ) {
	std::cerr << "buffer overflow" << std::endl;
      }
      else if( bufsize > 0 ) {
	std::string st(data);
	std::cout << st;
      }
      else {
	std::cout << "connection closed. client id = " << m_client << std::endl;
	this->stop();
	return;
      }
    }
  }
}
