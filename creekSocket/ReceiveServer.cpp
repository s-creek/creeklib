#include "ReceiveServer.h"
#include <iostream>
#include <algorithm>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <sys/un.h>
#include <unistd.h>
#define MAX_BUFFER_SIZE 512
#define MAX_CONNECTION_NUM 10

using namespace creek;

ReceiveServer::ReceiveServer()
  : m_socket(-1),
    m_active(false)
{
}


ReceiveServer::~ReceiveServer()
{
  this->stop();
}


bool ReceiveServer::init(unsigned short in_portNum)
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

 
  return true;
}


bool ReceiveServer::start()
{
  if( !ThreadBase::start() )
    return false;

  m_active = true;
  return true;
}


bool ReceiveServer::stop()
{
  m_active = false;

  // socket のクローズ
  ::close(m_socket);
  for(unsigned int i=0; i<m_receiver.size(); i++)
    ::close(m_receiver[i]);

  m_socket = -1;
  m_receiver.clear();


  // thread の停止
  if( !ThreadBase::stop() )
    return false;


  return true;
}


void ReceiveServer::run()
{
  fd_set fdSet;
  struct timeval tv = {0, 1000};  // time out
 
  while(m_active) {
    // fd_setを初期化
    FD_ZERO(&fdSet);
    // select対象のsocketを設定
    FD_SET(m_socket, &fdSet);
    for(unsigned int i=0; i<m_receiver.size(); i++)
      FD_SET(m_receiver[i], &fdSet);
    

    int select_socket = m_socket + 1;
    if( !m_receiver.empty() )
      select_socket = std::max(m_socket, *std::max_element(m_receiver.begin(), m_receiver.end())) + 1;


    int ret = select(select_socket, &fdSet, NULL, NULL, &tv);  // ret = -1:error, 0:time out
    if( ret == -1 ) {
      this->stop();
      break;
    }
    else if( ret != 0 ) {
      procReceiver(fdSet);

      addClient(fdSet);
    }
  }
}


void ReceiveServer::addClient(fd_set &fdSet)
{
  if( FD_ISSET(m_socket, &fdSet) != 0 ) {
    struct sockaddr_in clientAddr;
    int len = sizeof(clientAddr);
    int client = accept(m_socket, (struct sockaddr*) &clientAddr, (socklen_t*) &len);

    if( client == -1 ) {
      std::cout << "accept error" << std::endl;
    }
    else if( m_receiver.size() < MAX_CONNECTION_NUM ) {
      m_receiver.push_back(client);
      std::cout << "accept client = " << client << std::endl;
    }
    else {
      ::close(client);
      std::cout << "max connection" << std::endl;
    }
  }
}


void ReceiveServer::procReceiver(fd_set &fdSet)
{
  for( std::vector<int>::iterator iter = m_receiver.begin(); iter != m_receiver.end(); ) {
    int client = *iter;
    if( FD_ISSET(client, &fdSet) != 0 ) {
      char data[MAX_BUFFER_SIZE+1];
      int bufsize = recv(client, &data[0], MAX_BUFFER_SIZE, 0);

      if( bufsize >= MAX_BUFFER_SIZE ) {
	std::cerr << "buffer overflow" << std::endl;
      }

      if( bufsize > 0 ) {
	std::string st(data, bufsize);
	eraseEndOfWhiteSpace(st);
	std::cout << st << std::endl;
	++iter;
      }
      else {
	std::cout << "connection closed. client id = " << client << std::endl;
	::close(client);
	FD_CLR(client, &fdSet);
	iter = m_receiver.erase(iter);
      }
    }
    else
      ++iter;
  }
}


void ReceiveServer::eraseEndOfWhiteSpace(std::string &st)
{
  while( *st.rbegin() == ' ' || *st.rbegin() == '\t' || *st.rbegin() == '\r' || *st.rbegin() == '\n' ) {
    st.erase(st.size()-1);
  }
}
