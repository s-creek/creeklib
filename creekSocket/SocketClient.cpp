#include "SocketClient.h"

#include <cstring>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

using namespace creek;

SocketClient::SocketClient()
  : m_socket(-1),
    m_isInit(false)
{
}


SocketClient::~SocketClient()
{
  SocketClient::close();
}


bool SocketClient::connect(std::string in_serverIP, unsigned short in_portNum)
{
  m_isInit = false;


  // ソケットの作成
  m_socket = socket(AF_INET, SOCK_STREAM, 0);


  // サーバ情報の設定
  struct hostent*  hostInfo;
  hostInfo = gethostbyname(in_serverIP.c_str());

  long hostAddress;  
  std::memcpy(&hostAddress, hostInfo->h_addr, hostInfo->h_length);


  // sockaddr_in 構造体の設定
  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_port        = htons(in_portNum);
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = hostAddress;


  // 接続
  if( ::connect(m_socket, (struct sockaddr*) &addr, sizeof(addr)) == 0 ) {
    m_isInit = true;
  }


  return m_isInit;
}


bool SocketClient::send(std::string &data)
{
  if( !m_isInit )
    return false;

  if( ::send(m_socket, data.c_str(), data.size(), 0) < 0 )
    return false;

  return true;
}


void SocketClient::close()
{
  ::close(m_socket);
  m_socket = -1;
  m_isInit = false;
}
