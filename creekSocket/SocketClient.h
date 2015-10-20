// -*- c++ -*-

#ifndef CREEK_SOCKET_CLIENT_H
#define CREEK_SOCKET_CLIENT_H

#include <string>

namespace creek
{
  class SocketClient
  {
  public:
    SocketClient();
    ~SocketClient();

    bool connect(std::string in_serverIP, unsigned short in_portNum);
    bool send(std::string &data);
    void close();

    inline bool isInit() { return m_isInit; }
    

  private:
    int  m_socket;
    bool m_isInit;
  };
}

#endif
