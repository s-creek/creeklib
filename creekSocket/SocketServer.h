// -*- c++ -*-

#ifndef CREEK_SOCKET_SERVER_H
#define CREEK_SOCKET_SERVER_H

#include <string>
#include <vector>
#include "ThreadBase.hpp"

namespace creek
{
  class SocketServer : public ThreadBase
  {
  public:
    SocketServer();
    ~SocketServer();

    bool init(unsigned short in_portNum);
    inline void setClient(int in_client) { m_client = in_client; }
    inline bool isActive() { return m_active; }
    void clear();

    // from ThreadBase
    bool start();
    bool stop();  // 停止処理に追加
    void run();   // thread の実装部


  private:
    int m_socket;
    int m_client;

    bool m_active;

    std::vector<creek::SocketServer*> m_receiver;
  };
}

#endif
