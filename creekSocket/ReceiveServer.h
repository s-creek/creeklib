// -*- c++ -*-

#ifndef CREEK_RECEIVE_SERVER_H
#define CREEK_RECEIVE_SERVER_H

#include <string>
#include <vector>
#include "ThreadBase.hpp"
#include <sys/select.h>

namespace creek
{
  class ReceiveServer : public ThreadBase
  {
  public:
    ReceiveServer();
    ~ReceiveServer();

    bool init(unsigned short in_portNum);
    inline bool isActive() { return m_active; }
 
    // from ThreadBase
    bool start();
    bool stop();  // 停止処理に追加
    void run();   // thread の実装部


  private:
    void addClient(fd_set &fdSet);
    void procReceiver(fd_set &fdSet);
    void eraseEndOfWhiteSpace(std::string &st);

    int m_socket;
    std::vector<int> m_receiver;

    bool m_active;
  };
};

#endif
