// -*- c++ -*-

#ifndef CREEK_THREAD_BASE_HPP
#define CREEK_THREAD_BASE_HPP

#include <pthread.h>

namespace creek
{
  class ThreadBase
  {
  public:
    ThreadBase();
    ~ThreadBase();

    bool start();
    bool stop();


  protected:
    virtual void run(){};


  private:
    static void* __run(void* own) {
      reinterpret_cast<ThreadBase*>(own)->run();
      return NULL;
    }

    pthread_t m_thread;
  };
}

//---------------------------------------------------------------------------------

namespace creek
{
  ThreadBase::ThreadBase()
  {
  }


  ThreadBase::~ThreadBase()
  {
    stop();
  }


  bool ThreadBase::start()
  {
    if( pthread_create(&m_thread, NULL, __run, this) != 0 )
      return false;

    return true;
  }


  bool ThreadBase::stop()
  {
    if( pthread_cancel(m_thread) == 0 ) {
      pthread_join(m_thread, NULL);
      return true;
    }
    return false;
  }
}

#endif
