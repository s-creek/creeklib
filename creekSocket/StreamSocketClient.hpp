// -*- c++ -*-

#ifndef CREEK_STREAM_SOCKET_CLIENT_HPP
#define CREEK_STREAM_SOCKET_CLIENT_HPP

#include <sstream>
#include <string>
#include <vector>

#include "SocketClient.h"

namespace creek
{
  class StreamSocketClient
    : public std::stringbuf,
      public creek::SocketClient
  {
  public:
    StreamSocketClient()
      : header("")
    {
    }


    ~StreamSocketClient()
    {
      this->close();
    }


    virtual int sync()
    {
      if( this->isInit() ) {
	std::string data = header + str();
	eraseEndOfWhiteSpace(data);
	if( data.size() > header.size() )
	  this->send( data );
      }

      str("");
      return 0;
    }


    void set(std::ostream &in_os)
    {
      os.push_back(&in_os);
      if( this->isInit() )
	orgbuf.push_back(in_os.rdbuf(this));
    }


    void setHeader(std::string in_header)
    {
      header = in_header + " ";
    }


    bool connect(std::string in_serverIP, unsigned short in_portNum)
    {
      if( this->isInit() ) {
	return true;
      }
      else if( SocketClient::connect(in_serverIP, in_portNum) ) {
	unsigned int n = os.size();
	orgbuf.resize(n);
	for(unsigned int i=0; i<n; i++)
	  orgbuf[i] = os[i]->rdbuf(this);

	return true;
      }
      else {
	return false;
      }
    }


    void close()
    {
      this->sync();
      if( this->isInit() )
	for(unsigned int i=0; i<os.size(); i++)
	  os[i]->rdbuf(orgbuf[i]);
	  
      SocketClient::close();
    }


    void eraseEndOfWhiteSpace(std::string &st)
    {
      while( *st.rbegin() == ' ' || *st.rbegin() == '\t' || *st.rbegin() == '\r' ) {
	st.erase(st.size()-1);
      }
    }

    

  private:
    std::vector< std::ostream* > os;
    std::vector< std::streambuf* > orgbuf;

    std::string header;
  };
};

#endif
