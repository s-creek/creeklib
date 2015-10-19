LIBNAME=creekSocket
SONAME=lib$(LIBNAME).so

CXX=g++
CXXFLAGS=-Wall -fPIC -I.
LDFLAGS=-L. -lpthread

OBJS=SocketServer.o SocketClient.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h) ThreadBase.hpp


.SUFFIXES: .cpp .o

all: $(SONAME) client server stream

.cpp.o:
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@

$(SONAME): $(OBJS) $(HEAD)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJS) $(LDFLAGS)

clean:
	rm -f $(SONAME)
	rm -f *.o
	rm -f *~
	rm -f client server stream

client: client.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lcreekSocket

server: server.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lcreekSocket

stream: stream.cpp StreamSocketClient.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lcreekSocket