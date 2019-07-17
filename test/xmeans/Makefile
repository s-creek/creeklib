CXXFLAGS=-std=c++11 -I../../sclib/include
EIGEN_FLAGS=`pkg-config eigen3 --cflags`

all: xmeans_test

xmeans_test: xmeans_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(EIGEN_FLAGS)

clean:
	rm -rf *~
	rm -rf xmeans_test
