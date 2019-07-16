CXXFLAGS=-std=c++11 -I../../sclib/include
CV_FLAGS=`pkg-config opencv --libs --cflags`
EIGEN_FLAGS=`pkg-config eigen3 --cflags`

all: kmeans_test xmeans_test

kmeans_test: kmeans_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CV_FLAGS)

xmeans_test: xmeans_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(EIGEN_FLAGS)

clean:
	rm -rf *~
	rm -rf kmeans_test xmeans_test
