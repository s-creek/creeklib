CXXFLAGS=-std=c++11 -I../../sclib/include
CV_FLAGS=`pkg-config opencv --libs --cflags`

all: kmeans_test kernel_test

kmeans_test: kmeans_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(CV_FLAGS)

kernel_test: kernel_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf *~
	rm -rf kmeans_test kernel_test
