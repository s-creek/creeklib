CXXFLAGS=-std=c++11 -I../../sclib/include
EIGEN_FLAGS=`pkg-config eigen3 --cflags`

all: gmm_test

gmm_test: gmm_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(EIGEN_FLAGS)

clean:
	rm -rf *~
	rm -rf gmm_test
