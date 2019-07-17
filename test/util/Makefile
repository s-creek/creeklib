CXXFLAGS=-std=c++11 -I../../sclib/include
EIGEN_FLAGS=`pkg-config eigen3 --cflags`

all: eigen_test statistics_test

eigen_test: eigen_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(EIGEN_FLAGS)

statistics_test: statistics_test.cpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(EIGEN_FLAGS)

clean:
	rm -rf *~
	rm -rf eigen_test statistics_test
