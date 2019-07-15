CXXFLAGS=-std=c++11 -I../../sclib/include

all: kmeans

kmeans: kmeans.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf *~
	rm -rf kmeans
