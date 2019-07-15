CXXFLAGS=-std=c++11 -I../../sclib/include

all: kmeans xmeans

kmeans: kmeans.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

xmeans: xmeans.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

clean:
	rm -rf *~
	rm -rf kmeans
