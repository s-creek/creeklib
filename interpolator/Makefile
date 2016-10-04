CXXFLAGS=-Wall -fPIC

OBJS=Interpolator.o CubicSplineInterpolator.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h)

all: libcreekInterpolator.so $(OBJS)

.cpp.o:
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@

libcreekInterpolator.so: $(OBJS) $(HEAD)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJS)

clean:
	rm -f *~
	rm -f *.o
	rm -f libcreekInterpolator.so
