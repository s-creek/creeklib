CXXFLAGS=-Wall -fPIC `pkg-config eigen3 --cflags`

OBJS=Interpolator.o CubicSplineInterpolator.o RotationInterpolator.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h)

all: libcreekInterpolator.so $(OBJS)

.cpp.o:
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_CNOID_MODEL

libcreekInterpolator.so: $(OBJS) $(HEAD)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJS)

clean:
	rm -f *~
	rm -f *.o
	rm -f libcreekInterpolator.so
