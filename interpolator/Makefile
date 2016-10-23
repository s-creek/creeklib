CXXFLAGS=-Wall -fPIC `pkg-config eigen3 --cflags`

OBJS=Interpolator.o CubicSplineInterpolator.o
ROTS=RotationInterpolator.o.hrp RotationInterpolator.o.cnoid
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h) RotationInterpolator.h


all: libcreekInterpolator_hrp.so libcreekInterpolator_cnoid.so $(OBJS) $(ROTS)

.cpp.o:
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@

RotationInterpolator.o.hrp: RotationInterpolator.cpp RotationInterpolator.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_HRP_MODEL

RotationInterpolator.o.cnoid: RotationInterpolator.cpp RotationInterpolator.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_CNOID_MODEL

libcreekInterpolator_hrp.so: $(OBJS) $(HEAD) $(ROTS)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJS) RotationInterpolator.o.hrp

libcreekInterpolator_cnoid.so: $(OBJS) $(HEAD) $(ROTS)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJS) RotationInterpolator.o.cnoid

clean:
	rm -f *~
	rm -f *.o
	rm -f RotationInterpolator.o.hrp RotationInterpolator.o.cnoid
	rm -f *.so
