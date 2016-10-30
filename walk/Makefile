CXXFLAGS=-Wall -fPIC `pkg-config eigen3 --cflags`

OBJS=StepSequence.o CapturePoint.o Geometry.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h) discretization.hpp BipedRobot.h WalkPlanner.h

OBJ_HRP=$(OBJS:.o=.o.hrp)
OBJ_CNOID=$(OBJS:.o=.o.cnoid) BipedRobot.o.cnoid WalkPlanner.o.cnoid


CNOID_VERSION=$(subst .,,$(shell pkg-config --modversion choreonoid-body-plugin))


all: $(OBJS) libcreekWalk_hrp.so libcreekWalk_cnoid.so BipedRobot.o.cnoid WalkPlanner.o.cnoid main

.cpp.o: $(HEAD)
	rm -f $@.hrp $@.cnoid
	$(CXX) $(CXXFLAGS) $< -c -o $@.hrp -DUSE_HRP_MODEL
	$(CXX) $(CXXFLAGS) $< -c -o $@.cnoid -DUSE_CNOID_MODEL

BipedRobot.o.cnoid: BipedRobot.cpp BipedRobot.h
	rm -f $@
	$(CXX) $(CXXFLAGS) `pkg-config choreonoid-body-plugin --cflags` $< -c -o $@ -DUSE_CNOID_MODEL -DCNOID_VERSION=$(CNOID_VERSION)

WalkPlanner.o.cnoid: WalkPlanner.cpp WalkPlanner.h
	rm -f $@
	$(CXX) $(CXXFLAGS) `pkg-config choreonoid-body-plugin --cflags` $< -c -o $@ -DUSE_CNOID_MODEL -DCNOID_VERSION=$(CNOID_VERSION)

libcreekWalk_hrp.so: $(OBJ_HRP)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_HRP)

libcreekWalk_cnoid.so: $(OBJ_CNOID)
	$(CXX) $(CXXFLAGS) `pkg-config choreonoid-body-plugin --cflags` -shared -o $@ $(OBJ_CNOID)

main: main.cpp libcreekWalk_cnoid.so
	$(CXX) $(CXXFLAGS) `pkg-config choreonoid-body-plugin --cflags` $< -o $@ libcreekWalk_cnoid.so ../interpolator/libcreekInterpolator_cnoid.so `pkg-config choreonoid-body-plugin --libs` -DCNOID_VERSION=$(CNOID_VERSION)

clean:
	rm -f *~
	rm -f *.o
	rm -f *.so
	rm -f $(OBJ_HRP) $(OBJ_CNOID)
