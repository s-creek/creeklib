CXXFLAGS=-std=c++0x -Wall -fPIC -DBOOST_SIGNALS_NO_DEPRECATION_WARNING `pkg-config eigen3 --cflags`

OBJS=StepSequence.o CapturePoint.o Geometry.o BipedRobot.o WalkPlanner.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h) discretization.hpp

OBJ_HRP=$(OBJS:.o=.o.hrp)
OBJ_CNOID=$(OBJS:.o=.o.cnoid)

HRP_FLAGS=$(CXXFLAGS) `pkg-config openhrp3.1 --cflags`
CNOID_FLAGS=$(CXXFLAGS) `pkg-config choreonoid-body-plugin --cflags`

HRP_LIBS=`pkg-config openhrp3.1 --libs`
CNOID_LIBS=`pkg-config choreonoid-body-plugin --libs`
CNOID_VERSION=$(subst .,,$(shell pkg-config --modversion choreonoid-body-plugin))


all: $(OBJS) libcreekWalk_hrp.so libcreekWalk_cnoid.so main

.cpp.o: $(HEAD)
	rm -f $@.hrp $@.cnoid
	$(CXX) $(HRP_FLAGS) $< -c -o $@.hrp -DUSE_HRP_MODEL
	$(CXX) $(CNOID_FLAGS) $< -c -o $@.cnoid -DUSE_CNOID_MODEL -DCNOID_VERSION=$(CNOID_VERSION)

libcreekWalk_hrp.so: $(OBJ_HRP)
	$(CXX) $(HRP_FLAGS) -shared -o $@ $(OBJ_HRP) $(HRP_LIBS)

libcreekWalk_cnoid.so: $(OBJ_CNOID)
	$(CXX) $(CNOID_FLAGS) -shared -o $@ $(OBJ_CNOID) $(CNOID_LIBS)

main: main.cpp libcreekWalk_cnoid.so
	$(CXX) $(CNOID_FLAGS) $< -o $@ libcreekWalk_cnoid.so ../interpolator/libcreekInterpolator_cnoid.so $(CNOID_LIBS) -DCNOID_VERSION=$(CNOID_VERSION)

clean:
	rm -f *~
	rm -f *.o
	rm -f *.so
	rm -f $(OBJ_HRP) $(OBJ_CNOID)
