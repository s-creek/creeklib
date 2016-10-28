CXXFLAGS=-Wall -fPIC `pkg-config eigen3 --cflags`

OBJS=StepSequence.o CapturePoint.o Geometry.o
SRCS=$(OBJS:.o=.cpp)
HEAD=$(OBJS:.o=.h) discretization.hpp

OBJ_HRP=$(OBJS:.o=.o.hrp)
OBJ_CNOID=$(OBJS:.o=.o.cnoid)


all: $(OBJS) libcreekWalk_hrp.so libcreekWalk_cnoid.so

.cpp.o: $(HEAD)
	rm -f $@.hrp $@.cnoid
	$(CXX) $(CXXFLAGS) $< -c -o $@.hrp -DUSE_HRP_MODEL
	$(CXX) $(CXXFLAGS) $< -c -o $@.cnoid -DUSE_CNOID_MODEL

libcreekWalk_hrp.so: $(OBJ_HRP)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_HRP)

libcreekWalk_cnoid.so: $(OBJ_CNOID)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_CNOID)

clean:
	rm -f *~
	rm -f *.o
	rm -f *.so
	rm -f $(OBJ_HRP) $(OBJ_CNOID)
