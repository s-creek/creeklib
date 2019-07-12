CXXFLAGS=-Wall -fPIC `pkg-config eigen3 --cflags`
LD_FLAGS=-L../interpolator

OBJ_HRP=StepSequence.o.hrp CapturePoint.o.hrp
OBJ_CNOID=StepSequence.o.cnoid CapturePoint.o.cnoid
SRCS=StepSequence.cpp CapturePoint.cpp
HEAD=StepSequence.h CapturePoint.h

all: $(OBJ_HRP) $(OBJ_CNOID) libcreekWalk_hrp.so libcreekWalk_cnoid.so

StepSequence.o.hrp: StepSequence.cpp StepSequence.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_HRP_MODEL

StepSequence.o.cnoid: StepSequence.cpp StepSequence.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_CNOID_MODEL

CapturePoint.o.hrp: CapturePoint.cpp CapturePoint.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_HRP_MODEL

CapturePoint.o.cnoid: CapturePoint.cpp CapturePoint.h
	rm -f $@
	$(CXX) $(CXXFLAGS) $< -c -o $@ -DUSE_CNOID_MODEL

libcreekWalk_hrp.so: $(OBJ_HRP)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_HRP)

libcreekWalk_cnoid.so: $(OBJ_CNOID)
	$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJ_CNOID)

clean:
	rm -f *~
	rm -f *.o
	rm -f *.so
	rm -f $(OBJ_HRP) $(OBJ_CNOID)
