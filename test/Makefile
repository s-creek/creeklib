CNOID_VERSION=$(subst .,,$(shell pkg-config --modversion choreonoid-body-plugin))

all: cnoid hrp inter eigen tvmet

cnoid: model.cpp
	$(CXX) `pkg-config choreonoid-body-plugin --cflags` -I../../ $< -o $@ -DUSE_CNOID_MODEL -DBOOST_SIGNALS_NO_DEPRECATION_WARNING `pkg-config choreonoid-body-plugin --libs` -DCNOID_VERSION=$(CNOID_VERSION)

hrp: model.cpp
	$(CXX) `pkg-config openhrp3.1 --cflags` -I../../ $< -o $@ -DUSE_HRP_MODEL -Wno-write-strings `pkg-config openhrp3.1 --libs`

inter: inter.cpp
	$(CXX)  -I../../ $< -o $@ -L../interpolator -lcreekInterpolator

tvmet: matrix.cpp
	$(CXX)  -I../../ $< -o $@ -DUSE_HRP_MODEL

eigen: matrix.cpp
	$(CXX)  -I../../ `pkg-config eigen3 --cflags` $< -o $@ -DUSE_CNOID_MODEL

clean:
	rm -f cnoid hrp tvmet eigen
	rm -f *~
