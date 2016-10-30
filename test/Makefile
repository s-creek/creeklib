CNOID_VERSION=$(subst .,,$(shell pkg-config --modversion choreonoid-body-plugin))

all: cnoid hrp inter eigen tvmet step_tvmet step_eigen

cnoid: model.cpp
	$(CXX) `pkg-config choreonoid-body-plugin --cflags` -I../../ $< -o $@ -DUSE_CNOID_MODEL -DBOOST_SIGNALS_NO_DEPRECATION_WARNING `pkg-config choreonoid-body-plugin --libs` -DCNOID_VERSION=$(CNOID_VERSION)

hrp: model.cpp
	$(CXX) `pkg-config openhrp3.1 --cflags` -I../../ $< -o $@ -DUSE_HRP_MODEL -Wno-write-strings `pkg-config openhrp3.1 --libs`

inter: inter.cpp
	$(CXX)  -I../../ $< -o $@ -L../interpolator -lcreekInterpolator_cnoid

tvmet: matrix.cpp
	$(CXX)  -I../../ $< -o $@ -DUSE_HRP_MODEL

eigen: matrix.cpp
	$(CXX)  -I../../ `pkg-config eigen3 --cflags` $< -o $@ -DUSE_CNOID_MODEL

rinter: rinter.cpp
	$(CXX)  -I../../ `pkg-config eigen3 --cflags` $< -o $@ -DUSE_HRP_MODEL -L../interpolator -lcreekInterpolator_hrp

step_tvmet: step.cpp
	$(CXX)  -I../../ `pkg-config eigen3 --cflags` $< -o $@ -DUSE_HRP_MODEL -L../walk -lcreekWalk_hrp -L../interpolator -lcreekInterpolator_hrp

step_eigen: step.cpp
	$(CXX)  -I../../ `pkg-config eigen3 --cflags` `pkg-config choreonoid-body-plugin --cflags` $< -o $@ -DUSE_CNOID_MODEL -L../walk -lcreekWalk_cnoid -L../interpolator -lcreekInterpolator_cnoid `pkg-config choreonoid-body-plugin --libs`

clean:
	rm -f cnoid hrp tvmet eigen inter rinter step_tvmet step_eigen
	rm -f *~
