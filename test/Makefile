target: cnoid hrp inter

cnoid: test.cpp
	$(CXX) `pkg-config choreonoid-body-plugin --cflags --libs` -I../../ $< -o $@ -DUSE_CNOID_MODEL -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 

hrp: test.cpp
	$(CXX) `pkg-config openhrp3.1 --cflags --libs` -I../../ $< -o $@ -DUSE_HRP_MODEL -Wno-write-strings

inter: inter.cpp
	$(CXX)  -I../../ $< -o $@ -L../interpolator -lcreekInterpolator

clean:
	rm -f cnoid hrp
	rm -f *~