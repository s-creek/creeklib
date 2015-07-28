CXXFLAGS = -std=c++0x -I. `pkg-config opencv --cflags`
LDFLAGS  = -L. `pkg-config opencv --libs`

target: main

libQrCodeDetector.so: creekQrCodeDetector.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) $< -c -o $@

main: main.cpp libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

clean:
	rm -f *~
	rm -f main
	rm -f libQrCodeDetector.so