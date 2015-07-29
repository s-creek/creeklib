CXXFLAGS = -std=c++0x -I. `pkg-config opencv --cflags`
LDFLAGS  = -L. `pkg-config opencv --libs`

target: main video

libQrCodeDetector.so: creekQrCodeDetector.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) $< -c -o $@

main: main.cpp libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

video: video.cpp libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

zbar: zbar.cpp libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS) `pkg-config zbar --libs`

clean:
	rm -f *~
	rm -f main video zbar
	rm -f libQrCodeDetector.so