CXXFLAGS = -std=c++0x -I. `pkg-config opencv --cflags`
LDFLAGS  = -L. `pkg-config opencv --libs`

target: libQrCodeDetector.so main video

libQrCodeDetector.so: creekQrCodeDetector.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) -fPIC -shared $< -o $@

main: main.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lQrCodeDetector

video: video.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lQrCodeDetector

zbar: zbar.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS) -lQrCodeDetector `pkg-config zbar --libs`

clean:
	rm -f *~
	rm -f main video zbar
	rm -f libQrCodeDetector.so