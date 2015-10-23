CXXFLAGS = -std=c++0x -I. `pkg-config opencv --cflags`
LDFLAGS  = -L. `pkg-config opencv --libs`

target: main video libQrCodeDetector.so

libQrCodeDetector.so: creekQrCodeDetector.cpp creekQrCodeDetector.h
	$(CXX) $(CXXFLAGS) -fPIC -shared $< -o $@

main: main.cpp creekQrCodeDetector.h libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

video: video.cpp creekQrCodeDetector.h libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

rect: rect.cpp creekQrCodeDetector.h libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS)

zbar: zbar.cpp creekQrCodeDetector.h libQrCodeDetector.so
	$(CXX) $(CXXFLAGS) $< -o $@ -lQrCodeDetector $(LDFLAGS) `pkg-config zbar --libs`

clean:
	rm -f *~
	rm -f main video zbar
	rm -f libQrCodeDetector.so