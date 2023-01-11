PandaBusLibs_PATH = /opt/PandaBusLibs_18.04
CXX = g++
CXXFLAGS = -I$(PandaBusLibs_PATH)/include/opencv-4.1.0/opencv4/ -L$(PandaBusLibs_PATH)/lib/opencv-4.1.0 -L$(PandaBusLibs_PATH)/lib/opencv-4.1.0_cuda-11.1/ -lopencv_core -lopencv_imgproc -Wl,-rpath=$(PandaBusLibs_PATH)/lib/opencv-4.1.0_cuda-11.1/ -lopencv_imgcodecs


all: main

main: main.cpp
	$(CXX) main.cpp -o main $(CXXFLAGS)

run:
	export=LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(PandaBusLibs_PATH)/lib/opencv-4.1.0_cuda-11.1/
	./main

clean:
	rm -f main
