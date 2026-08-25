CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra

.PHONY: all test clean

all: test

data/manifest.txt:
	python3 gen_streams.py

test_parser: test_mjpeg_parser.cpp ../components/ha_live_camera/mjpeg_parser.cpp ../components/ha_live_camera/mjpeg_parser.h
	$(CXX) $(CXXFLAGS) -o $@ test_mjpeg_parser.cpp ../components/ha_live_camera/mjpeg_parser.cpp

test: data/manifest.txt test_parser
	./test_parser

clean:
	rm -f test_parser
	rm -rf data
