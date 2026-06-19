CXX     = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++17
LDFLAGS  = -lwebsockets -lssl -lcrypto

TARGET  = obs_stream_control

all: $(TARGET)

$(TARGET): obs_stream_control.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

# Installation des dépendances (Debian/Ubuntu)
deps:
	sudo apt-get install -y libwebsockets-dev libssl-dev nlohmann-json3-dev

.PHONY: all clean deps
