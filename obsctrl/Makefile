CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -lwebsockets -lssl -lcrypto -ljson-c

TARGET  = obs_stream_control

all: $(TARGET)

$(TARGET): obs_stream_control.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGET)

# Installation des dépendances (Debian/Ubuntu)
deps:
	sudo apt-get install -y libwebsockets-dev libssl-dev libjson-c-dev

.PHONY: all clean deps
