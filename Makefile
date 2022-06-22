CROSS_COMPILE=arm-linux-gnueabihf-
SYSROOT=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/staging
CXX=$(CROSS_COMPILE)g++
HOST_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/host
FLAGS = -Wall -Wno-write-strings -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -liio -g -O2 -mfpu=neon --sysroot=$(SYSROOT) -mfloat-abi=hard
PAPR_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/radarfrm/board/pluto/overlay/root

all: pluto_mqtt_ctrl pluto_stream  
pluto_mqtt_ctrl: pluto_mqtt_ctrl.cpp mymqtt.h mqtthandlecommand.h mqtthandlecommand.cpp iiofshelper.cpp iiofshelper.h
	$(CXX) $(FLAGS) -o pluto_mqtt_ctrl pluto_mqtt_ctrl.cpp mqtthandlecommand.cpp iiofshelper.cpp -lm -lrt -lpthread -lmosquitto -ljansson
pluto_stream: pluto_stream.cpp mymqtt.h mqtthandlestream.h mqtthandlestream.cpp iiofshelper.cpp
	$(CXX) $(FLAGS) -o pluto_stream pluto_stream.cpp mqtthandlestream.cpp iiofshelper.cpp -lm -lrt -lpthread -lmosquitto -ljansson	

	
clean:
	rm -f  pluto_mqtt_ctrl pluto_stream
