CROSS_COMPILE=arm-linux-gnueabihf-
SYSROOT=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/plutosdr-fw/buildroot/output/staging
CXX=$(CROSS_COMPILE)g++
CC=$(CROSS_COMPILE)gcc
HOST_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/host
FLAGS = -Wall -Wno-write-strings -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -liio -g -mfpu=neon --sysroot=$(SYSROOT) -mfloat-abi=hard
PAPR_ORI=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/pluto-buildroot/board/pluto/overlay/root
INC=-I/usr/local/include/gse

all: pluto_mqtt_ctrl pluto_stream bbgse 
pluto_mqtt_ctrl: pluto_mqtt_ctrl.cpp mymqtt.h mqtthandlecommand.h mqtthandlecommand.cpp iiofshelper.cpp iiofshelper.h
	$(CXX) $(FLAGS) -o pluto_mqtt_ctrl pluto_mqtt_ctrl.cpp mqtthandlecommand.cpp iiofshelper.cpp liboscimp_fpga_static.a -lm -lrt -lpthread -lmosquitto -ljansson
pluto_stream: pluto_stream.cpp mymqtt.h mqtthandlestream.h mqtthandlestream.cpp iiofshelper.cpp dvbs2neon/dvbs2neon0v40.S tsinputmux.cpp tsinputmux.h 
	$(CXX) $(FLAGS) -o pluto_stream dvbs2neon/dvbs2neon0v40.S pluto_stream.cpp mqtthandlestream.cpp iiofshelper.cpp tsinputmux.cpp -lm -lrt -lpthread -lmosquitto -ljansson	
bbgse: mygse/bbgse.c 
	$(CC) $(FLAGS) $(INC) -DPLUTO -o bbgse mygse/bbgse.c -lm -lrt -lpthread -lmosquitto -ljansson -liio -lgse
install: 
	cp bbgse $(PAPR_ORI)
	cp pluto_stream $(PAPR_ORI)
	cp pluto_mqtt_ctrl $(PAPR_ORI)
clean:
	rm -f  pluto_mqtt_ctrl pluto_stream bbgse
