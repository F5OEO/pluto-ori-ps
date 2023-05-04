CROSS_COMPILE=arm-linux-gnueabihf-
SYSROOT=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/plutosdr-fw/buildroot/output/staging
CXX=$(CROSS_COMPILE)g++
CC=$(CROSS_COMPILE)gcc
HOST_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/host
FLAGS = -fpermissive -Wall -Wno-write-strings -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -liio -g -mfpu=neon --sysroot=$(SYSROOT) -mfloat-abi=hard
PAPR_ORI=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/pluto-buildroot/board/pluto/overlay/root
INC=-I/usr/local/include/gse

all: pluto_mqtt_ctrl pluto_stream 
pluto_mqtt_ctrl: pluto_mqtt_ctrl.cpp mymqtt.h mqtthandlecommand.h mqtthandlecommand.cpp iiofshelper.cpp iiofshelper.h
	$(CXX) $(FLAGS) -o pluto_mqtt_ctrl pluto_mqtt_ctrl.cpp mqtthandlecommand.cpp iiofshelper.cpp liboscimp_fpga_static.a -lm -lrt -lpthread -lmosquitto -ljansson
pluto_stream: pluto_stream.cpp mymqtt.h mqtthandlestream.h mqtthandlestream.cpp iiofshelper.cpp dvbs2neon/dvbs2neon0v43.S tsinputmux.cpp tsinputmux.h gsemux.cpp gsemux.h iqtofft.h iqtofft.cpp
	$(CXX) $(INC) $(FLAGS) -o pluto_stream gsemux.cpp dvbs2neon/dvbs2neon0v43.S pluto_stream.cpp mqtthandlestream.cpp iiofshelper.cpp tsinputmux.cpp iqtofft.cpp -lm -lrt -lpthread -lmosquitto -ljansson -lgse	-lNE10 -lcivetweb
bbgse: mygse/bbgse.c 
	$(CC) $(FLAGS) $(INC) -DPLUTO -o bbgse mygse/bbgse.c -lm -lrt -lpthread -lmosquitto -ljansson -liio -lgse
install: 
	
	cp pluto_stream $(PAPR_ORI)
	cp pluto_mqtt_ctrl $(PAPR_ORI)
	cp qo100initdvb.sh $(PAPR_ORI)
	cp gain.sh $(PAPR_ORI)
	cp mute.sh $(PAPR_ORI)
	cp unmute.sh $(PAPR_ORI)
	cp settxmode.sh $(PAPR_ORI)
	cp initdvb.sh $(PAPR_ORI)

clean:
	rm -f  pluto_mqtt_ctrl pluto_stream bbgse
