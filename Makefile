#FOR 0.33
#CROSS_COMPILE=arm-linux-gnueabihf-
#SYSROOT=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/plutosdr-fw/buildroot/output/staging
#PAPR_ORI=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/pluto-buildroot/board/pluto/overlay/root
#FOR 0.37
CROSS_COMPILE=arm-linux-
SYSROOT=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/37plutosdr-fw/buildroot/output/staging
PAPR_ORI=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/pluto-buildroot/board/pluto/overlay/root
PAPR_WWW=/home/linuxdev/prog/pluto/pluto-ori/pluto-ori-frm/pluto-buildroot/board/pluto/overlay/www

CXX=$(CROSS_COMPILE)g++
CC=$(CROSS_COMPILE)gcc
#HOST_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/host
FLAGS = -fpermissive -Wall -Wno-write-strings -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -liio -g -mfpu=neon --sysroot=$(SYSROOT) -mfloat-abi=hard

INC=-I./include_gse/gse
VER=$(shell git describe --tags)

all: pluto_mqtt_ctrl pluto_stream 

pluto_mqtt_ctrl: pluto_mqtt_ctrl.cpp mymqtt.h mqtthandlecommand.h mqtthandlecommand.cpp iiofshelper.cpp iiofshelper.h
	$(CXX) $(FLAGS) -o pluto_mqtt_ctrl pluto_mqtt_ctrl.cpp mqtthandlecommand.cpp iiofshelper.cpp liboscimp_fpga_static.a -lm -lrt -lpthread -lmosquitto -ljansson
pluto_stream: pluto_stream.cpp mymqtt.h mqtthandlestream.h mqtthandlestream.cpp iiofshelper.cpp dvbs2neon/dvbs2neon0v43.S tsinputmux.cpp tsinputmux.h gsemux.cpp gsemux.h iqtofft.h iqtofft.cpp ts_util/sdt.cpp
	$(CXX) $(INC) $(FLAGS) -DCOMIT_FW=\"$(VER)\" -o pluto_stream gsemux.cpp dvbs2neon/dvbs2neon0v43.S ts_util/sdt.cpp pluto_stream.cpp mqtthandlestream.cpp iiofshelper.cpp tsinputmux.cpp iqtofft.cpp -lm -lrt -lpthread -lmosquitto -ljansson -lgse	-lNE10 -lcivetweb
install: 
	
	cp pluto_stream $(PAPR_ORI)
	cp pluto_mqtt_ctrl $(PAPR_ORI)
	cp qo100initdvb.sh $(PAPR_ORI)
	cp gain.sh $(PAPR_ORI)
	cp mute.sh $(PAPR_ORI)
	cp unmute.sh $(PAPR_ORI)
	cp settxmode.sh $(PAPR_ORI)
	cp initdvb.sh $(PAPR_ORI)
	cp mqtt_ifconfig.sh $(PAPR_ORI)
	cp mqtt_iptable.sh $(PAPR_ORI)
	cp mqtt_reboot.sh $(PAPR_ORI)
	cp mqtt_setcall.sh $(PAPR_ORI)
	cp mqtt_longmynd.sh $(PAPR_ORI)
	cp passthrough.sh $(PAPR_ORI)
	cp agctest.sh $(PAPR_ORI)
	cp watchconsoletx.sh $(PAPR_ORI)
	./makepatern.sh
	cp mire.ts $(PAPR_ORI)
	cp www/* -r $(PAPR_WWW)
clean:
	rm -f  pluto_mqtt_ctrl pluto_stream bbgse
