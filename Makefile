
CXX=$(CROSS_COMPILE)g++
CC=$(CROSS_COMPILE)gcc
#HOST_DIR=/home/linuxdev/prog/pluto/firm033/pluto_radar/plutosdr-fw/buildroot/output/host
FLAGS = -O2 -Wa,--noexecstack -Wl,-z,noexecstack -fpermissive -Wall -Wno-write-strings -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable -Wno-pointer-arith -Wno-format-zero-length -Wno-sign-compare -liio -mfpu=neon -mfloat-abi=hard

INC=-I./include_gse/gse
VER ?= $(shell cd $(CURDIR) && git describe --tags)
$(shell cd $(CURDIR) && git log --pretty=format:"%h - %ad : %s" > history.txt)

all: pluto_mqtt_ctrl pluto_stream  iio_ws_proxy

pluto_mqtt_ctrl: pluto_mqtt_ctrl.cpp mymqtt.h mqtthandlecommand.h mqtthandlecommand.cpp iiofshelper.cpp iiofshelper.h 
	$(TOOLS_PATH) $(CXX) $(FLAGS) -o pluto_mqtt_ctrl pluto_mqtt_ctrl.cpp mqtthandlecommand.cpp iiofshelper.cpp -lm -lrt -lpthread -lmosquitto 
pluto_stream: pluto_stream.cpp mymqtt.h mqtthandlestream.h mqtthandlestream.cpp iiofshelper.cpp dvbs2neon/dvbs2neon0v43.S tsinputmux.cpp tsinputmux.h gsemux.cpp gsemux.h iqtofft.h iqtofft.cpp ts_util/sdt.cpp ts_util/pcrpts.c dvbsarm/fec100.c dvbsarm/dvbsenco8.s
	$(TOOLS_PATH) $(CXX) $(INC) $(FLAGS) -DCOMIT_FW=\"$(VER)\" -o pluto_stream gsemux.cpp dvbs2neon/dvbs2neon0v43.S ts_util/sdt.cpp ts_util/pcrpts.c pluto_stream.cpp mqtthandlestream.cpp iiofshelper.cpp tsinputmux.cpp iqtofft.cpp dvbsarm/fec100.c dvbsarm/dvbsenco8.s -lm -lrt -lpthread -lmosquitto  -lgse	-lNE10 -lcivetweb
iio_ws_proxy: iio_ws_proxy.c
	$(TOOLS_PATH) $(CC) -O2 -mfpu=neon -mfloat-abi=hard iio_ws_proxy.c -o iio_ws_proxy -liio -lwebsockets -lpthread

clean:
	rm -f  pluto_mqtt_ctrl pluto_stream  iio_ws_proxy
