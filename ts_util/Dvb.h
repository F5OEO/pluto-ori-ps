#pragma once

#include "tp.h"


typedef struct {
	short re;
	short im;
}scmplx;

typedef enum { HW_DATV_EXPRESS = 1, HW_LIME_SDR, HW_ADALM_PLUTO, HW_FMCOMMSx }SdrHwType;



// Buffering of calback data

typedef struct{
	uint64_t time;
	int len;
	uint8_t *b;
}SampleBuffer;

typedef unsigned int u32;
typedef unsigned char u8;

#define TP_LEN 188
#define DISP_MODE_PAL 0
#define DISP_MODE_NTSC 1
#define PES_PAYLOAD_LENGTH 184
// PIDS
#define PAT_PID  0x0000
#define NIT_PID  0x0010
#define SDT_PID  0x0011
#define EIT_PID  0x0012
#define TDT_PID  0x0014
#define NULL_PID 0x1FFF

#define P1_MAP_PID  0xFFF
#define P1_VID_PID  256
#define P1_AUD_PID  257
#define P1_PCR_PID  0x1FFF
#define P1_DATA_PID 258

#define DEFAULT_NETWORK_ID 1
#define DEFAULT_STREAM_ID  P1_MAP_PID
#define DEFAULT_SERVICE_ID P1_MAP_PID
#define DEFAULT_PROGRAM_NR P1_MAP_PID

#define S_USE_VIDEO_DEVICE "Use Video Device"
#define CRC_32_LEN 4

