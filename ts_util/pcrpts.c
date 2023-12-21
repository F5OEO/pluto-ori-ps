/*  
 * Copyright (C) 2004-2013  Lorenzo Pallara, l.pallara@avalpa.com 
 * Copyright (C) 2020  Evariste F5OEO
 * 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/*
* TODOLIST
* Fix correction for all bitrate (using modulo of divider)
* Add PCR only packet when Picture too heavy to have a pcr in the middle
* Change PTS when PTS<PCR
* Calculate drift of PCR vs original PCR
*/

#include <netinet/in.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <errno.h>
#include <ctype.h>
#include <sys/time.h>
#include <time.h>

#define TS_PACKET_SIZE 188
#define MAX_PCR 1000
#define SYSTEM_CLOCK_FREQUENCY 27000000ll
#define OUTPUT_BUFFER_IN_PACKETS 5000
// 250ms max
#define WINDOW_TIME_MS 200ll
#define WINDOW_AVERAGE_MAX (WINDOW_TIME_MS * SYSTEM_CLOCK_FREQUENCY / 1000ll)
#define PID_PCR 256
#define PID_AUDIO 257
// PCR/PTS 400ms
unsigned long long pcr_pts = 700ll;
//#define PCR_PTS_MS 700ll
#define PCR_PTS ((pcr_pts * SYSTEM_CLOCK_FREQUENCY) / 1000ll)

unsigned char null_ts_packet[TS_PACKET_SIZE];
//unsigned char ts_packet[TS_PACKET_SIZE]; /* TS packet */
unsigned char *ts_packet; /* TS packet */
unsigned char output_packet_buffer[OUTPUT_BUFFER_IN_PACKETS * TS_PACKET_SIZE];
unsigned char *current_output_packet = 0;
unsigned long long int ibits;
unsigned long long int obits = 0ll;
unsigned long long int fcounter;
unsigned long long int ucounter;
unsigned long long int last_ucounter;
unsigned long long int ts_packet_output;

unsigned long long int pid_pcr_table[MAX_PCR];		 /* PCR table for the TS packets */
unsigned long long int pid_pcr_index_table[MAX_PCR]; /* PCR index table for the TS packets */

unsigned long long int ts_packet_count = 0ll;

int NoPCR = 0;
unsigned long long int CalculatePCR = 0ll;
int synctopcr = 0;
bool discountinuity=false;

void set_timedts_pts(unsigned long long ts, unsigned char *buff)
{
	ts = ts / 300ll;

	buff[0] = buff[0] & 0xF0;
	buff[0] |= (((ts >> 30) << 1) & 0x0F) | 1; //3bits
	buff[1] = ts >> 22;						   //8
	buff[2] = ((ts >> 15) << 1) | 1;		   //8 bits
	buff[3] = (ts >> 7);					   //8bits
	buff[4] = ((ts) << 1) | 1;				   //7bits
}

unsigned long long parse_timedts_pts(unsigned char *buf)
{

	uint64_t ts;
	uint64_t pts1, pts2, pts3;
	unsigned int data0 = buf[0];
	unsigned int data1 = buf[1];
	unsigned int data2 = buf[2];
	unsigned int data3 = buf[3];
	unsigned int data4 = buf[4];
	int marker;

	pts1 = (data0 & 0x0E) >> 1;
	marker = data0 & 0x01;

	pts2 = (data1 << 7) | ((data2 & 0xFE) >> 1);
	marker = data2 & 0x01;

	pts3 = (data3 << 7) | ((data4 & 0xFE) >> 1);
	marker = data4 & 0x01;

	ts = (pts1 << 30) | (pts2 << 15) | pts3;

	ts = ts * 300ll;

	return ts;
}

unsigned char *GetPes(unsigned char *Packet)
{
	
	unsigned char AdaptationField = (Packet[3] >> 4) & 0x03;
	int ts_header_size = 4;
	unsigned char *pes = NULL;
	/* check adaptation field */
	if (AdaptationField == 0)
	{
		pes = nullptr;
		fprintf(stderr, "invalid packet!\n");
	}
	else if (AdaptationField == 1)
	{
		ts_header_size = 4; /* just payload */
		pes = Packet + ts_header_size;
	}
	else if (AdaptationField == 2)
	{
		ts_header_size += Packet[4] + 1; /* only adaptation field */
		pes = Packet + ts_header_size;
	}
	else if (AdaptationField == 3)
	{
		ts_header_size += Packet[4] + 1; /* jump the adaptation field */
		pes = Packet + ts_header_size;
	};
	//fprintf(stderr,"Adapt %d\n",AdaptationField);
	if(pes/*&&(AdaptationField==3)*/&&(pes[0]==0)&&(pes[1]==0)&&(pes[2]==1))
		return (pes);
	else
		return (nullptr);
}

char GetPTSFromPacket(unsigned char *Packet, unsigned long long *pts, unsigned long long *dts, int *PacketOffsetPTS, int *PacketOffsetDTS)
{
	*pts = 0ll;
	*dts = 0ll;
	*PacketOffsetPTS = 0;
	*PacketOffsetDTS = 0;

	unsigned char ts_header_size;

	unsigned char SkipAdapt = 0;
	char PTS_DTS_Flag = 0;

	unsigned char *pes = GetPes(Packet);
	
	
	if (pes) // There is a PES payload
	{
		/*
		for(int i=0;i<32;i++)
	{
		if(i%16==0) fprintf(stderr,"\n");
		fprintf(stderr,"%x ",pes[i]);
		
	}
	 fprintf(stderr,"\n");
	 */

		unsigned char streamid = pes[3];

		unsigned short pes_size = (pes[4] << 8) | pes[5];
		unsigned char PESScramb = pes[6];

		PTS_DTS_Flag = ((pes[7] >> 6) & 0x3);

		unsigned char pes_header_length = pes[8];

		if ((PTS_DTS_Flag == 2) || (PTS_DTS_Flag == 3)) //PTS
		{
			*pts = parse_timedts_pts(&pes[9]);
			*PacketOffsetPTS = (&pes[9] - Packet);
		}

		if ((PTS_DTS_Flag == 3)) /// PTS AND DTS
		{
			*dts = parse_timedts_pts(&pes[14]);
			*PacketOffsetDTS = (&pes[14] - Packet);
		}
	}
	else
	{
		PTS_DTS_Flag = 0;
		//fprintf(stderr, "no pes\n");
	}

	return PTS_DTS_Flag;
}

unsigned long long GetPCRFromPacket(unsigned char *Packet)
{
	unsigned long long pcr_base = 0ll;
	unsigned long long pcr_ext = 0ll;

	pcr_base = Packet[6];
	pcr_base = pcr_base << 25;
	pcr_base += (Packet[7] << 17);
	pcr_base += (Packet[8] << 9);
	pcr_base += (Packet[9] << 1);
	pcr_base += ((Packet[10] & 0x80) >> 7) & 0x1ll;
	pcr_ext = ((Packet[10] & 0x01) << 8) & 0x1FFll;
	pcr_ext += Packet[11];

	return (pcr_base * 300ull + pcr_ext);
}

void SetPacketPCR(unsigned char *Packet, unsigned long long pcr)
{
	unsigned long long CodingPCR = 0;
	unsigned long long CodingPCRExt = 0;

	CodingPCR = ((pcr) / 300ll) & 0x1FFFFFFFFll;
	CodingPCRExt = ((pcr) % 300ll);

	Packet[11] = (char)((CodingPCRExt)&0xFF);
	Packet[10] = (char)((((CodingPCR & 0x1) << 7) & 0xFF) | ((CodingPCRExt >> 8) & 0x1));
	Packet[9] = (char)(CodingPCR >> 1) & 0xFF;
	Packet[8] = (char)(CodingPCR >> 9) & 0xFF;
	Packet[7] = (char)(CodingPCR >> 17) & 0xFF;
	Packet[6] = (char)(CodingPCR >> 25) & 0xFF;
	if(discountinuity==true)
	{
		Packet[5]=Packet[5]|0x80;
		discountinuity=false; //reset discountinuity
	}
}

char PCRAvailable(char *Packet)
{
	return ((Packet[3] & 0x20) && (Packet[4] != 0) && (Packet[5] & 0x10));
}

unsigned short GetPid(char *Packet)
{
	return (((Packet[1] << 8) | Packet[2]) & 0x1fff);
}

void fill_buffer(void)
{

	/* copy ts packet from input to output buffer */
	memcpy(current_output_packet, ts_packet, TS_PACKET_SIZE);
	current_output_packet += TS_PACKET_SIZE;
	ts_packet_count++;
	if (current_output_packet >= output_packet_buffer + TS_PACKET_SIZE * OUTPUT_BUFFER_IN_PACKETS)
	{
		fprintf(stderr, "buffer too small\n");
		exit(2);
	}
}

unsigned long long GetInstantBitrate(int index_pcr)
{
	unsigned long long Bitrate = 0ll;
	unsigned long long PcrStart = pid_pcr_table[index_pcr];
	unsigned long long PcrStop = pid_pcr_table[index_pcr + 1];
	unsigned long long NbBit = (pid_pcr_index_table[index_pcr + 1] - pid_pcr_index_table[index_pcr]) * 8;
	if (PcrStop > (PcrStart + (10 * SYSTEM_CLOCK_FREQUENCY) / 1000ll))
		Bitrate = (NbBit * SYSTEM_CLOCK_FREQUENCY) / ((PcrStop - PcrStart));
	return Bitrate;
}

unsigned long long GetTimeFrame(int index_pcr)
{
	unsigned long long timeframe = 0;
	unsigned long long PcrStart = pid_pcr_table[index_pcr];
	unsigned long long PcrStop = pid_pcr_table[index_pcr + 1];

	if (PcrStop > (PcrStart + (1 * SYSTEM_CLOCK_FREQUENCY) / 1000ll))
		timeframe = (1000 * (PcrStop - PcrStart) / SYSTEM_CLOCK_FREQUENCY);
	return timeframe;
}

unsigned long long GetTimeMs(int index_pcr)
{

	unsigned long long NbBit = (pid_pcr_index_table[index_pcr + 1] - pid_pcr_index_table[index_pcr]) * 8;
	unsigned long long TimeMs = (1000ll * NbBit) / obits;
	return TimeMs;
}

unsigned long long GetGop(int index_pcr)
{
	static int FrameNumber = 0;
	static unsigned long long Gop = 0;
	if (GetTimeFrame(index_pcr) == 0)
	{
		if (FrameNumber != 0)
		{
			Gop = FrameNumber;
		}
		FrameNumber = 0;
	}
	else
	{
		FrameNumber++;
	}

	return Gop;
}

unsigned long long RemainClock = 0;

unsigned long long IncrementClock(unsigned long long bit)
{
	
	unsigned long long ClkInc = (bit * SYSTEM_CLOCK_FREQUENCY) / obits;
	unsigned long ClkMod = (bit * SYSTEM_CLOCK_FREQUENCY) % obits;
	RemainClock += ClkMod;

	CalculatePCR += (ClkInc + ClkMod / obits);
	RemainClock = RemainClock % obits;

	return CalculatePCR;
}

unsigned long long IncrementPacketClock()
{
	return IncrementClock(188 * 8);
}

signed long long BitPadding = 0;
unsigned long long NbOuputBitWindowRest = 0;

signed long long CalculateBitPadding()
{

	unsigned long long TimeWindow = pid_pcr_table[NoPCR] - pid_pcr_table[0];
	unsigned long long NbInputBitWindow = (pid_pcr_index_table[NoPCR]) * 8; //Bit incoming
	unsigned long long NbOuputBitWindow = (obits * TimeWindow) / SYSTEM_CLOCK_FREQUENCY;

	NbOuputBitWindowRest += (obits * TimeWindow) % SYSTEM_CLOCK_FREQUENCY;
	BitPadding += ((signed long long)(NbOuputBitWindow + NbOuputBitWindowRest / SYSTEM_CLOCK_FREQUENCY) - (signed long long)NbInputBitWindow);
	NbOuputBitWindowRest = NbOuputBitWindowRest % SYSTEM_CLOCK_FREQUENCY;

	return BitPadding;
}

int PacketPaddingMod = 0;
unsigned long CalculatePacketPadding()
{
	signed long long BitPadding = CalculateBitPadding();

	unsigned long PacketPadding = 0;
	if (BitPadding > 0)
	{
		PacketPaddingMod += BitPadding % (TS_PACKET_SIZE * 8);
		PacketPadding = (BitPadding + PacketPaddingMod) / (TS_PACKET_SIZE * 8);
		PacketPaddingMod = PacketPaddingMod % (TS_PACKET_SIZE * 8);
	}
	else
	{
		PacketPadding = 0;
	}
	//fprintf(stderr, "Packet Padding %d Mod %d Bitpadding %lld\n", PacketPadding, PacketPaddingMod, BitPadding);
	return PacketPadding;
}

void InsertPacketPadding()
{
	BitPadding -= TS_PACKET_SIZE * 8;
	IncrementPacketClock();
	fwrite(null_ts_packet, TS_PACKET_SIZE, 1, stdout);
}




long GetMilliOfDay()
{
	struct timeval current_time,midnight_time;
	gettimeofday(&current_time, NULL);
	struct tm *todaymidnight;
	todaymidnight=localtime(&current_time.tv_sec);
	todaymidnight->tm_hour=0;
	todaymidnight->tm_min=0;
	todaymidnight->tm_sec=0;

	midnight_time.tv_sec = mktime(todaymidnight);
	midnight_time.tv_usec = 0;

	long mssincemidnight=(current_time.tv_sec-midnight_time.tv_sec)*1000L+current_time.tv_usec/1000L;
	return mssincemidnight;
}


static int64_t PCRReference=0;
static int64_t ClockTimeReference=0;
static int64_t OldPCR=0;

uint64_t GetPcrTime(uint64_t PCR)
{
		
	int64_t ClockTime=GetMilliOfDay();
	
	if((OldPCR==0)||(abs(OldPCR-(int64_t)PCR)>27000LL*100LL)) // pcr period too long 100ms
	{
		
		PCRReference=PCR;
		ClockTimeReference=ClockTime;
		fprintf(stderr,"clktimereference = %lld pcrref %lld\n",ClockTimeReference,PCRReference);
	}
	OldPCR=PCR;
	//fprintf(stderr,"pcrtime %lld\n",PCR-PCRReference+ClockTimeReference*27000LL);
	return (PCR-PCRReference+ClockTimeReference*27000LL);
}

uint64_t GetPtsDtsTime(uint64_t pts_dts)
{
	return (pts_dts-PCRReference+ClockTimeReference*27000LL);
}

long GetLatencyTransmission(uint64_t PCR)
{
	return (GetMilliOfDay()-PCR/27000LL); 
}

void ProcessCorectPCR(uint8_t *Buffer, size_t BUFF_MAX_SIZE)
{
    uint8_t *cur_packet = Buffer;
    static unsigned long long pts, oldpts;
    unsigned long long dts;
    static unsigned long long pcr, newpcr;
    long long video_delay;
    int PacketOffsetPTS;
    int PacketOffsetDTS;
    char flag = 0;
    static unsigned long long offset_time_clk=4*27000000LL;
    for (size_t i = 0; i < BUFF_MAX_SIZE; i += 188)
    {
        if (GetPid((char *)cur_packet) == 256) // video
        {

            // if(PCRAvailable(cur_packet))
            {
                if (PCRAvailable((char *)cur_packet))
                {
                    
                    pcr = GetPCRFromPacket(cur_packet);
					newpcr = GetPcrTime(pcr);
                    //fprintf(stderr, "PCR %llu \n", pcr);
                    SetPacketPCR(cur_packet,newpcr);
                    
                }
                flag = GetPTSFromPacket(cur_packet, &pts, &dts, &PacketOffsetPTS, &PacketOffsetDTS);
                if (flag == 2) // PTS
                {
                    
                   if (PacketOffsetPTS)
                    {
                        set_timedts_pts(GetPtsDtsTime(pts), cur_packet+PacketOffsetPTS);
                        //fprintf(stderr, "VPTS %llu \n", pts);
                    }
                }
                if (flag == 3) // PTS/DTS
                {

                    if (PacketOffsetPTS)
                    {
                        set_timedts_pts(GetPtsDtsTime(pts), cur_packet+PacketOffsetPTS);
                        //fprintf(stderr, "VPTS %llu \n", pts);
                    }

                    if (PacketOffsetDTS)
                    {
                        //fprintf(stderr, "VDTS %llu \n", dts);
                         set_timedts_pts(GetPtsDtsTime(dts),cur_packet+ PacketOffsetDTS);
                    }
                }
            }
        }
        else if (GetPid((char *)cur_packet) == 257) // audio
        {
            flag = GetPTSFromPacket(cur_packet, &pts, &dts, &PacketOffsetPTS, &PacketOffsetDTS);
            if (flag == 2) // PTS
            {
               if (PacketOffsetPTS)
                {
                    //fprintf(stderr, "APPTS %llu \n", dts);
                    set_timedts_pts(GetPtsDtsTime(pts),cur_packet+ PacketOffsetPTS);
                }
                //fprintf(stderr, "Warning :: PTS in audio APTS %llu ADTS %llu \n", pts,dts);
            }
            if (flag == 3) // DTS
            {
                if (PacketOffsetDTS)
                {
                    //fprintf(stderr, "ADTS %llu \n", dts);
                    set_timedts_pts(GetPtsDtsTime(dts),cur_packet+ PacketOffsetDTS);
                }    
            }
        }
        else
        {
        }
        cur_packet += 188;
    }
}

void ProcessTSTiming(uint8_t *Buffer, size_t BUFF_MAX_SIZE,size_t *video_delay,size_t *audio_delay,long *TransDelay)
{

    uint8_t *cur_packet = Buffer;
for (size_t i = 0; i < BUFF_MAX_SIZE; i += 188)
    {
       
        static long long pts, oldpts;
        static  long long pcr, oldpcr;
        unsigned long long dts;

        int PacketOffsetPTS;
        int PacketOffsetDTS;
        char flag = 0;
        if (GetPid((char *)cur_packet) == 256) // video
        {

            if (PCRAvailable((char *)cur_packet)) // Just take when PCR to low cpu
            {
                pcr = GetPCRFromPacket(cur_packet);
				//fprintf(stderr,"TxDelay %ld\n",GetLatencyTransmission(pcr-27000L*600L));
				long TxDelay=GetLatencyTransmission(pcr);
				if(abs(TxDelay)<10000L) // if more than 10s , this is not a DVB2 compliant device
				{
					*TransDelay=GetLatencyTransmission(pcr);
				}
				else
				{
					*TransDelay=0;
				}
                flag = GetPTSFromPacket(cur_packet, (unsigned long long *)&pts, (unsigned long long *)&dts, &PacketOffsetPTS, &PacketOffsetDTS);
                if (flag == 2) // PTS
                {

                    *video_delay = (pts - pcr) / 27000LL;

                    // fprintf(stderr, "Video PCR/PTS %lld \n", video_delay / 27000LL);

                    
                }
                if (flag == 3) // PTS/DTS
                {

                    *video_delay = (pts - pcr) / 27000LL;

                    // fprintf(stderr, "Video PCR/PTS %lld \n", video_delay / 27000LL);

                    
                }
            }
        }
        else if (GetPid((char *)cur_packet) == 257) // audio
        {
            flag = GetPTSFromPacket(cur_packet, (unsigned long long *)&pts, (unsigned long long *)&dts, &PacketOffsetPTS, &PacketOffsetDTS);
            if (flag == 2) // PTS
            {

                *audio_delay = (pts - pcr) / 27000LL;
            }
            if (flag == 3) // PTS/DTS
            {
                *audio_delay = (pts - pcr) / 27000LL;
            }
        }
        cur_packet += 188;
    }
}