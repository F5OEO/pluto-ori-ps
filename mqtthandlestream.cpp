/*
  ===========================================================================

  Copyright (C) 2022 Evariste F5OEO


  PLUTO_DVB is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  PLUTO_DVB is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License LIMfor more details.

  You should have received a copy of the GNU General Public License
  along with PLUTO_DVB.  If not, see <http://www.gnu.org/licenses/>.

  ===========================================================================
*/
#define FPGA 1
//

#include <stdio.h>
#include <stdio_ext.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>

#include <getopt.h>
#include <ctype.h>
#include <termios.h>

#include <pthread.h>
#include <iio.h>
#include "iio-private.h"
#include <jansson.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//#include "ad9363.h"

#include "mqtthandlestream.h"
#include "dsp.h"
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "iiofshelper.h"

using namespace std;

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};
#define UDP_BUFF_MAX_SIZE (1472)
char sCmdRoot[255];
char sDtRoot[255];
char m_addport[255];
struct mosquitto *m_mosq;

// RX Variable
FILE *fdout = stdout;
char m_iface[255];
int PipeSize = 0;
size_t burstsizerx = 0;
size_t average = 1;
int m_format = 0;
int fftsize = 2048;
size_t BufferLenrx = 0;

// TX Variable
FILE *fdin = stdin;

size_t burstsizetx = 0;

int m_formattx = 0;

size_t BufferLentx = 0;

// ************ Rx Thread *********************
static pthread_t m_tid[1];
static bool RunRx = true;
pthread_mutex_t buffer_mutexrx;

// ************ Tx Thread *********************
static pthread_t m_tidtx[1];
static bool RunTx = true;
pthread_mutex_t buffer_mutextx;

// *********** Channels variables *******************
static char tmpstr[64];
static struct iio_context *m_ctx = NULL;
static struct iio_device *m_dev = NULL;
//Channels variable
static struct iio_buffer *m_rxbuf = NULL;
static struct iio_buffer *m_txbuf = NULL;

struct iio_device *m_tx = NULL;
static struct iio_channel *m_tx0_i = NULL;
static struct iio_channel *m_tx0_q = NULL;

struct iio_device *m_rx = NULL;
static struct iio_channel *m_rx0_i = NULL;
static struct iio_channel *m_rx0_q = NULL;

static int m_max_len; // Maximum size of the buffer
static int m_offset;  // Current offset into the buffer

/* Some usefull variabe to get */

size_t m_latency = 20000; //Latency at 80ms by default
size_t Underflow = 0;
size_t m_SR = 3000000;

int m_sock;
struct sockaddr_in m_client;

size_t m_latencytx = 20000; //Latency at 80ms by default
size_t Underflowtx = 0;
size_t m_SRtx = 3000000;


enum
{
    output_stdout,
    output_udp
};
size_t typeouput = output_stdout;

void ResetDVBS2()
{
    
    size_t value = ReadRegister(0x79020000+0x40BC);
  
    WriteRegister(0x79020000+0x40BC, (value & 0xFFF1) | 0);
 
    WriteRegister(0x79020000+0x40BC, (value & 0xFFF1) | 2);
   

}

/* helper function generating channel names */
static char *get_ch_name(const char *type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d)
    {
    case TX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        return *dev != NULL;
    case RX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
        return *dev != NULL;
    default:
        return false;
    }
}

void udp_set_ip(const char *ip, const char *iface)
{
    char text[40];
    char *add[2];
    u_int16_t sock;

    strcpy(text, ip);
    add[0] = strtok(text, ":");
    add[1] = strtok(NULL, ":");
    if (strlen(add[1]) == 0)
        sock = 1314;
    else
        sock = atoi(add[1]);
    // Construct the client sockaddr_in structure
    memset(&m_client, 0, sizeof(m_client));       // Clear struct
    m_client.sin_family = AF_INET;                // Internet/IP
    m_client.sin_addr.s_addr = inet_addr(add[0]); // IP address
    m_client.sin_port = htons(sock);              // server socket

    strcpy(m_addport, ip);

    struct in_addr localInterface;
    localInterface.s_addr = inet_addr(iface);
    setsockopt(m_sock, IPPROTO_IP, IP_MULTICAST_IF, (char *)&localInterface, sizeof(localInterface));
}

inline void udp_send(char *b, int len)
{

    
    int index = 0;
    for (index = 0; index < len; index += UDP_BUFF_MAX_SIZE)
    {

        sendto(m_sock, b + index, UDP_BUFF_MAX_SIZE, MSG_ZEROCOPY, (struct sockaddr *)&m_client, sizeof(m_client));
    }
}

void udp_init(void)
{
    // Create a socket for transmitting UDP TS packets
    if ((m_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
    {
        fprintf(stderr, "Failed to create socket\n");
        return;
    }
    int one;
    //#define SO_ZEROCOPY 60

    if (setsockopt(m_sock, SOL_SOCKET, SO_ZEROCOPY, &one, sizeof(one)))
    {
        fprintf(stderr, "UDP zerocopy mode failed\n");
    }

    //SO_ZEROCOPY
    //udp_set_ip("192.168.1.39:10000");
}

#define LOOPBACK_DISABLE 0
#define LOOPBACK_TX2RX 1
#define LOOPBACK_RX2TX 2
void fmc_set_loopback(bool enable,int Type)
{
	//Type=LOOPBACK_RX2TX
    if(enable)
	    iio_device_debug_attr_write_longlong(m_rx,"loopback",Type);
    else
    {
        iio_device_debug_attr_write_longlong(m_rx,"loopback",LOOPBACK_DISABLE);
    }
}

void InitRxChannel(size_t len, unsigned int nbBuffer)
{
    if (m_ctx == NULL)
        m_ctx = iio_create_local_context();
    if (m_ctx == NULL)
        fprintf(stderr, "Init context failn");
    iio_context_set_timeout(m_ctx, 0);

    get_ad9361_stream_dev(m_ctx, RX, &m_rx);
    //fprintf(stderr,"* Initializing AD9361 IIO streaming channels\n");
    get_ad9361_stream_ch(RX, m_rx, 0, &m_rx0_i);
    get_ad9361_stream_ch(RX, m_rx, 1, &m_rx0_q);

    fmc_set_loopback(true,LOOPBACK_TX2RX);

    fprintf(stderr, "Rcv Stream with %u buffers of %d samples\n", nbBuffer, len);
    // Change the size of the buffer
    m_max_len = len;

    if (m_rxbuf)
    {
        iio_channel_disable(m_rx0_i); // Fix the bug https://github.com/analogdevicesinc/libiio/commit/02527e69ab57aa2eac995e964b58421b0f5af5ad
        iio_channel_disable(m_rx0_q);
        iio_buffer_destroy(m_rxbuf);
        m_rxbuf = NULL;
    }

    iio_device_set_kernel_buffers_count(m_rx, nbBuffer); // SHould be called BEFORE create_buffer (else not setting)

    //	printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(m_rx0_i);
    iio_channel_enable(m_rx0_q);

    m_rxbuf = iio_device_create_buffer(m_rx, len, false);

    if (m_rxbuf == NULL)
    {
        fprintf(stderr, "Could not allocate iio mem\n");
        //exit(1);
    }

    iio_buffer_set_blocking_mode(m_rxbuf, true);

    int ret = iio_device_reg_write(m_rx, 0x80000088, 0x4);
    if (ret)
    {
        fprintf(stderr, "Failed to clearn DMA status register: %s\n",
                strerror(-ret));
    }
}

void InitTxChannel(size_t len, unsigned int nbBuffer)
{
    if (m_ctx == NULL)
        m_ctx = iio_create_local_context();
    if (m_ctx == NULL)
        fprintf(stderr, "Init context failn");
    iio_context_set_timeout(m_ctx, 0);

    get_ad9361_stream_dev(m_ctx, TX, &m_tx);
    //fprintf(stderr,"* Initializing AD9361 IIO streaming channels\n");
    get_ad9361_stream_ch(TX, m_tx, 0, &m_tx0_i);
    get_ad9361_stream_ch(TX, m_tx, 1, &m_tx0_q);

    fprintf(stderr, "Tx Stream with %u buffers of %d samples\n", nbBuffer, len);
    // Change the size of the buffer
    //m_max_len = len;

    if (m_txbuf)
    {
        iio_channel_disable(m_tx0_i); // Fix the bug https://github.com/analogdevicesinc/libiio/commit/02527e69ab57aa2eac995e964b58421b0f5af5ad
        iio_channel_disable(m_tx0_q);
        iio_buffer_destroy(m_txbuf);
        m_txbuf = NULL;
    }

    iio_device_set_kernel_buffers_count(m_tx, nbBuffer); // SHould be called BEFORE create_buffer (else not setting)

    //	printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(m_tx0_i);
    iio_channel_enable(m_tx0_q);

    m_txbuf = iio_device_create_buffer(m_tx, len, false);

    if (m_txbuf == NULL)
    {
        fprintf(stderr, "Could not allocate iio mem tx\n");
        //exit(1);
    }

    iio_buffer_set_blocking_mode(m_txbuf, true);

    /*
    int ret = iio_device_reg_write(m_tx, 0x80000088, 0x4);
    if (ret)
    {
        fprintf(stderr, "Failed to clearn Tx DMA status register: %s\n",
                strerror(-ret));
    }
    */
}

void InitRxChannel(size_t LatencyMicro)
{

    m_latency = LatencyMicro;
    pthread_mutex_lock(&buffer_mutexrx);

    if (m_format == 0) // CS16
    {
        BufferLenrx = LatencyMicro * (m_SR / 1e6); // 12 because FFT transform
        if (BufferLenrx > UDP_BUFF_MAX_SIZE)
            BufferLenrx -= BufferLenrx % UDP_BUFF_MAX_SIZE;
        InitRxChannel(BufferLenrx, 4);
        

        

        
    }
    if (m_format == 1) // CS8
    {
        BufferLenrx = LatencyMicro * (m_SR / 2 / 1e6); // 12 because FFT transform
        if (BufferLenrx > UDP_BUFF_MAX_SIZE)
            BufferLenrx -= BufferLenrx % UDP_BUFF_MAX_SIZE;
        InitRxChannel(BufferLenrx, 4);
    }
    if (m_format == 2) // FFT
    {

        BufferLenrx = 600 * (m_SR / 1e6);     // 12 because FFT transform
        BufferLenrx -= BufferLenrx % fftsize; // FixMe shoudl be relataed to mode (fft or iq)

        if (burstsizerx > BufferLenrx)
        {
            BufferLenrx = burstsizerx;
        }
        if(BufferLenrx<fftsize)
        {
           BufferLenrx= fftsize;
        }
        InitRxChannel(BufferLenrx, 4);
    }

    if (burstsizerx != 0)
    {

        //BufferLenrx = burstsizerx*16;
        int ret = fcntl(fileno(fdout), F_SETPIPE_SZ, burstsizerx * 2 * sizeof(short));
        PipeSize = fcntl(fileno(fdout), F_GETPIPE_SZ);
    }
    else
    {
        int ret = fcntl(fileno(fdout), F_SETPIPE_SZ, BufferLenrx * 2 * sizeof(short));
        PipeSize = fcntl(fileno(fdout), F_GETPIPE_SZ);
    }

    Underflow = 0;

    fprintf(stderr, "New pipe size %d burstsizerx=%d\n", PipeSize, burstsizerx);
    pthread_mutex_unlock(&buffer_mutexrx);
}

void InitTxChannel(size_t LatencyMicro)
{

    //m_latency = LatencyMicro;
    pthread_mutex_lock(&buffer_mutextx);

    if (m_formattx == 0) // CS16
    {
        BufferLentx = LatencyMicro * (m_SRtx / 1e6); // 12 because FFT transform
        //if (BufferLentx > UDP_BUFF_MAX_SIZE)
        //    BufferLentx -= BufferLentx % UDP_BUFF_MAX_SIZE;
        //BufferLentx=3072+1; //3072+1 for fec 1/4_short IS WORKING,3072/2+1 OK, BUT NOT 3072/4+1 
        //BufferLentx=3072/4+1; //3072 for fec 1/4_short

        BufferLentx=(1+(58192)/8)*16; //MAX BBFRAME LENGTH*4
        InitTxChannel(BufferLentx, 2);
       
        fprintf(stderr, "ENd init\n");
    }
    /*
    if (m_formattx == 1) // CS8
    {
        BufferLentx = LatencyMicro * (m_SR / 2 / 1e6); // 12 because FFT transform
        if (BufferLentx > UDP_BUFF_MAX_SIZE)
            BufferLentx -= BufferLentx % UDP_BUFF_MAX_SIZE;
        InitTxChannel(BufferLentx, 4);    
    }
    if (m_formattx == 2) // FFT
    {
        
       BufferLentx = 600  * (m_SR / 1e6); // 12 because FFT transform
       BufferLentx -= BufferLentx % fftsize;        // FixMe shoudl be relataed to mode (fft or iq)
      InitTxChannel(BufferLentx, 4);
    }
    */
    if (burstsizetx != 0)
    {

        //BufferLenrx = burstsizerx*16;
        int ret = fcntl(fileno(fdin), F_SETPIPE_SZ, burstsizetx * 2 * sizeof(short));
        PipeSize = fcntl(fileno(fdin), F_GETPIPE_SZ);
    }
    else
    {
        int ret = fcntl(fileno(fdin), F_SETPIPE_SZ, BufferLentx * 2 * sizeof(short));
        PipeSize = fcntl(fileno(fdin), F_GETPIPE_SZ);
    }

    Underflow = 0;

    fprintf(stderr, "New pipe size %d burstsizetx=%d\n", PipeSize, burstsizetx);
    pthread_mutex_unlock(&buffer_mutextx);
}

static uint64_t _timestamp_ns(void)
{
    struct timespec tp;

    if (clock_gettime(CLOCK_REALTIME, &tp) != 0)
    {
        return (0);
    }

    return ((int64_t)tp.tv_sec * 1e9 + tp.tv_nsec);
}
ssize_t direct_rx_samples(short **RxBuffer)
{
    ssize_t nsamples_rx = 0;
    if (burstsizerx == 0) // Count only if no burstsizerx as in burtsmode we knwow that we are missing frames
    {
        //uint64_t T0 = _timestamp_ns();
        nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));

        //if ((_timestamp_ns() - T0) / 1000 > m_latency)  fprintf(stderr, "refill  %llu us\n", (_timestamp_ns() - T0) / 1000);

        if (nsamples_rx < 0)
        {
            fprintf(stderr, "Error refilling Rx buf %d\n", (int)nsamples_rx);
        }

        *RxBuffer = (short *)iio_buffer_start(m_rxbuf);

        // Code to know if underrun , but could maybe disturb at high SR ?

        uint32_t val = 0;
        //https://wiki.analog.com/resources/fpga/docs/hdl/regmap
        int ret = iio_device_reg_read(m_rx, 0x80000088, &val);
        if (val & 4)
        {
            fprintf(stderr, "!");
            fflush(stderr);
            Underflow++;
            iio_device_reg_write(m_rx, 0x80000088, val); // Clear bits
        }
        else
        {
            //fprintf(stderr,"*");  fflush(stderr);
        }
    }
    else
    {
        bool underflow = false;
        //do
        {
            nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));
            uint32_t val = 0;
            //https://wiki.analog.com/resources/fpga/docs/hdl/regmap
            int ret = iio_device_reg_read(m_rx, 0x80000088, &val);
            if (val & 4)
            {
                underflow = true;
                iio_device_reg_write(m_rx, 0x80000088, val); // Clear bits
            }
            else
            {
                underflow = false;
            }
        }
        //while (underflow == true);
        *RxBuffer = (short *)iio_buffer_start(m_rxbuf);
    }
    return nsamples_rx;
}

void *rx_buffer_thread(void *arg)
{
    short *RxBuffer;
    static ssize_t RxSize = 0;
    int64_t time_first, current_time;
    time_first = _timestamp_ns();
    pthread_mutex_init(&buffer_mutexrx, NULL);
    udp_init();
    strcpy(m_iface, "127.0.0.1");
    udp_set_ip("230.0.0.1:10000", m_iface);
    remove("/dev/rx1");
    mkfifo("/dev/rx1", 0666);
    fdout = fopen("/dev/rx1", "wb");
    // For fpga recording
    fdout = fopen("/tmp/rxiq", "wb");
    InitRxChannel(20000);

    while (true)
    {

        if (RunRx)
        {
            pthread_mutex_lock(&buffer_mutexrx);

            switch (typeouput)
            {
            case output_stdout:
            {
                int nout = 0;

                // int ret = ioctl(fileno(fdout), FIONREAD, &nout);
                uint64_t T0 = _timestamp_ns();

                //if ((_timestamp_ns() - T0) / 1000 > m_latency) fprintf(stderr, "Size %d Time %llu us\n", RxSize, (_timestamp_ns() - T0) / 1000);

                if ((m_format == 0) || (m_format == 1))
                {
                    RxSize = direct_rx_samples(&RxBuffer);
                    ioctl(fileno(fdout), FIONREAD, &nout);
                    if (nout <= 2 * RxSize * 2 * sizeof(short))
                    {
                        fwrite(RxBuffer, RxSize, 2 * sizeof(short), fdout);
                    }
                    break;
                }
                if ((m_format == 2)) //FFT
                {
                    static bool fftaligned = false;
                    static int offset = 0;

                    RxSize = direct_rx_samples(&RxBuffer);
                    ioctl(fileno(fdout), FIONREAD, &nout);
                    if (nout >= (int)(burstsizerx * 2 * sizeof(short)))
                    {
                        //fprintf(stderr,"Pipe full %d\n",nout);
                        break;
                    }

                    if (RxSize != BufferLenrx)
                    {
                        fprintf(stderr, "Read Error %d\n", RxSize);
                    }

                    if (true)
                    {
                        ioctl(fileno(fdout), FIONREAD, &nout);
                        fwrite(RxBuffer + RxSize * 2 - burstsizerx * 2, (burstsizerx), 2 * sizeof(short), fdout);
                    }
                    break;
                }

                break;
            }
            case output_udp:
            {
                RxSize = direct_rx_samples(&RxBuffer);
                udp_send((char *)RxBuffer, RxSize * 2 * sizeof(short));
                break;
            }
            }

            pthread_mutex_unlock(&buffer_mutexrx);
            current_time = _timestamp_ns();
            //if((_timestamp_ns()-time_first)>m_latency*1000*2)  fprintf(stderr,"Time(ns) = %.0f us\n",(_timestamp_ns()-time_first)/1e3-m_latency);
            time_first = current_time;
        }
        else
        {
            usleep(m_latency);
        }
        //pthread_mutex_unlock(&buffer_mutexrx);
    }

    return NULL;
}

ssize_t write_from_file(FILE *fd, int len)
{

    /*
	if(iio_buffer_get_poll_fd(m_txbuf)<0)
	{
			fprintf(stderr,"Full\n");
	}
    */
    int nout;
    /*
   do
   {
        ioctl(fileno(fd), FIONREAD, &nout);
   } 
   while(nout<2*BufferLentx*sizeof(short));   
   */
  /*
    int res=ioctl(fileno(fd), FIONREAD, &nout);
    if(res<0) return -1;

    if (nout < 2 * len * sizeof(short))
        return 0;
        */
    short *buffpluto = (short *)iio_buffer_first(m_txbuf, m_tx0_i);
    buffpluto[0]=0x00;
    buffpluto[1]=0;
    int Read = fread(&(buffpluto[2]), 2 * sizeof(short),len-1, fd);
    //int Read = fread(buffpluto, 2 * sizeof(short),len, fd);
    if(Read!=len-1)
    {
        fprintf(stderr,"Only read %d / %d\n",Read,len);
        return Read;
    }

    
    //int sent=iio_buffer_push_partial(m_txbuf,Read);
    size_t sent = iio_buffer_push(m_txbuf);
    fprintf(stderr,"*");fflush(stderr);
    
	uint32_t val=0;
	int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
	if(val&1)
	{
		fprintf(stderr, "@");fflush(stderr);
		iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits	
	}
    
    return sent;
}

ssize_t write_from_buffer(short *Buffer, int len)
{
    /*
    static int old_len=0;
    if(old_len!=len)
    {
        iio_buffer_destroy(m_txbuf);
        m_txbuf = iio_device_create_buffer(m_tx, len, false);
        old_len=len;
    }
    */
    // iio_buffer_set_data(m_tx,Buffer);

    short *buffpluto = (short *)iio_buffer_start(m_txbuf);
    memcpy(buffpluto, Buffer, 2 * sizeof(short) * len);

    size_t sent=iio_buffer_push_partial(m_txbuf,len);
   // size_t sent = iio_buffer_push(m_txbuf);
    fprintf(stderr,"*");fflush(stderr);
    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    return sent;
}

ssize_t write_byte_from_buffer_split(unsigned char *Buffer, int len)
{
   static unsigned char BufferLeft[4];
   static int left=0;

   int lensample=(len)/4;
      
    short *buffpluto = (short *)iio_buffer_start(m_txbuf);

    memcpy(buffpluto,BufferLeft,left);
    memcpy(buffpluto+left, Buffer, len);
    
    size_t sent=0;
    sent=iio_buffer_push_partial(m_txbuf,(len+left)/4);
    

    left=(left+len)%4;
    if(left>0)
    {
        fprintf(stderr,"left %d\n",left);
        memcpy(BufferLeft,Buffer+len-left,left);
    }

   // size_t sent = iio_buffer_push(m_txbuf);
    fprintf(stderr,"*");fflush(stderr);
    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    return sent;
}

ssize_t write_byte_from_buffer_burst(unsigned char *Buffer, int len)
{
  static int cur_idx=0;
      
    unsigned char *buffpluto = (unsigned char *)iio_buffer_start(m_txbuf);

    memcpy(buffpluto+cur_idx,Buffer,len);
    cur_idx+=len;        
    size_t sent=0;
    if(cur_idx%4==0)
    {
        //while(ReadRegister(0x43C10008)>=1) ;
        //fprintf(stderr,"Depth %d\n",ReadRegister(0x43C10008));
        sent=iio_buffer_push_partial(m_txbuf,cur_idx/4);
       // system("./regs.sh");
        cur_idx=0;
        //fprintf(stderr,"*");fflush(stderr);
    }    
    else
        sent=0;    
   // size_t sent = iio_buffer_push(m_txbuf);
    
    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    return sent;
}

void *tx_buffer_thread(void *arg)
{
    short *TxBuffer;
    static ssize_t TxSize = 0;

    int64_t time_first, current_time;
    time_first = _timestamp_ns();
    pthread_mutex_init(&buffer_mutextx, NULL);
    //udp_init();
    //strcpy(m_iface, "127.0.0.1");
    //udp_set_ip("230.0.0.1:10000", m_iface);
    #ifndef FPGA
    remove("/dev/tx1");
    mkfifo("/dev/tx1", 0666);
    //fdin = fopen("/dev/tx1", "rb");
    fdin = fopen("test14_2.bb", "rb");
    if (fdin == NULL)
        fprintf(stderr, "Tx Pipe error\n");
    #endif   
    InitTxChannel(20000);
    //InitTxChannel(20000);
    short *Tone = (short *)malloc(BufferLentx * 2 * sizeof(short));
    for (int i = 0; i < BufferLentx; i++)
    {
        Tone[i * 2] = 0X7FFF;
        Tone[i * 2 + 1] = 0;
    }

    short *Noise = (short *)malloc(BufferLentx * 2 * sizeof(short));
    for (int i = 0; i < BufferLentx; i++)
    {
        Noise[i * 2] = (rand() * 0xFFFF) / RAND_MAX - 0x7FFF;
        Noise[i * 2 + 1] = (rand() * 0xFFFF) / RAND_MAX - 0x7FFF;
    }

    unsigned char *pattern23 = (unsigned char *)malloc(43040/8+4);

     pattern23[0]=0x31;
     pattern23[1]=0x00;
     //pattern23[0]=0x47;
     pattern23[2]=0x00;
    pattern23[3]=0x00; 
    for (int i = 4; i < 43040/8+4; i++)
    {
        pattern23[i ] = 0;
        
    }

     unsigned char *pattern238psk = (unsigned char *)malloc(43040/8+4);

     pattern238psk[0]=0x3c;
     pattern238psk[1]=0x00;
     //pattern23[0]=0x47;
     pattern238psk[2]=0x00;
    pattern238psk[3]=0x00; 
    for (int i = 4; i < 43040/8+4; i++)
    {
        pattern238psk[i ] = 0;
        
    }

    unsigned char *pattern910 = (unsigned char *)malloc(58192/8+4);
     pattern910[0]=0x36;
     pattern910[1]=0x00;
     pattern910[2]=0x00;
     pattern910[3]=0x00;
     
    for (int i = 4; i < 58192/8+4; i++)
    {
        pattern910[i ] = 0;
        
    }

     unsigned char *pattern56 = ( unsigned char *)malloc(53840/8+4);

     pattern56[0]=0x34;
     pattern56[1]=0x00;
     pattern56[2]=0x00;
     pattern56[3]=0x00;
     
    for (int i = 4; i < 53840/8+4; i++)
    {
        pattern56[i ] = 0;
        
    }

    unsigned char *pattern568 = ( unsigned char *)malloc(53840/8+4);

     pattern568[0]=0x3F;
     pattern568[1]=0x00;
     pattern568[2]=0x00;
     pattern568[3]=0x00;
     
    for (int i = 4; i < 53840/8+4; i++)
    {
        pattern568[i ] = 0;
        
    }

    unsigned char *pattern14 = (unsigned char  *)malloc(21408/8+4);

    pattern14[0]=0x2D;
     pattern14[1]=0x00;
     pattern14[2]=0x00;
     pattern14[3]=0x00;
     
    for (int i = 4; i < 21408/8+4; i++)
    {
        pattern14[i ] = 0;
        
    }
    ResetDVBS2();
    while (true)
    {

        if (RunTx)
        {
            pthread_mutex_lock(&buffer_mutextx);

            switch (typeouput)
            {
                case output_stdout:
                {
                    #ifndef FPGA    
                    ssize_t written = write_from_file(fdin, BufferLentx);
                    #else
                    static int count=0;
                    size_t written=0;
                     // written = write_byte_from_buffer_burst(pattern23,43040/8+4);
                     /* 
                     for(int i=0;i<10;i++)
                     {
                     written = write_byte_from_buffer_burst(pattern14,21408/8+4);
                     while(written==0)
                        {
                           written = write_byte_from_buffer_burst(pattern14+4,21408/8);
                        }
                     } */  

                       
                       
                        //written = write_byte_from_buffer_burst(pattern23,43040/8+4);
                        
                       //if(count==0)
                       {
                        //written = write_byte_from_buffer_burst(pattern238psk,43040/8+4);
                        
                         written = write_byte_from_buffer_burst(pattern23,43040/8+4);
                         written = write_byte_from_buffer_burst(pattern238psk,43040/8+4);
                       }
                        
                        /*written = write_byte_from_buffer_burst(pattern910,58192/8+4);
                        while(written==0)
                        {
                            written = write_byte_from_buffer_burst(pattern910+4,58192/8);
                        }*/
                        /*
                        written = write_byte_from_buffer_burst(pattern568,53840/8+4);
                        while(written==0)
                        {
                            written = write_byte_from_buffer_burst(pattern568+4,53840/8);
                        }
                        */

                        /*
                        written = write_byte_from_buffer_burst(pattern14,21408/8+4);
                        while(written==0)
                        {
                            written = write_byte_from_buffer_burst(pattern14+4,21408/8);
                        }
                       */
                      
                                 //written = write_from_buffer((short *)pattern23,1+43040/4);
                     //  written = write_from_buffer((short *)pattern14,2+3072/2);
                     // written = write_from_buffer((short *)pattern56,1+53840/4);
                      /*
                    if(count%20<12)
                             written = write_from_buffer((short *)pattern23,1+43040/8/4);
                    else
                          written = write_from_buffer((short *)pattern56,1+53840/8/4);
                    */
                    count++;    
                    fprintf(stderr,"Written %d\n",written);
                    #endif
                    
    /*
                    if(written==0)
                    {
                       
                        fclose(fdin);
                         fdin = fopen("/dev/tx1", "rb");
                        //freopen("/dev/tx1", "rb",fdin);
                    }
                    if(written<0)
                    {
                        fprintf(stderr,"CRITICAL ERROR ON READ FIFO");fflush(stdout);
                        
                    }
                   */ 
                    break;
                }
                case output_udp:
                {
                    break;
                }
            }

            pthread_mutex_unlock(&buffer_mutextx);
        }
        else
        {
            usleep(m_latency);
        }
    }

    return NULL;
}

bool publish(char *mqttkey, float value)
{
    char svalue[255];
    sprintf(svalue, "%.0f", value);
    char pubkey[255];
    sprintf(pubkey, "%s%s", sDtRoot, mqttkey);
    //fprintf(stderr,"pub %s%s\n",sDtRoot,mqttkey);
    mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
    return true;
}

bool publish(char *mqttkey, char *svalue)
{
    char pubkey[255];
    sprintf(pubkey, "%s%s", sDtRoot, mqttkey);
    //fprintf(stderr,"pub %s%s\n",sDtRoot,mqttkey);
    mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
    return true;
}

bool publishstatus(char *iio_key, char *mqttkey)
{
    FILE *fdread = NULL;
    fdread = fopen(iio_key, "r");
    char svalue[255];
    //fgets(svalue,255,fdread);
    fscanf(fdread, "%s", svalue); // To avoid getting units
    fclose(fdread);
    fprintf(stderr, "%s %s\n", iio_key, svalue);
    char pubkey[255];
    sprintf(pubkey, "%s%s", sDtRoot, mqttkey);

    mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
    return true;
}

bool SendCommand(char *skey, char *svalue)
{
    if (strcmp(svalue, "?") == 0)
        return true; // This is a request status

    FILE *fdwrite = NULL;
    fdwrite = fopen(skey, "w");
    if (fdwrite == NULL)
        return false;
    if (svalue[strlen(svalue) - 1] == 'M')
    {
        svalue[strlen(svalue) - 1] = 0;
        float value = atof(svalue) * 1e6;
        sprintf(svalue, "%.0f", value);
    }
    if (svalue[strlen(svalue) - 1] == 'K')
    {
        svalue[strlen(svalue) - 1] = 0;
        float value = atof(svalue) * 1e3;
        sprintf(svalue, "%.0f", value);
    }

    fprintf(fdwrite, "%s", svalue);
    fclose(fdwrite);
    return true;
}

char strcmd[][255] = {"listcmd", "rx/stream/run", "rx/stream/udp_addr_port", "rx/stream/output_type", "rx/stream/burst",
                      "rx/stream/average", "tx/stream/run" /*,"rx/stream/iqtype","rx/stream/udpaddress","rx/stream/udpport"*/, ""};
enum defidx
{
    listcmd,
    cmd_rxstreamrun,
    cmd_rxstreamudpadd,
    cmd_rxstreamoutputtype,
    cmd_rxstreamburst,
    cmd_rxstreamaverage,
    cmd_txstreamrun
    
};

bool publishcmd()
{
    char svalue[2500];
    sprintf(svalue, "");
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        strcat(svalue, strcmd[i]);
        strcat(svalue, ",");
    }
    publish("listcmd_stream", (char *)svalue);
    //mosquitto_publish(m_mosq, NULL, "listcmd", strlen(svalue), svalue, 2, false);
    return true;
}

void PubTelemetry()
{
    char svalue[2500];
    sprintf(svalue, "");
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        HandleCommand(strcmd[i], "?");
    }
    publish("rx/stream/underflow", (float)Underflow);
}

bool HandleCommand(char *key, char *svalue)
{

    int cmdidx = -1;
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        if (strcmp(strcmd[i], key) == 0)
        {

            cmdidx = i;
            break;
        }
    }
    if (cmdidx == -1)
        return false;

    switch (cmdidx)
    {
    case listcmd:
    {
        publishcmd();
        break;
    }
    case cmd_rxstreamrun:
    {
        if (strcmp(svalue, "?") == 0)
        {
            if (RunRx)
                publish("rx/stream/run", "1");
            else
                publish("rx/stream/run", "0");
            break;
        }
        if (strcmp(svalue, "0") == 0)
        {
            RunRx = false;
        }
        else
        {
            RunRx = true;
        }
        if (RunRx)
            publish("rx/stream/run", "1");
        else
            publish("rx/stream/run", "0");
        break;
    }
    case cmd_rxstreamudpadd:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/udp_addr_port", m_addport);
            break;
        }
        udp_set_ip(svalue, m_iface);
        publish("rx/stream/udp_addr_port", m_addport);
        break;
    }
    case cmd_rxstreamoutputtype:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/output_type", (float)typeouput);
            break;
        }
        typeouput = atoi(svalue);
        publish("rx/stream/output_type", (float)typeouput);
        break;
    }
    case cmd_rxstreamburst:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/burst", (float)burstsizerx);
            break;
        }
        burstsizerx = atoi(svalue);
        publish("rx/stream/burst", (float)burstsizerx);
        break;
    }
    case cmd_rxstreamaverage:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/average", (float)burstsizerx);
            break;
        }
        average = atoi(svalue);
        publish("rx/stream/average", (float)burstsizerx);
        break;
    }
    case cmd_txstreamrun:
    {
        if (strcmp(svalue, "?") == 0)
        {
            if (RunTx)
                publish("tx/stream/run", "1");
            else
                publish("tx/stream/run", "0");
            break;
        }
        if (strcmp(svalue, "0") == 0)
        {
            RunTx = false;
        }
        else
        {
            RunTx = true;
        }
        if (RunTx)
            publish("tx/stream/run", "1");
        else
            publish("tx/stream/run", "0");
        break;
    }
    
    }
    return true;
}

bool HandleStatus(char *key, char *svalue)
{
    //fprintf(stderr,"Handle status %s\n",key);
    if (strcmp(key, "rx/finalsr") == 0)
    {
        if (atol(svalue) != m_SR)
        {
            m_SR = atol(svalue);
            fprintf(stderr, "New sr %d\n", m_SR);

            if (RunRx)
            {
                RunRx = false; // Dirty trick to let some time to get mutex
                InitRxChannel(m_latency);
                RunRx = true;
            }
            else
                InitRxChannel(m_latency);
        }
    }
    if (strcmp(key, "rx/format") == 0)
    {
        if (atoi(svalue) != m_format)
        {
            m_format = atoi(svalue);
            fprintf(stderr, "New format %d\n", m_format);
            RunRx = false; // Dirty trick to let some time to get mutex
            InitRxChannel(m_latency);
            RunRx = true;
        }
    }
    /*
    if (strcmp(key, "tx/finalsr") == 0)
    {
        if (atol(svalue) != m_SRtx)
        {
            m_SRtx = atol(svalue);
            fprintf(stderr, "New sr %d\n", m_SRtx);

            if (RunTx)
            {
                RunTx = false; // Dirty trick to let some time to get mutex
                InitTxChannel(m_latencytx);
                RunTx = true;
            }
            else
                InitTxChannel(m_latencytx);
        }
    }*/
    return true;
}

void HandleCommandInit(struct mosquitto *mosq, char *sSerial)
{
    m_mosq = mosq;

    sprintf(sDtRoot, "dt/pluto/%s/", sSerial);
    fprintf(stderr, "Before thread \n");

    /*
    udp_init();
    strcpy(m_iface, "127.0.0.1");
    udp_set_ip("230.0.0.1:10000", m_iface);
    remove("/dev/rx1");
    mkfifo("/dev/rx1", 0666);
    fdout = fopen("/dev/rx1", "wb");
    InitRxChannel(20000);
    */
    // typeouput=output_udp;

    if (pthread_create(&(m_tid[0]), NULL, &rx_buffer_thread, NULL) != 0)
    {
        fprintf(stderr, "Rx thread cannot be started\n");
    }
    else
    {
        fprintf(stderr, "Rx thread Started\n");
    }

    if (pthread_create(&(m_tidtx[0]), NULL, &tx_buffer_thread, NULL) != 0)
    {
        fprintf(stderr, "Tx thread cannot be started\n");
    }
    else
    {
        fprintf(stderr, "Tx thread Started\n");
    }
}