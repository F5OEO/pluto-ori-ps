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

#include "./dvbs2neon/dvbs2neon.h"
#include "./dvbs2neon/bbframe23.h"

//#include "mygse/bbheader_sink_impl.h"

using namespace std;

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};
#define UDP_BUFF_MAX_SIZE (1472)
#define UDP_BUFF_MAX_BBFRAME (58192 / 8)
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
// Channels variable
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

size_t m_latency = 20000; // Latency at 80ms by default
size_t Underflow = 0;
size_t m_SR = 3000000;

int m_sock;
struct sockaddr_in m_client;

size_t m_latencytx = 20000; // Latency at 80ms by default
size_t Underflowtx = 0;
size_t m_SRtx = 3000000;

enum
{
    output_stdout,
    output_udp
};
size_t typeouput = output_stdout;

static DVB2FrameFormat fmt;

uchar BBFrameNeonBuff[144000] __attribute__((aligned(128)));
uchar symbolbuff[144 * 1024] __attribute__((aligned(16)));

bool m_fpga = true;

void ResetDVBS2()
{

    size_t value = ReadRegister(0x79020000 + 0x40BC);

    WriteRegister(0x79020000 + 0x40BC, (value & 0xFFF1) | 0);
    usleep(100);
    WriteRegister(0x79020000 + 0x40BC, (value & 0xFFF1) | 2);

    int status1 = dvbs2neon_control(0, CONTROL_RESET_FULL, (uint32)symbolbuff, sizeof(symbolbuff));
    // int status1 = dvbs2neon_control (STREAM0,CONTROL_GET_LAST_BBFRAME, (uint32)BBFrameNeonBuff, 0) ;
    int status2 = dvbs2neon_control(STREAM0, CONTROL_RESET_STREAM, 0, 0);
    // fprintf(stderr,"dvbs2neon status %d \n",status1);
}

#define switchsrc 0x43C00000
#define switchdest 0x43C20000
void SetFPGAMode(bool dvbs2)
{
    if (dvbs2)
    {
        WriteRegister(switchsrc + 0x40, 0x00);       // SI0->MI0
        WriteRegister(switchsrc + 0x44, 0x80000000); // MI1 unused
        WriteRegister(switchsrc + 0x00, 0x02);

        WriteRegister(switchdest + 0x40, 0x00); // SI0->MI0
        WriteRegister(switchdest + 0x00, 0x02);
    }
    else
    {
        WriteRegister(switchsrc + 0x40, 0x80000000); // MI0 unused
        WriteRegister(switchsrc + 0x44, 0x0);        // SI0-> MI1
        WriteRegister(switchsrc + 0x00, 0x02);

        WriteRegister(switchdest + 0x40, 0x01); // SI1->MI0
        WriteRegister(switchdest + 0x00, 0x02);
    }
}

// https://github.com/phase4ground/dvb_fpga/blob/master/third_party/airhdl/dvbs2_encoder_regs.md
#define DVBS2Register 0x43C10000
void SetDVBS2Constellation()
{
    int16_t imap;
    int16_t qmap;    
    size_t Reg;    
    for(size_t i=0;i<12;i++)
    {
        size_t Reg= ReadRegister(DVBS2Register+0xC+i*4);
        int16_t imap=(int16_t)(Reg>>16);
        int16_t qmap=(int16_t)(Reg&0xFFFF);    
        if(i<4)
            fprintf(stderr,"QPSKMap[%d]=%x %d %d\n",i,Reg,imap,qmap);
        if((i>=4)&&(i<12))
            fprintf(stderr,"8PSKMap[%d]=%x %d %d\n",i,Reg,imap,qmap);
        
    }
    imap=0x5AB1;    
    qmap=0x5AB1;
    Reg=(((size_t)imap)<<16) | ((size_t)qmap);   
    WriteRegister(DVBS2Register+0xC,Reg);
    /*
    imap=-0x7FFF;    
    qmap=-0x7FFF;
    Reg=(((size_t)imap)<<16) | ((size_t)qmap);   
    WriteRegister(0x43C1000C+4,Reg);
    */
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

inline size_t udp_receive(unsigned char *b)
{

    size_t rcvlen = recv(m_sock, b, UDP_BUFF_MAX_BBFRAME, MSG_ZEROCOPY);
    return rcvlen;
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

    // SO_ZEROCOPY
    // udp_set_ip("192.168.1.39:10000");
}

#define LOOPBACK_DISABLE 0
#define LOOPBACK_TX2RX 1
#define LOOPBACK_RX2TX 2
void fmc_set_loopback(bool enable, int Type)
{
    // Type=LOOPBACK_RX2TX
    if (enable)
        iio_device_debug_attr_write_longlong(m_rx, "loopback", Type);
    else
    {
        iio_device_debug_attr_write_longlong(m_rx, "loopback", LOOPBACK_DISABLE);
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
    // fprintf(stderr,"* Initializing AD9361 IIO streaming channels\n");
    get_ad9361_stream_ch(RX, m_rx, 0, &m_rx0_i);
    get_ad9361_stream_ch(RX, m_rx, 1, &m_rx0_q);

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

    fmc_set_loopback(true, LOOPBACK_TX2RX);

    m_rxbuf = iio_device_create_buffer(m_rx, len, false);

    if (m_rxbuf == NULL)
    {
        fprintf(stderr, "Could not allocate iio mem\n");
        // exit(1);
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
    // fprintf(stderr,"* Initializing AD9361 IIO streaming channels\n");
    get_ad9361_stream_ch(TX, m_tx, 0, &m_tx0_i);
    get_ad9361_stream_ch(TX, m_tx, 1, &m_tx0_q);

    fprintf(stderr, "Tx Stream with %u buffers of %d samples\n", nbBuffer, len);
    // Change the size of the buffer
    // m_max_len = len;

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
        // exit(1);
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
        if (BufferLenrx < fftsize)
        {
            BufferLenrx = fftsize;
        }
        InitRxChannel(BufferLenrx, 4);
    }

    if (burstsizerx != 0)
    {

        // BufferLenrx = burstsizerx*16;
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

    // m_latency = LatencyMicro;
    pthread_mutex_lock(&buffer_mutextx);

    if (m_formattx == 0) // CS16
    {
        BufferLentx = LatencyMicro * (m_SRtx / 1e6); // 12 because FFT transform
        // if (BufferLentx > UDP_BUFF_MAX_SIZE)
        //     BufferLentx -= BufferLentx % UDP_BUFF_MAX_SIZE;
        // BufferLentx=3072+1; //3072+1 for fec 1/4_short IS WORKING,3072/2+1 OK, BUT NOT 3072/4+1
        // BufferLentx=3072/4+1; //3072 for fec 1/4_short

        // BufferLentx = (1 + (58192) / 8) * 16; // MAX BBFRAME LENGTH*4
        //BufferLentx = ((58192 / 4) + 1) * 16; // MAX BBFRAME LENGTH aligned 8
        BufferLentx = ((58192 / 8) + 8) * 2; // MAX BBFRAME LENGTH aligned 8
        // BufferLentx = ((5380/4 + 1) ) ; // MAX BBFRAME LENGTH aligned 8
        InitTxChannel(BufferLentx, 8);

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

        // BufferLenrx = burstsizerx*16;
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
        // uint64_t T0 = _timestamp_ns();
        nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));

        // if ((_timestamp_ns() - T0) / 1000 > m_latency)  fprintf(stderr, "refill  %llu us\n", (_timestamp_ns() - T0) / 1000);

        if (nsamples_rx < 0)
        {
            fprintf(stderr, "Error refilling Rx buf %d\n", (int)nsamples_rx);
        }

        *RxBuffer = (short *)iio_buffer_start(m_rxbuf);

        // Code to know if underrun , but could maybe disturb at high SR ?

        uint32_t val = 0;
        // https://wiki.analog.com/resources/fpga/docs/hdl/regmap
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
            // fprintf(stderr,"*");  fflush(stderr);
        }
    }
    else
    {
        bool underflow = false;
        // do
        {
            nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));
            uint32_t val = 0;
            // https://wiki.analog.com/resources/fpga/docs/hdl/regmap
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
        // while (underflow == true);
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
    // fdout = fopen("/tmp/rxiq", "wb");
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

                // if ((_timestamp_ns() - T0) / 1000 > m_latency) fprintf(stderr, "Size %d Time %llu us\n", RxSize, (_timestamp_ns() - T0) / 1000);

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
                if ((m_format == 2)) // FFT
                {
                    static bool fftaligned = false;
                    static int offset = 0;

                    RxSize = direct_rx_samples(&RxBuffer);
                    ioctl(fileno(fdout), FIONREAD, &nout);
                    if (nout >= (int)(burstsizerx * 2 * sizeof(short)))
                    {
                        // fprintf(stderr,"Pipe full %d\n",nout);
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
            // if((_timestamp_ns()-time_first)>m_latency*1000*2)  fprintf(stderr,"Time(ns) = %.0f us\n",(_timestamp_ns()-time_first)/1e3-m_latency);
            time_first = current_time;
        }
        else
        {
            usleep(m_latency);
        }
        // pthread_mutex_unlock(&buffer_mutexrx);
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
    buffpluto[0] = 0x00;
    buffpluto[1] = 0;
    int Read = fread(&(buffpluto[2]), 2 * sizeof(short), len - 1, fd);
    // int Read = fread(buffpluto, 2 * sizeof(short),len, fd);
    if (Read != len - 1)
    {
        fprintf(stderr, "Only read %d / %d\n", Read, len);
        return Read;
    }

    // int sent=iio_buffer_push_partial(m_txbuf,Read);
    size_t sent = iio_buffer_push(m_txbuf);
    fprintf(stderr, "*");
    fflush(stderr);

    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");
        fflush(stderr);
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

    size_t sent = iio_buffer_push_partial(m_txbuf, len);
    // size_t sent = iio_buffer_push(m_txbuf);
    // fprintf(stderr, "*");
    // fflush(stderr);
    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");
        fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    return sent;
}

// https://github.com/phase4ground/dvb_fpga/blob/master/rtl/inline_config_adapter.vhd
enum
{
    shortframe,
    longframe = 0x2c
};
enum
{
    mod_qpsk,
    mod_8psk = 0xB,
    mod_16apsk = 0x16,
    mod_32apsk = 0x21
};
enum
{
    C1_4,
    C1_3,
    C2_5,
    C1_2,
    C3_5,
    C2_3,
    C3_4,
    C4_5,
    C5_6,
    C8_9,
    C9_10
};
unsigned int BBFrameLenLut[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0,
                                16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};
unsigned char m_ModCode = 0;
unsigned int m_BBFrameLenBit = 0;
unsigned char m_CodeRate = 0xFF; // OxFF means not initialized
unsigned char m_CodeConstel = 0; // QPSK
unsigned char m_CodeFrame = longframe;
ssize_t write_byte_from_buffer_burst(unsigned char *Buffer, int len, bool reset);
ssize_t write_byte_from_buffer_split(unsigned char *Buffer, int len, bool reset);

#define CRC_POLY 0xAB
// Reversed
#define CRC_POLYR 0xD5
uint8_t m_crc_tab[256];
void build_crc8_table(void)
{
    int r, crc;

    for (int i = 0; i < 256; i++)
    {
        r = i;
        crc = 0;
        for (int j = 7; j >= 0; j--)
        {
            if ((r & (1 << j) ? 1 : 0) ^ ((crc & 0x80) ? 1 : 0))
                crc = (crc << 1) ^ CRC_POLYR;
            else
                crc <<= 1;
        }
        m_crc_tab[i] = crc;
    }
}

uint8_t calc_crc8(uint8_t *b, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++)
    {
        crc = m_crc_tab[b[i] ^ crc];
    }
    return crc;
}

void SetModCode(uint FrameType, uint Constellation, uint CodeRate)
{
    static char OldModCode = 0xFF;
    if (CodeRate == 0xFF)
        return;
    if (BBFrameLenLut[(FrameType == 0 ? 0 : 11) + CodeRate] % 32 != 0)
    {
        // fprintf(stderr, "Info : Modcod is not 32 bits aligned !!!! \n");
    }

    unsigned char NewModCode = FrameType + Constellation + CodeRate;

    if (NewModCode != OldModCode)
    {

        //write_byte_from_buffer_burst(NULL, 0, true);
        // write_byte_from_buffer_split(NULL, 0, true); // Done prior because m_code is still used
        if (BBFrameLenLut[(FrameType == 0 ? 0 : 11) + CodeRate] == 0)
        {
            fprintf(stderr, "Coderate illegal\n");
            return;
        }
        m_BBFrameLenBit = BBFrameLenLut[(FrameType == 0 ? 0 : 11) + CodeRate];
        /*
        if ((m_BBFrameLenBit / 8 + 4) % 8 != 0)
        {
            fprintf(stderr, "CodeRate %d -> Wokaround , Lost BBFrame Sync %d\n",CodeRate, (m_BBFrameLenBit / 8 + 4) % 8);
            ResetDVBS2();
        }
        */
        m_ModCode = NewModCode;
        OldModCode = m_ModCode;
        fprintf(stderr, "Modcode = %x Coderate %d Len (bit) = %d Len (Byte) = %d \n", m_ModCode, CodeRate, m_BBFrameLenBit, m_BBFrameLenBit / 8);

        {
            int status = dvbs2neon_control(STREAM0, CONTROL_SET_OUTPUT_BUFFER, (uint32)BBFrameNeonBuff, 0); // CONTROL_SET_OUTPUT_BUFFER
            // fprintf(stderr, "Status %d \n", status);
        }

        switch (Constellation)
        {
        case mod_qpsk:
            fmt.constellation = M_QPSK;
            break;
        case mod_8psk:
            fmt.constellation = M_8PSK;
            break;
        case mod_16apsk:
            fmt.constellation = M_16APSK;
            break;
        case mod_32apsk:
            fmt.constellation = M_32APSK;
            break;
        default:
            fmt.constellation = M_QPSK;
        }
        fmt.fec = CodeRate;
        fmt.frame_type = (FrameType == shortframe) ? FRAME_SHORT : FRAME_NORMAL;
        fmt.output_format = OUTPUT_FORMAT_SYMBOLS; // OUTPUT_FORMAT_BBFRAME is segfault
        fmt.pilots = PILOTS_OFF;
        fmt.roll_off = RO_0_35;

        // fprintf(stderr, "Trying to set dvbs2neon \n");
        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
        modulator_mapping(fmt.constellation, CodeRate);
        // fprintf(stderr, "Status dvbs2neon_control %d \n", status);
    }
}

/*
it should be OK if the frame is not multiple of the AXI stream data width, the width converters should handle that, but for some reason that doesn't seem to be happening
as for sending less than BB frame, every chunk of data needs the AXI metadata at the start, even if it's a continuation of a prev frame.
For example, if the BB frame is 256 bytes, an entire frame would have 260 bytes (256 bytes of data + 4 bytes of metadata at the start)
If you send 200 bytes for example, the first 4 bytes will be used as metadata and the remaining 196 bytes will be sent to the encoding process. If you send another 200 bytes, the same thing happens: the first 4 bytes will be taken off and assigned to the metadata and the remaining 196 bytes will be sent to the encoding process.
So, at the end of sending 2 * 200 bytes, the encoder will have received 196*2 = 392 bytes, which is 1 BB frame of 256 bytes and 136 extra bytes that will be encoded in the next frame. The 256 bytes will use the encoding set in by the  first 4 bytes of the first 200 byte frame wile the following 136 bytes will be encoded using the first 4 bytes of the 2nd 200 byte frame





19 h 45
All in all, I think having the metadata at the start of the data stream is good if we have multiple streams, each one using potentially a different config (for example stream A has good SNR so we can use 9/10 and stream B has bad SNR so we prefer using 1/4)
If we have a single stream, we can move the metadata to be set via AXI lite and send data in whatever size we want. The new config will be used in the next frame

Coderate Sentbytes inputbeats
0   8008    8004
1   2680    2676
2   3220    3212 !!! /4 -> impair , non diviosble par 8
3   8056    8052
4   4840    4836
5   5384    5380
6   12016   12012
7   6460    6452 !!! /4 impair
8   13464   13460
9   7188    7180 !!! /4 impair
10  14552   14548



 ---> Go to split method with %8 and add metadata at each chunk
Registers :
 https://github.com/phase4ground/dvb_fpga/blob/master/third_party/airhdl/dvbs2_encoder_regs.md
*/

ssize_t write_byte_from_buffer_burst(unsigned char *Buffer, int len, bool reset)
{
    static int cur_idx = 4;
    if (reset)
    {
        cur_idx = 4;

        return 0;
    }
    // unsigned char *buffpluto = (unsigned char *)iio_buffer_start(m_txbuf);
    unsigned char *buffpluto = (unsigned char *)iio_buffer_first(m_txbuf, m_tx0_i);
    buffpluto[0] = m_ModCode;
    buffpluto[1] = 0;
    buffpluto[2] = 0;
    buffpluto[3] = 0;

    memcpy(buffpluto + cur_idx, Buffer, len);

    cur_idx += len;
    size_t sent = 0;
    if (cur_idx % 4 == 0) // %4 should work but seems %8 should fix
    {
// while(ReadRegister(0x43C10008)>=1) ;
// fprintf(stderr,"Depth %d\n",ReadRegister(0x43C10008));
#ifdef DEBUGFRAME
        for (int i = 0; i < len; i++)
        {

            if (i % 16 == 0)
                fprintf(stderr, "\n%.4x ", i);
            fprintf(stderr, "%.2x ", Buffer[i]);
        }
        fprintf(stderr, "\n");
#endif
        sent = iio_buffer_push_partial(m_txbuf, cur_idx / 4);
        // system("./regs.sh");
        cur_idx = 4;

        uint32_t val = 0;
        int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
        if (val & 1)
        {
            fprintf(stderr, "@");
            fflush(stderr);
            iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
        }
        // fprintf(stderr,"*");fflush(stderr);
    }
    else
    {
        /*buffpluto[cur_idx] = m_ModCode;
        buffpluto[cur_idx+1] = 0;
        buffpluto[cur_idx+2] = 0;
        buffpluto[cur_idx+3] = 0;
        cur_idx += 4; // AXI metadata*/
        // fprintf(stderr,"curidx %d mod %d\n",cur_idx,cur_idx%4);fflush(stderr);
        sent = 0;
    }

    return sent;
}

ssize_t write_byte_from_buffer_burstpatch(unsigned char *Buffer, int len, bool reset)
{
    if(reset==1) return 0;  
    
    unsigned int LazyLut[]={7,6,5,4,3,2,1,0};
    unsigned int IdxStart=LazyLut[(len+4)%8];
    
    unsigned char *buffpluto = (unsigned char *)iio_buffer_start(m_txbuf);
    //unsigned char *buffpluto = (unsigned char *)iio_buffer_first(m_txbuf, m_tx0_i);
    //unsigned int IdxStart=(len%4+3)%4;
    
    //fprintf(stderr,"BBLen %d IdexStart %d len+4+IdxStart+1 %d (len+IdxStart+1+4)mod8 %d\n",len,IdxStart,(len+IdxStart+1+4),(len+4+IdxStart+1)%8);
    ssize_t sent=0;
    // Normal behavior
    
    
    memset(buffpluto,0,4); //Idx = len -1
    buffpluto[0] = m_ModCode;
    memcpy(buffpluto + 4, Buffer, len);
    

        if((len+4+IdxStart+1)%8!=0)
            fprintf(stderr,"len %d is not mod 8\n",len+4+IdxStart+1);

        sent = iio_buffer_push_partial(m_txbuf, (len+4+IdxStart+1)/4);

       
        uint32_t val = 0;
        int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
        if (val & 1)
        {
            fprintf(stderr, "@");
            fflush(stderr);
            iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
        }
   
    
    return sent;
}

ssize_t write_byte_from_buffer_split(unsigned char *Buffer, int len, bool reset)
{
    static unsigned char BufferLeft[8];
    static int left = 0;

    int ToSent = 0;
    short *buffpluto = (short *)iio_buffer_start(m_txbuf);

    unsigned char *buffplutometa = (unsigned char *)iio_buffer_start(m_txbuf);

    buffplutometa[0] = m_ModCode;
    buffplutometa[1] = 0;
    buffplutometa[2] = 0;
    buffplutometa[3] = 0;
    memcpy(buffplutometa + 4, BufferLeft, left);
    size_t sent = 0;

    if (reset)
    {

        fprintf(stderr, "Loose %d in BBFRAME \n", left);
        sent = 0;

        if (left != 0)
        {
            int padding = 8 - (4 + left) % 8;
            memset(buffplutometa + 4 + left, 0, padding); // -> Padding should be to complete the BBFrame ! (not only 16 aligned)
            fprintf(stderr, "padding %d  \n", padding);
            sent = iio_buffer_push_partial(m_txbuf, (4 + left + padding) / 4);
            fprintf(stderr, "Sent with padding %d\n", sent);
        }

        left = 0; // We loose left bytes -> don't know how the encoder will handle it   : IT IS LOST !!

        return sent;
    }

    ToSent = ((4 + left + len) / 8) * 8;
    memcpy(buffplutometa + left + 4, Buffer, ToSent); // Fixme we transfer too much buffer (but not sent)

    fprintf(stderr, "Try sending %d bytes\n", ToSent);
    sent = iio_buffer_push_partial(m_txbuf, ToSent / 4);
    // system("./regs.sh");

    left = (4 + left + len) % 8;
    if (left > 0)
    {
        fprintf(stderr, "left %d len %d tocpy %d\n", left, len, len - 1 - left);
        memcpy(BufferLeft, Buffer + len - 1 - left, left);
    }

    // size_t sent = iio_buffer_push(m_txbuf);
    // fprintf(stderr, "*");
    fflush(stderr);
    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");
        fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    return sent;
}

ssize_t WriteTestBBFrame()
{
    static unsigned char *TestBBFrame = (unsigned char *)malloc(58192 * 8 / 8);
    static unsigned char GSEHeader[] = {0x40, 0x01, 0x00, 0x00, 0x96, 0xd0, 0xee, 0xcc, 0xcc, 0xe7}; // FEC 3/5
    int headerlen = sizeof(GSEHeader);
    memcpy(TestBBFrame, GSEHeader, headerlen);
    for (size_t i = headerlen; i < m_BBFrameLenBit / 8; i++)
    {
        TestBBFrame[i] = i % 256;
    }

    TestBBFrame[0] = (TestBBFrame[0] & 0xCF) | (0x3 << 4); // Single /ACM -> MultiSTream have to be studied : not well received ACM also
    TestBBFrame[1] = 0;                                    // STream 1
    TestBBFrame[9] = calc_crc8(TestBBFrame, 9);
    
    return write_byte_from_buffer_burstpatch(TestBBFrame, m_BBFrameLenBit / 8, false);
    // return write_byte_from_buffer_split(TestBBFrame,m_BBFrameLenBit/8,false);
}

ssize_t WriteReadBBFrame()
{
    static unsigned char *TestBBFrame = (unsigned char *)malloc(58192 / 8 * 4);
    int Read = fread(TestBBFrame, 1, m_BBFrameLenBit / 8, fdin);
    if (Read < m_BBFrameLenBit / 8)
    {
        fseek(fdin, 0, SEEK_SET);
        fread(TestBBFrame, 1, m_BBFrameLenBit / 8, fdin);
        fprintf(stderr, "Wrap\n");
    }
    return write_byte_from_buffer_burstpatch(TestBBFrame, m_BBFrameLenBit / 8, false);
    // return write_byte_from_buffer_split(TestBBFrame,m_BBFrameLenBit/8,false);
}

ssize_t WriteTestTS(bool fpga)
{

    unsigned char NullPacket[188] = {0x47, 0x1F, 0xFE, 0x10, 'F', '5', 'O', 'E', 'O'};
    unsigned char *bbframeptr = NULL;
    size_t packet = 0;
    static unsigned char cc = 0;
    while (bbframeptr == NULL)
    {
        NullPacket[3] = 0x10 + cc;
        cc = (cc + 1) % 16;
        bbframeptr = (unsigned char *)dvbs2neon_packet(0, (uint32)(NullPacket), 0);
        packet++;
    }
     //fprintf(stderr, " Get symbols after %d packets \n",packet);
    if (fpga)
    {
        bbframeptr = (unsigned char *)dvbs2neon_control(STREAM0, CONTROL_GET_LAST_BBFRAME, (uint32)BBFrameNeonBuff, 0);

        unsigned short *p16 = (unsigned short *)bbframeptr;
        unsigned short ByteCount = p16[-1];
        uchar *p8 = (uchar *)bbframeptr;
        uchar output_format = p8[-3];
        uchar fec = p8[-5];
        uchar frame_type = p8[-7];

        // fprintf(stderr, " Get BBframe of len %d/%d outputformat %d fec %d frametype %d \n",  ByteCount, m_BBFrameLenBit / 8, output_format, fec, frame_type);
        if (ByteCount != m_BBFrameLenBit / 8)
            fprintf(stderr, "BBFrame len mismatch fpga %d dvbneon %d\n", ByteCount, m_BBFrameLenBit / 8);
            /*
        bbframeptr[0] = (bbframeptr[0] & 0xCF) | (0x3 << 4); // Single /CCM
        bbframeptr[1] = 0;                                   // STream 0
        bbframeptr[9] = calc_crc8(bbframeptr, 9);
        */
        // fprintf(stderr,"CRC %x/%x \n",calc_crc8(bbframeptr,9),bbframeptr[9]);
        return write_byte_from_buffer_burstpatch(bbframeptr, ByteCount, false);
    }
    else
    {
        static sfcmplx *Frame = (sfcmplx *)malloc(144000 * sizeof(sfcmplx));
        unsigned short *p16 = (unsigned short *)bbframeptr;
        unsigned short ByteCount = p16[-1];
        for (int n = 0; n < ByteCount; n++)
            Frame[n] = symbol_lut[bbframeptr[n]];
        return write_from_buffer((short *)Frame, ByteCount);
    }

    // return write_byte_from_buffer_burst(bbf23,m_BBFrameLenBit/8, false);
    //  return write_byte_from_buffer_split(TestBBFrame,m_BBFrameLenBit/8,false);
}

void *tx_buffer_thread(void *arg)
{
    short *TxBuffer;
    static ssize_t TxSize = 0;

    int64_t time_first, current_time;
    time_first = _timestamp_ns();
    pthread_mutex_init(&buffer_mutextx, NULL);
// udp_init();
// strcpy(m_iface, "127.0.0.1");
// udp_set_ip("230.0.0.1:10000", m_iface);
#ifndef FPGA
    remove("/dev/tx1");
    mkfifo("/dev/tx1", 0666);
    // fdin = fopen("/dev/tx1", "rb");
    // fdin = fopen("test14_2.bb", "rb");
    if (fdin == NULL)
        fprintf(stderr, "Tx Pipe error\n");
#endif
    fdin = fopen("a-94-bbframes-of-4836-bytes-sr333-qp35-ln35", "rb");
    InitTxChannel(20000);
    // InitTxChannel(20000);
    short *Tone = (short *)malloc(BufferLentx * 2 * sizeof(short));
    for (size_t i = 0; i < BufferLentx; i++)
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

    int testcoderate = 0;

    SetFPGAMode(m_fpga);
    ResetDVBS2();
   // SetDVBS2Constellation();
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
                static int count = 0;
                size_t written = 0;

                if (m_CodeRate != 0xFF)
                {
                    // SetModCode(longframe, mod_qpsk, m_CodeRate);
                    SetModCode(m_CodeFrame, m_CodeConstel, m_CodeRate);
                    //    written = WriteTestBBFrame();
                    //for (int i = 0; i < 8; i++)
                    //    written = WriteTestBBFrame();
                    // written = WriteTestBBFrame();
                    //  m_CodeRate=0xFF;//Once
                     //for(int i=0;i<8;i++)
                         written = WriteTestTS(m_fpga);
                    // SetModCode(m_CodeFrame, m_CodeConstel, m_CodeRate+1);
                    // written = WriteTestTS(m_fpga);
                    // written= WriteReadBBFrame();
                }

                count++;
                // if(written!=0)                              fprintf(stderr, "Written %d Depth %d\n", written,ReadRegister(0x43C10D08));
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
                static unsigned char udpbbframe[UDP_BUFF_MAX_BBFRAME];
                size_t udplen = udp_receive(udpbbframe);
                SetModCode(m_CodeFrame, m_CodeConstel, m_CodeRate);
                if (udplen == m_BBFrameLenBit / 8)
                        write_byte_from_buffer_burst(udpbbframe, udplen, false);
                else
                     fprintf(stderr, "udp/bbframelen %d/%d mismatch\n", udplen, m_BBFrameLenBit / 8);
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
        // fprintf(stderr,"pub %s%s\n",sDtRoot,mqttkey);
        mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
        return true;
    }

    bool publish(char *mqttkey, char *svalue)
    {
        char pubkey[255];
        sprintf(pubkey, "%s%s", sDtRoot, mqttkey);
        // fprintf(stderr,"pub %s%s\n",sDtRoot,mqttkey);
        mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
        return true;
    }

    bool publishstatus(char *iio_key, char *mqttkey)
    {
        FILE *fdread = NULL;
        fdread = fopen(iio_key, "r");
        char svalue[255];
        // fgets(svalue,255,fdread);
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
                          "rx/stream/average", "tx/stream/run" /*,"rx/stream/iqtype","rx/stream/udpaddress","rx/stream/udpport"*/,
                          "tx/dvbs2/coderate", "tx/dvbs2/constel", "tx/dvbs2/frame", ""};
    enum defidx
    {
        listcmd,
        cmd_rxstreamrun,
        cmd_rxstreamudpadd,
        cmd_rxstreamoutputtype,
        cmd_rxstreamburst,
        cmd_rxstreamaverage,
        cmd_txstreamrun,
        cmd_txdvbs2coderate,
        cmd_txdvbs2constellation,
        cmd_txdvbs2frame

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
        // mosquitto_publish(m_mosq, NULL, "listcmd", strlen(svalue), svalue, 2, false);
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
        case cmd_txdvbs2coderate:
        {
            if (strcmp(svalue, "?") == 0)
            {
                publish("tx/dvbs2/coderate", m_CodeRate);
                break;
            }
            m_CodeRate = atoi(svalue);
            publish("tx/dvbs2/coderate", m_CodeRate);
            break;
        }
        case cmd_txdvbs2constellation:
        {
            if (strcmp(svalue, "?") == 0)
            {
                switch (m_CodeConstel)
                {
                case mod_qpsk:
                    publish("tx/dvbs2/constel", "qpsk");
                    break;
                case mod_8psk:
                    publish("tx/dvbs2/constel", "8psk");
                    break;
                case mod_16apsk:
                    publish("tx/dvbs2/constel", "16apsk");
                    break;
                case mod_32apsk:
                    publish("tx/dvbs2/constel", "32apsk");
                    break;
                }

                break;
            }
            if (strcmp(svalue, "qpsk") == 0)
                m_CodeConstel = mod_qpsk;
            if (strcmp(svalue, "8psk") == 0)
                m_CodeConstel = mod_8psk;
            if (strcmp(svalue, "16apsk") == 0)
                m_CodeConstel = mod_16apsk;
            if (strcmp(svalue, "32apsk") == 0)
                m_CodeConstel = mod_32apsk;
            publish("tx/dvbs2/constel", svalue);
            break;
        }
        case cmd_txdvbs2frame:
        {
            if (strcmp(svalue, "?") == 0)
            {
           
                switch (m_CodeFrame)
                {
                case shortframe:
                    publish("tx/dvbs2/frame", "short");
                    break;
                case longframe:
                    publish("tx/dvbs2/frame", "long");
                    break;
                }
                break;
            }
            if (strcmp(svalue, "short") == 0)
                m_CodeFrame = shortframe;
            if (strcmp(svalue, "long") == 0)
                m_CodeFrame = longframe;
           
            publish("tx/dvbs2/frame", svalue);
            
            break;
        }
        }
        return true;
    }

    bool HandleStatus(char *key, char *svalue)
    {
        // fprintf(stderr,"Handle status %s\n",key);
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

    void HandleCommandInit(struct mosquitto * mosq, char *sSerial)
    {
        m_mosq = mosq;

        sprintf(sDtRoot, "dt/pluto/%s/", sSerial);
        fprintf(stderr, "Before thread \n");
        build_crc8_table();

        udp_init();
        strcpy(m_iface, "127.0.0.1");
        udp_set_ip("230.0.0.1:1234", m_iface);
        /*
        remove("/dev/rx1");
        mkfifo("/dev/rx1", 0666);
        fdout = fopen("/dev/rx1", "wb");
        InitRxChannel(20000);
        */
        typeouput = output_stdout;
/*
        if (pthread_create(&(m_tid[0]), NULL, &rx_buffer_thread, NULL) != 0)
        {
            fprintf(stderr, "Rx thread cannot be started\n");
        }
        else
        {
            fprintf(stderr, "Rx thread Started\n");
        }
*/
        if (pthread_create(&(m_tidtx[0]), NULL, &tx_buffer_thread, NULL) != 0)
        {
            fprintf(stderr, "Tx thread cannot be started\n");
        }
        else
        {
            fprintf(stderr, "Tx thread Started\n");
        }
    }