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
#include <net/if.h>
#include <netinet/in.h>
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

#include "tsinputmux.h"
#include "gsemux.h"
#include "iqtofft.h"
#include <queue>

//#include "mygse/bbheader_sink_impl.h"

using namespace std;

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};
#define UDP_BUFF_MAX_SIZE (1472)
//#define UDP_BUFF_MAX_BBFRAME (58192 / 8)
#define UDP_BUFF_MAX_BBFRAME 8000
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
size_t fftsize = 2048;
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
pthread_mutex_t bufpluto_mutexrx;
// ************ Tx Thread *********************
static pthread_t m_tidtx[1];
static bool RunTx = true;
pthread_mutex_t buffer_mutextx;
pthread_mutex_t bufpluto_mutextx;

static pthread_t m_tidtxfillbuff[1];

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
float m_maxgain = -100.0; // Impossible gain, means not AGC

int m_sock;
struct sockaddr_in m_client;

size_t m_latencytx = 20000; // Latency at 80ms by default
size_t Underflowtx = 0;
size_t m_SRtx = 3000000;

size_t m_s2sr = 1000000;
uint32_t m_efficiency = 1000000;
char mcast_rxiface[255]; // mcast ip to receive bbrame from longmynd

enum
{
    rx_mode_pass,
    rx_mode_stdout,
    rx_mode_udp,
    rx_mode_websocket,

};
char s_rxmode[255] = "pass";
size_t m_rxmode = rx_mode_pass;

/*
uchar BBFrameNeonBuff[144000] __attribute__((aligned(128)));
uchar symbolbuff[144 * 1024] __attribute__((aligned(16)));
*/
bool m_fpga = true;

enum
{
    tx_passtrough,
    tx_iq,
    tx_dvbs2_ts,
    tx_dvbs2_gse,
    tx_test

};
char s_txmode[255] = "pass";
int m_txmode = tx_passtrough;

int m_gainvariable=0;

// int m_txmode = tx_dvbs2_ts;
//  https://github.com/phase4ground/dvb_fpga/blob/master/rtl/inline_config_adapter.vhd
enum
{
    longframe,
    shortframe

};
enum
{
    mod_qpsk,
    mod_8psk = 1,
    mod_16apsk = 2,
    mod_32apsk = 3
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

char TabFec[][255] = {"1/4", "1/3", "2/5", "1/2", "3/5", "2/3", "3/4", "4/5", "5/6", "8/9", "9/10"};

#define MAX_QUEUE_ITEM 20
typedef struct
{
    ssize_t size;
    ssize_t modecod;
    uint8_t bbframe[UDP_BUFF_MAX_BBFRAME];
} buffer_t;

enum
{
    fec_fix,
    fec_variable
};

int m_Fecmode = fec_fix;
int m_FecRange = 11;

queue<buffer_t *> m_bbframe_queue;

float TheoricMER[] = {0, -2.4, -1.2, 0, 1.0, 2.2, 3.2, 4.0, 4.6, 5.2, 6.2, 6.5, 5.5, 6.6, 7.9, 9.4, 10.6, 11.0, 9.0, 10.2, 11.0, 11.6, 12.9, 13.1, 12.6, 13.6, 14.3, 15.7, 16.1};

void ResetDVBS2()
{

    size_t value = ReadRegister(0x79020000 + 0x40BC);

    WriteRegister(0x79020000 + 0x40BC, (value & 0xFFF1) | 0);
    usleep(100);
    WriteRegister(0x79020000 + 0x40BC, (value & 0xFFF1) | 2);

    // int status1 = dvbs2neon_control(0, CONTROL_RESET_FULL, (uint32)symbolbuff, sizeof(symbolbuff));

    // int status2 = dvbs2neon_control(STREAM0, CONTROL_RESET_STREAM, 0, 0);
    //  fprintf(stderr,"dvbs2neon status %d \n",status1);
}

#define switchsrc 0x43C00000
#define switchdest 0x43C20000
void SetFPGAMode(bool dvbs2)
{
    usleep(100000); // Maybe the time I/Q samples don't go to dvbs2
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

    /*
        fprintf(stderr,"%x %x\n",switchsrc + 0x40,ReadRegister(switchsrc + 0x40));
        fprintf(stderr,"%x %x\n",switchsrc + 0x44,ReadRegister(switchsrc + 0x44));
        fprintf(stderr,"%x %x\n",switchsrc + 0x0,ReadRegister(switchsrc + 0x0));

        fprintf(stderr,"%x %x\n",switchdest + 0x40,ReadRegister(switchdest + 0x40));
        fprintf(stderr,"%x %x\n",switchdest + 0x0,ReadRegister(switchdest + 0x0));
    */
}

// https://github.com/phase4ground/dvb_fpga/blob/master/third_party/airhdl/dvbs2_encoder_regs.md
#define DVBS2Register 0x43C10000
void SetDVBS2Constellation()
{
    int16_t imap;
    int16_t qmap;
    size_t Reg;
    // int16_t magqspk=23170;
    // int16_t magqspk=32000;
    int16_t magqspk = 23170;
    imap = magqspk;
    qmap = magqspk;
    WriteRegister(DVBS2Register + 0x110 + 0, (imap << 16) | (qmap & 0xFFFF));
    imap = magqspk;
    qmap = -magqspk;
    WriteRegister(DVBS2Register + 0x110 + 4, (imap << 16) | (qmap & 0xFFFF));
    imap = -magqspk;
    qmap = magqspk;
    WriteRegister(DVBS2Register + 0x110 + 8, (imap << 16) | (qmap & 0xFFFF));
    imap = -magqspk;
    qmap = -magqspk;
    WriteRegister(DVBS2Register + 0x110 + 12, (imap << 16) | (qmap & 0xFFFF));

    for (size_t i = 0; i < 12; i++)
    {
        size_t Reg = ReadRegister(DVBS2Register + 0x110 + i * 4);
        int16_t imap = (int16_t)(Reg >> 16);
        int16_t qmap = (int16_t)(Reg & 0xFFFF);
        if (i < 4)
            fprintf(stderr, "QPSKMap[%d]=%x %d %d\n", i, Reg, imap, qmap);
        if ((i >= 4) && (i < 12))
            fprintf(stderr, "8PSKMap[%d]=%x %d %d\n", i, Reg, imap, qmap);
    }
    /*
    imap = 0x5AB1;
    qmap = 0x5AB1;
    Reg = (((size_t)imap) << 16) | ((size_t)qmap);
    WriteRegister(DVBS2Register + 0xC, Reg);
    */
    /*
    imap=-0x7FFF;
    qmap=-0x7FFF;
    Reg=(((size_t)imap)<<16) | ((size_t)qmap);
    WriteRegister(0x43C1000C+4,Reg);
    */
}
void SetDVBS2Dummy(bool enable)
{
    size_t Reg = ReadRegister(DVBS2Register);

    Reg = Reg & 0xFFFBFFFF;
    Reg |= enable ? (1 << 18) : 0;

    WriteRegister(DVBS2Register, Reg);
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
    // #define SO_ZEROCOPY 60

    /*if (setsockopt(m_sock, SOL_SOCKET, SO_ZEROCOPY, &one, sizeof(one)))
    {
        fprintf(stderr, "UDP zerocopy mode failed\n");
    }
    */

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
    if (len == 0) // We are in passthrough, don't get the stream because it is used externally
    {
        return;
    }
    iio_device_set_kernel_buffers_count(m_rx, nbBuffer); // SHould be called BEFORE create_buffer (else not setting)

    //	printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(m_rx0_i);
    iio_channel_enable(m_rx0_q);

    //fmc_set_loopback(true, LOOPBACK_TX2RX);

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
       fprintf(stderr, "Rx Stream with %u buffers of %d samples\n", nbBuffer, len);
}
extern bool SendCommand(char *skey, char *svalue);
void InitTxChannel(size_t len, unsigned int nbBuffer)
{
    if (m_ctx == NULL)
        m_ctx = iio_create_local_context();
    if (m_ctx == NULL)
        fprintf(stderr, "Init context fail\n");
    iio_context_set_timeout(m_ctx, 0);

    get_ad9361_stream_dev(m_ctx, TX, &m_tx);
    // fprintf(stderr,"* Initializing AD9361 IIO streaming channels\n");
    get_ad9361_stream_ch(TX, m_tx, 0, &m_tx0_i);
    get_ad9361_stream_ch(TX, m_tx, 1, &m_tx0_q);

    // Change the size of the buffer
    // m_max_len = len;
    char msgerror[255];
    // pthread_mutex_lock(&bufpluto_mutextx);
    if (m_txbuf != NULL)
    {
        iio_buffer_cancel(m_txbuf);
        iio_strerror(errno, msgerror, sizeof(msgerror));
        fprintf(stderr, "Cancel buffer %s\n", msgerror);
        iio_buffer_destroy(m_txbuf);
        iio_strerror(errno, msgerror, sizeof(msgerror));
        fprintf(stderr, "Destroy buffer %s\n", msgerror);
        iio_channel_disable(m_tx0_i); // Fix the bug https://github.com/analogdevicesinc/libiio/commit/02527e69ab57aa2eac995e964b58421b0f5af5ad
        iio_channel_disable(m_tx0_q);
        iio_strerror(errno, msgerror, sizeof(msgerror));
        fprintf(stderr, "channel disable tx %s\n", msgerror);
        m_txbuf = NULL;
        // SendCommand("/sys/bus/iio/devices/iio:device2/buffer/enable", "0");
    }
    if (len == 0) // We are in passthrough, don't get the stream because it is used externally
    {
        return;
    }

    len = ((len / 8) + 1) * 8;
    int ret = iio_device_set_kernel_buffers_count(m_tx, nbBuffer); // SHould be called BEFORE create_buffer (else not setting)
    iio_strerror(errno, msgerror, sizeof(msgerror));
    fprintf(stderr, "Kernel count %s\n", msgerror);
    if (ret != 0)
        fprintf(stderr, "set_kernel_buffers_count issue\n");
    //	printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(m_tx0_i);
    iio_channel_enable(m_tx0_q);
    iio_strerror(errno, msgerror, sizeof(msgerror));
    fprintf(stderr, "enable buffer %s\n", msgerror);
    m_txbuf = iio_device_create_buffer(m_tx, len, false);

    if (m_txbuf == NULL)
    {
        iio_strerror(errno, msgerror, sizeof(msgerror));
        fprintf(stderr, "Could not allocate iio mem tx %s\n", msgerror);
        sleep(10000);
        // exit(1);
    }

    iio_buffer_set_blocking_mode(m_txbuf, true);
    fprintf(stderr, "Tx Stream with %u buffers of %d samples\n", nbBuffer, len);
    // pthread_mutex_unlock(&bufpluto_mutextx);
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

        BufferLentx = ((58192 / 8) + 8) * 2; // MAX BBFRAME LENGTH aligned 8

        InitTxChannel(BufferLentx, 2);

        fprintf(stderr, "ENd init\n");
    }

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
    pthread_mutex_lock(&bufpluto_mutexrx);
    if(m_rxbuf==NULL)
    {
        pthread_mutex_unlock(&bufpluto_mutexrx);
        return 0;
    } 
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
            // fprintf(stderr, "!");
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
    pthread_mutex_unlock(&bufpluto_mutexrx);
    return nsamples_rx;
}


void SetRxMode(int Mode)
{
    pthread_mutex_lock(&bufpluto_mutexrx);

    switch (Mode)
    {
    case rx_mode_pass:
    {
        InitRxChannel(0, 0);
        
    }
    break;
    case rx_mode_websocket:
    {
       InitRxChannel(fftsize * 30, 2);
       
       
    }
    break;
    }
    m_rxmode = Mode;
    fprintf(stderr, "Change rx mode %d\n", m_rxmode);
    
    pthread_mutex_unlock(&bufpluto_mutexrx);
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
    init_fft(fftsize, 10); 
    // fdout = fopen("/dev/rx1", "wb");

    // Fixme : CHange it in setrxmode
    // InitRxChannel(fftsize * 10, 2);
    // init_fft(fftsize, 30);

    while (true)
    {

        if (RunRx)
        {
            pthread_mutex_lock(&buffer_mutexrx);

            switch (m_rxmode)
            {
            case rx_mode_stdout:
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
            case rx_mode_udp:
            {
                RxSize = direct_rx_samples(&RxBuffer);
                udp_send((char *)RxBuffer, RxSize * 2 * sizeof(short));
            }
            break;

            case rx_mode_websocket:
            {
                RxSize = direct_rx_samples(&RxBuffer);
                if(RxSize!=0)
                    iqtofft(RxBuffer, RxSize);
            }
            break;
            case rx_mode_pass:
            {
                usleep(100000);
            }
            break;
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

    pthread_mutex_lock(&bufpluto_mutextx);

    short *buffpluto = (short *)iio_buffer_start(m_txbuf);

    int Read = fread(buffpluto, 2 * sizeof(short), len, fd);
    // Fixme : Could have some deadlock here
    if (m_txmode != tx_iq)
    {
        fprintf(stderr, "mode has changed \n");
        pthread_mutex_unlock(&bufpluto_mutextx);
        return 0;
    }
    if (Read != len)
    {
        fprintf(stderr, "Only read %d / %d\n", Read, len);
        return Read;
    }

    int sent = iio_buffer_push_partial(m_txbuf, Read);

    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    if (val & 1)
    {
        fprintf(stderr, "@");
        fflush(stderr);
        iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    }
    pthread_mutex_unlock(&bufpluto_mutextx);
    return sent;
}

ssize_t write_from_buffer(short *Buffer, int len)
{
    pthread_mutex_lock(&bufpluto_mutextx);
    if (m_txmode != tx_test)
    {
        fprintf(stderr, "mode has changed \n");
        pthread_mutex_unlock(&bufpluto_mutextx);
        return 0;
    }
    short *buffpluto = (short *)iio_buffer_start(m_txbuf);
    // fprintf(stderr, "buffpluto %x Buffer %x\n",buffpluto,Buffer);
    memcpy(buffpluto, Buffer, 2 * sizeof(short) * len);
    // fprintf(stderr, "*\n");

    size_t sent = iio_buffer_push_partial(m_txbuf, len);
    // fprintf(stderr, "!\n");
    pthread_mutex_unlock(&bufpluto_mutextx);
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

unsigned char m_ModCode = C2_3 + longframe;
unsigned int m_BBFrameLenBit = 0;
unsigned char m_CodeRate = 0xFF; // OxFF means not initialized
unsigned char m_CodeConstel = 0; // QPSK
unsigned char m_CodeFrame = longframe;
unsigned char m_Pilots = 0;
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

typedef struct
{
    uint32_t frame_type;
    uint32_t fec;
    uint32_t roll_off;
    uint32_t constellation;
    uint32_t pilots;
    uint32_t output_format; // dummy_frame in DATV-Express
} DVB2FrameFormat;

unsigned char getdvbs2modcod(uint FrameType, uint Constellation, uint CodeRate, uint Pilots)
{
    unsigned char NewModCode=0;

    if (CodeRate > 10)
        CodeRate = 10;
    if (Constellation == mod_qpsk)
    {
        NewModCode = CodeRate + 1;
    }
    if (Constellation == mod_8psk)
    {
        unsigned char Tab8PSK[] = {12, 12, 12, 12, 12, 13, 14, 15, 15, 16, 17};
        NewModCode = Tab8PSK[CodeRate];
    }
    if (Constellation == mod_16apsk)
    {
        unsigned char Tab16APSK[] = {18, 18, 18, 18, 18, 18, 19, 20, 21, 22, 23};
        NewModCode = Tab16APSK[CodeRate];
    }
    if (Constellation == mod_32apsk)
    {
        unsigned char Tab32APSK[] = {24, 24, 24, 24, 24, 24, 24, 25, 26, 27, 28};
        NewModCode = Tab32APSK[CodeRate];
    }
    NewModCode |= (Pilots & 1) << 5; // No pilots
    if (FrameType == longframe)
        NewModCode |= 0 << 6; // Longframe
    else
        NewModCode |= 1 << 6; // Longframe

    return NewModCode;
}

void SetModCode(uint FrameType, uint Constellation, uint CodeRate, uint Pilots)
{
    static char OldModCode = 0xFF;
    if (CodeRate == 0xFF)
        return;

    unsigned char NewModCode = getdvbs2modcod(FrameType, Constellation, CodeRate, Pilots);

    if (NewModCode != OldModCode)
    {
        fprintf(stderr, "Frame %d Constellation %d CodeRate %d Pilots %d\n", FrameType, Constellation, CodeRate, Pilots);
        // write_byte_from_buffer_burst(NULL, 0, true);
        //  write_byte_from_buffer_split(NULL, 0, true); // Done prior because m_code is still used

        m_BBFrameLenBit = BBFrameLenLut[(FrameType == shortframe ? 0 : 11) + CodeRate];

        // m_ModCode = NewModCode;
        // OldModCode = m_ModCode;
        OldModCode = NewModCode;
        fprintf(stderr, "Modcode = %x Coderate %d Len (bit) = %d Len (Byte) = %d \n", NewModCode, CodeRate, m_BBFrameLenBit, m_BBFrameLenBit / 8);

        {
            // int status = dvbs2neon_control(STREAM0, CONTROL_SET_OUTPUT_BUFFER, (uint32)BBFrameNeonBuff, 0); // CONTROL_SET_OUTPUT_BUFFER
            //  fprintf(stderr, "Status %d \n", status);
        }
        setneonmodcod(Constellation, CodeRate, FrameType, Pilots);
        fprintf(stderr, "Efficiency %d NetBitrate = %f !\n", m_efficiency, float(m_SRtx * (m_efficiency / 4e6)));
        // fprintf(stderr, "modocodgse  %d \n", (FrameType == 0 ? 0 : 11) + CodeRate);
        setgsemodcod(Constellation, CodeRate, FrameType, Pilots);
        // fprintf(stderr, "Status dvbs2neon_control %d \n", status);
    }
}

bool SendCommand(char *skey, char *svalue);

ssize_t write_bbframe()
{
    struct timespec start, now;

    unsigned int LazyLut[] = {7, 6, 5, 4, 3, 2, 1, 0};

    ssize_t sent = 0;
    // Normal behavior

    pthread_mutex_lock(&buffer_mutextx);
    pthread_mutex_lock(&bufpluto_mutextx);
    if (m_txbuf == NULL)
    {
        fprintf(stderr, "mode has changed \n");
        pthread_mutex_unlock(&buffer_mutextx);
        pthread_mutex_unlock(&bufpluto_mutextx);
        return 0;
    }
    unsigned char *buffpluto = (unsigned char *)iio_buffer_start(m_txbuf);
    buffer_t *newbuf = m_bbframe_queue.front();
    ssize_t len = newbuf->size;
    unsigned int IdxStart = LazyLut[(len + 2) % 8];

    memset(buffpluto, 0, 2); // Idx = len -1

    // buffpluto[0] = newbuf->modecod;
    buffpluto[0] = 0xB8;
    buffpluto[1] = newbuf->modecod;
    /*
   if(newbuf->modecod>=longframe)
   {
       buffpluto[1]=(newbuf->modecod-longframe+1);
       buffpluto[1]|=0<<5; // No pilots
       buffpluto[1]|=0<<6; // Longframe
   }
   else
   {
       buffpluto[1]=(newbuf->modecod+1);
       buffpluto[1]|=1<<5; // No pilots
       buffpluto[1]|=1<<6; // SHortframe
   }*/

    // fprintf(stderr,"Warning : fec %d short = %d\n",buffpluto[1]&0x1F,buffpluto[1]>>5);
    memcpy(buffpluto + 2, newbuf->bbframe, newbuf->size);
    free(newbuf);
    m_bbframe_queue.pop();
    pthread_mutex_unlock(&buffer_mutextx);
    if (BBFrameLenLut[buffpluto[1]] / 8 != len)
    {
        // fprintf(stderr,"Warning : modcod %x bbfram len %d len %d\n",buffpluto[0],BBFrameLenLut[buffpluto[0]-0x2c+11]/8,len);
        // return 0;
    }
    if ((len + 2 + IdxStart + 1) % 8 != 0)
        fprintf(stderr, "len %d is not mod 8\n", len + 2 + IdxStart + 1);
    clock_gettime(CLOCK_MONOTONIC, &start);

    if (m_gainvariable ==1 ) // FixMe : Should be max gain
    {

        float maxgain = 6.5;
        int modecode = buffpluto[1] & 0xF;
        if ((modecode > 0) && (modecode <= 11)) // qpsk
            maxgain = TheoricMER[11];
        if ((modecode > 11) && (modecode <= 17)) // 8psk
            maxgain = TheoricMER[17];
        if ((modecode > 17) && (modecode <= 23)) // 16apsk
            maxgain = TheoricMER[23];
        if ((modecode > 23) && (modecode <= 28)) // 32apsk
            maxgain = TheoricMER[28];

        float offsetgain = TheoricMER[buffpluto[1] & 0xF] - maxgain;
        if (offsetgain <= 0)
        {
            char svalue[255];
            sprintf(svalue, "%f", offsetgain + m_maxgain);
            //fprintf(stderr, "Offset %f Gain %s for modcod %d\n", offsetgain, svalue, buffpluto[1]);
            SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain", svalue);
        }
        else
        {
            fprintf(stderr, "Too much gain offset %f %f\n", offsetgain, offsetgain + m_maxgain);
        }
    }

    sent = iio_buffer_push_partial(m_txbuf, (len + 2 + IdxStart + 1) / 4);
    pthread_mutex_unlock(&bufpluto_mutextx);
    // AGC TX Gain

    clock_gettime(CLOCK_MONOTONIC, &now);

    size_t diff_us = (now.tv_sec - start.tv_sec) * 1000000L;
    diff_us += (now.tv_nsec - start.tv_nsec) / 1000L;
    // fprintf(stderr,"Diff %u \n",diff_us);
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

bool IsPhysicalUp(char *if_name)
{
    bool Isup = false;
    char sFilePath[255];
    sprintf(sFilePath, "/sys/class/net/%s/operstate", if_name);

    FILE *fd = fopen(sFilePath, "r");
    if (fd == NULL)
        return false;
    char sState[255];
    fscanf(fd, "%s", sState);
    if (strcmp(sState, "up") == 0)
    {
        return true;
    }
    return false;
}

bool GetInterfaceip(char *if_name, char *ip)
{

    if (!IsPhysicalUp(if_name))
    {
        fprintf(stderr, "No iface %s\n", if_name);
        return false;
    }
    struct ifreq ifr;
    size_t if_name_len = strlen(if_name);
    if (if_name_len < sizeof(ifr.ifr_name))
    {
        memcpy(ifr.ifr_name, if_name, if_name_len);
        ifr.ifr_name[if_name_len] = 0;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);

    ioctl(fd, SIOCGIFADDR, &ifr);
    close(fd);
    struct sockaddr_in *ipaddr = (struct sockaddr_in *)&ifr.ifr_addr;
    printf("IP address: %s\n", inet_ntoa(ipaddr->sin_addr));
    strcpy(ip, inet_ntoa(ipaddr->sin_addr));
    return true;
}

void SetTxMode(int Mode)
{
    pthread_mutex_lock(&bufpluto_mutextx);

    switch (Mode)
    {
    case tx_passtrough:
    {
        InitTxChannel(0, 0);
        SetFPGAMode(false);
    }
    break;
    case tx_iq:
    case tx_test:
    {
        int LatencyMicro = 20000; // 20 ms buffer
        BufferLentx = LatencyMicro * (m_SRtx / 1e6);
        InitTxChannel(BufferLentx, 2);
        SetFPGAMode(false);
    }
    break;
    case tx_dvbs2_ts:
    {
        static int debugbuffer = 2;
        BufferLentx = ((58192 / 8) + 8) * 2; // MAX BBFRAME LENGTH aligned 8
        // Should be calculated from mm_srtx
        int nbBuffer = ((m_SRtx / 2000000) / 2) * 8;

        InitTxChannel(BufferLentx, nbBuffer >= 2 ? nbBuffer : 2);
        // InitTxChannel(BufferLentx,debugbuffer);
        debugbuffer *= 2;

        SetFPGAMode(true);
        ResetDVBS2();
        SetDVBS2Constellation();
    }
    break;
    case tx_dvbs2_gse:
    {
        BufferLentx = ((58192 / 8) + 8) * 2; // MAX BBFRAME LENGTH aligned 8

        InitTxChannel(BufferLentx, 2);
        SetFPGAMode(true);
        ResetDVBS2();
        SetDVBS2Constellation();
    }
    break;
    }
    m_txmode = Mode;
    fprintf(stderr, "Change txmode %d\n", m_txmode);

    uint32_t val = 0;
    int ret = iio_device_reg_read(m_tx, 0x80000088, &val);
    while ((val & 1) == 0)
    {
        fprintf(stderr, "Wait for purging\n");
        usleep(1000);
        iio_device_reg_read(m_tx, 0x80000088, &val);
    }
    iio_device_reg_write(m_tx, 0x80000088, val); // Clear bits
    pthread_mutex_unlock(&bufpluto_mutextx);
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

    remove("/dev/tx1");
    mkfifo("/dev/tx1", 0666);
    if (fdin == NULL)
        fprintf(stderr, "Tx Pipe error\n");

    // InitTxChannel(20000);

    short *Noise = (short *)malloc(BufferLentx * 2 * sizeof(short));
    for (int i = 0; i < BufferLentx; i++)
    {
        Noise[i * 2] = (rand() * 0xFFFF) / RAND_MAX - 0x7FFF;
        Noise[i * 2 + 1] = (rand() * 0xFFFF) / RAND_MAX - 0x7FFF;
    }

    int testcoderate = 0;
    SetTxMode(tx_passtrough);
    char ip[255];
    if (!GetInterfaceip("eth0", ip)) // Choose first eth0
    {
        GetInterfaceip("usb0", ip);
        char scommand[512];
        sprintf(scommand, "ip route add default via %s", ip);
        system(scommand);
    }
    // init_tsmux("230.10.0.1:1234", ip);
    init_tsmux("230.10.0.1:1234", ip);
    init_gsemux("230.0.0.2:1234", ip, "44.0.0.2", 200000);
    // init_gsemux("230.0.0.3:1234", ip, "44.0.0.2", 20000); // Yves
    while (true)
    {

        if (RunTx)
        {

            switch (m_txmode)
            {

            case tx_dvbs2_ts:
            case tx_dvbs2_gse:
            {
                if (m_CodeRate == 0xFF)
                    break;
                /*
                    pthread_mutex_lock(&bufpluto_mutextx);
                   InitTxChannel(BufferLentx,2);
                    pthread_mutex_unlock(&bufpluto_mutextx);

                    usleep(500000);
                    continue;
                     */
                SetModCode(m_CodeFrame, m_CodeConstel, m_CodeRate, m_Pilots);
                // fprintf(stderr,"Queue %d\n",m_bbframe_queue.size());
                if (!m_bbframe_queue.empty())
                {

                    write_bbframe();
                }
                else // No more data : need padding
                {

                    if (m_txmode == tx_dvbs2_ts)
                        setpaddingts();
                    if (m_txmode == tx_dvbs2_gse)
                        setpaddinggse();
                    // setpaddingts();
                }
            };
            break;
            case tx_passtrough:
            {
                usleep(100000);
            };
            break;
            case tx_iq:
            {
                write_from_file(fdin, BufferLentx);
            };
            break;
            case tx_test:
            {
                static short *Tone = NULL;
                if (Tone == NULL)
                {
                    Tone = (short *)malloc(BufferLentx * 2 * sizeof(short));
                    fprintf(stderr, "Init Tone  %d\n", BufferLentx);
                    for (size_t i = 0; i < BufferLentx; i++)
                    {
                        Tone[i * 2] = 0x7FFF;
                        Tone[i * 2 + 1] = 0;
                    }
                }
                // fprintf(stderr,"Tone  %d\n",BufferLentx);
                write_from_buffer(Tone, BufferLentx);
            };
            break;
            }
        }
        else
        {
            usleep(m_latency);
        }
    }

    return NULL;
}

bool publish(char *mqttkey, float value, bool isstatus = true)
{
    char svalue[255];
    sprintf(svalue, "%.0f", value);
    char pubkey[512];
    if (isstatus)
        sprintf(pubkey, "%s%s", sDtRoot, mqttkey);
    else
        sprintf(pubkey, "%s%s", sCmdRoot, mqttkey);
    // fprintf(stderr,"pub %s%s\n",sDtRoot,mqttkey);
    mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
    return true;
}

bool publish(char *mqttkey, char *svalue, bool isstatus = true)
{
    char pubkey[512];
    if (isstatus)
        sprintf(pubkey, "%s%s", sDtRoot, mqttkey);
    else
        sprintf(pubkey, "%s%s", sCmdRoot, mqttkey);
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

char strcmd[][255] = {"listcmd", "rx/stream/run", "rx/stream/udp_addr_port", "rx/stream/output_type", "rx/stream/burst","rx/stream/mode",
                      "rx/stream/average", "tx/stream/run", "tx/stream/mode" ,
                      "tx/dvbs2/fec", "tx/dvbs2/constel", "tx/dvbs2/frame", "tx/dvbs2/pilots", "tx/dvbs2/sr", "tx/dvbs2/gainvariable","tx/dvbs2/sdt",
                      "tx/dvbs2/fecmode", "tx/dvbs2/fecrange","tx/dvbs2/rxbbframeip", "tx/dvbs2/tssourcemode", "tx/dvbs2/tssourceaddress", "tx/dvbs2/tssourcefile", "tx/gain", ""};
enum defidx
{
    listcmd,
    cmd_rxstreamrun,
    cmd_rxstreamudpadd,
    cmd_rxstreamoutputtype,
    cmd_rxstreamburst,
    cmd_rxstreammode,
    cmd_rxstreamaverage,
    cmd_txstreamrun,
    cmd_txstreammode,
    cmd_txdvbs2fec,
    cmd_txdvbs2constellation,
    cmd_txdvbs2frame,
    cmd_txdvbs2pilots,
    cmd_txdvbs2sr,
    cmd_txdvbs2gainvariable,
    cmd_txdvbs2sdt,
    cmd_txdvbs2fecmode,
    cmd_txdvbs2fecrange,
    cmd_txdvbs2rxbbframe,
    cmd_txdvbs2tsourcemode,
    cmd_txdvbs2tsourceip,
    cmd_txdvbs2tsourcefile,
    cmd_txgain

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
    
    sprintf(svalue, "PlutoDVB2-%s(F5OEO)", COMIT_FW);
    publish("system/version", svalue);
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        HandleCommand(strcmd[i], "?");
    }
    publish("rx/stream/underflow", (float)Underflow);
    publish("tx/dvbs2/queue", (float)m_bbframe_queue.size());

    if (m_txmode == tx_dvbs2_ts)
    {
        publish("tx/dvbs2/ts/bitrate", float(m_SRtx * (m_efficiency / (float)4e6)));
        publish("tx/dvbs2/ts/fecvariable", (char *)TabFec[m_variable_ts_coderate]);
    }
    if (m_txmode == tx_dvbs2_gse)
    {
        publish("tx/dvbs2/gse/fecvariable", (char *)TabFec[m_variable_gse_coderate]);
        if (m_MaxBBFrameByte != 0)
        {
            publish("tx/dvbs2/gse/efficiency", m_UsedBBFrameByte * 100.0 / (float)m_MaxBBFrameByte);
        }
        m_MaxBBFrameByte = 0;
        m_UsedBBFrameByte = 0;
    }

    /*
    extern float gse_efficiency;
    publish("tx/dvbs2/gseefficiency", gse_efficiency);
    */
}

void SaveToFlash(char *SaveFile) // Not working : Fixme !
{
    FILE *fdwrite = NULL;
    char CompletPath[255];
    strcpy(CompletPath, "/mnt/jffs2/configs/");
    strcat(CompletPath, SaveFile);
    mkdir(CompletPath, 0755);
    strcat(CompletPath, "/stream");
    fdwrite = fopen(SaveFile, "w");
    char svalue[2500];
    sprintf(svalue, "");
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        HandleCommand(strcmd[i], "?");
    }
}

void LoadFromFlash(char *LoadFile) // Not working : Fixme !
{
    /*FILE *fdread = NULL;
    char CompletPath[255];
    strcpy(CompletPath, "/mnt/jffs2/configs/");
    strcat(CompletPath, LoadFile);
    strcat(CompletPath, "/stream");
    fdread = fopen(LoadFile, "r");
    if (fread != NULL)
    {
    }
    */
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
    /*
    case cmd_rxstreamoutputtype:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/output_type", (float)rx_mode);
            break;
        }
        rx_mode = atoi(svalue);
        publish("rx/stream/output_type", (float)rx_mode);
        break;
    }*/
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

    case cmd_rxstreammode:
    {
        
         if (strcmp(svalue, "?") == 0)
        {

            publish("rx/stream/mode", s_rxmode);
            break;
        }
        
        if ((strcmp(svalue, "pass") == 0))
        {
            if (m_rxmode != rx_mode_pass)
            {
                SetRxMode(rx_mode_pass);
                publish("rx/stream/mode", s_rxmode);
                strcpy(s_rxmode, svalue);
            }
            break;
        }
        if ((strcmp(svalue, "webfft") == 0))
        {
            if (m_rxmode != rx_mode_websocket)
            {
                SetRxMode(rx_mode_websocket);
                publish("rx/stream/mode", s_rxmode);
                strcpy(s_rxmode, svalue);
            }
            break;
        }
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
    case cmd_txstreammode:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("tx/stream/mode", s_txmode);

            break;
        }
        if ((strcmp(svalue, "iq") == 0))
        {
            if (m_txmode != tx_iq)
            {
                SetTxMode(tx_iq);
                publish("tx/stream/mode", s_txmode);
                strcpy(s_txmode, svalue);
            }
            break;
        }
        if ((strcmp(svalue, "test") == 0))
        {
            if (m_txmode != tx_test)
            {
                SetTxMode(tx_test);
                publish("tx/stream/mode", s_txmode);
                strcpy(s_txmode, svalue);
            }
            break;
        }
        if ((strcmp(svalue, "pass") == 0))
        {
            if (m_txmode != tx_passtrough)
            {

                SetTxMode(tx_passtrough);
                strcpy(s_txmode, svalue);
                publish("tx/stream/mode", s_txmode);
            }
            break;
        }
        if ((strcmp(svalue, "dvbs2-ts") == 0))
        {
            if (m_txmode != tx_dvbs2_ts)
            {

                SetTxMode(tx_dvbs2_ts);
                strcpy(s_txmode, svalue);
                publish("tx/stream/mode", s_txmode);
            }
            break;
        }
        if ((strcmp(svalue, "dvbs2-gse") == 0))
        {
            if (m_txmode != tx_dvbs2_gse)
            {

                SetTxMode(tx_dvbs2_gse);
                strcpy(s_txmode, svalue);
                publish("tx/stream/mode", s_txmode);
            }
            break;
        }

        break;
    }

    case cmd_txdvbs2fec:
    {

        if (strcmp(svalue, "?") == 0)
        {
            if (m_CodeRate < 11)
                publish("tx/dvbs2/fec", TabFec[m_CodeRate]);
            else
                publish("tx/dvbs2/fec", "none");
            break;
        }

        if ((strcmp("1/2", svalue) == 0) || (strcmp("12", svalue) == 0))
            m_CodeRate = C1_2;
        if ((strcmp("2/3", svalue) == 0) || (strcmp("23", svalue) == 0))
            m_CodeRate = C2_3;
        if ((strcmp("3/4", svalue) == 0) || (strcmp("34", svalue) == 0))
            m_CodeRate = C3_4;
        if ((strcmp("5/6", svalue) == 0) || (strcmp("56", svalue) == 0))
            m_CodeRate = C5_6;
        // DVBS2 specific
        if ((strcmp("1/4", svalue) == 0) || (strcmp("14", svalue) == 0))
            m_CodeRate = C1_4;
        if ((strcmp("1/3", svalue) == 0) || (strcmp("13", svalue) == 0))
            m_CodeRate = C1_3;
        if ((strcmp("2/5", svalue) == 0) || (strcmp("25", svalue) == 0))
            m_CodeRate = C2_5;
        if ((strcmp("3/5", svalue) == 0) || (strcmp("35", svalue) == 0))
            m_CodeRate = C3_5;
        if ((strcmp("4/5", svalue) == 0) || (strcmp("45", svalue) == 0))
            m_CodeRate = C4_5;
        if ((strcmp("8/9", svalue) == 0) || (strcmp("89", svalue) == 0))
            m_CodeRate = C8_9;
        if ((strcmp("9/10", svalue) == 0) || (strcmp("910", svalue) == 0))
            m_CodeRate = C9_10;

        publish("tx/dvbs2/fec", m_CodeRate);
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
    case cmd_txdvbs2pilots:
    {
        if (strcmp(svalue, "?") == 0)
        {

            switch (m_Pilots)
            {
            case 0:
                publish("tx/dvbs2/pilots", "0");
                break;
            case 1:
                publish("tx/dvbs2/pilots", "1");
                break;
            }
            break;
        }
        if (strcmp(svalue, "1") == 0)
            m_Pilots = 1;
        if (strcmp(svalue, "0") == 0)
            m_Pilots = 0;

        publish("tx/dvbs2/pilots", svalue);

        break;
    }
    case cmd_txdvbs2sr:
    {
        if (strcmp(svalue, "?") == 0)
        {
            publish("tx/dvbs2/sr", m_s2sr);
            break;
        }
        m_s2sr = atol(svalue);

        publish("tx/sr", (float)4 * m_s2sr, false);

        break;
    }
    case cmd_txdvbs2gainvariable:
    {
        if (strcmp(svalue, "?") == 0)
        {
            publish("tx/dvbs2/gainvariable", m_gainvariable);
            break;
        }
        m_gainvariable= atoi(svalue);
        publish("tx/dvbs2/gainvariable", m_gainvariable);


        break;
    }
    case  cmd_txdvbs2sdt:
    {
        if (strcmp(svalue, "?") == 0)
        {
            
            break;
        }
        
        updatesdt(svalue);
        break;
    }

   
    case cmd_txgain:
    {
        if (strcmp(svalue, "?") == 0)
        {
            
            break;
        }
        // Get the gain as the max for a variable gain
        m_maxgain= atof(svalue);
        
        break;
    }

    case cmd_txdvbs2fecmode:
    {
        if (strcmp(svalue, "?") == 0)
        {
            if (m_Fecmode == fec_fix)
                publish("tx/dvbs2/fecmode", "fixed");
            if (m_Fecmode == fec_variable)
                publish("tx/dvbs2/fecmode", "variable");
            break;
        }
        if (strcmp(svalue, "fixed") == 0)
        {
            m_Fecmode = fec_fix;
            publish("tx/dvbs2/fecmode", svalue);
        }
        else if (strcmp(svalue, "variable") == 0)
        {
            m_Fecmode = fec_variable;
            publish("tx/dvbs2/fecmode", svalue);
        }
        else
            publish("tx/dvbs2/fecmode", "bad");

        break;
    }

    case cmd_txdvbs2fecrange:
    {
        if (strcmp(svalue, "?") == 0)
        {
           
            publish("tx/dvbs2/fecrange", (float)m_FecRange);
           
            break;
        }
        int fecrange=atoi(svalue);
        if((fecrange>=0)&&(fecrange<=11))
        {
            m_FecRange=fecrange;
            publish("tx/dvbs2/fecrange", (float)m_FecRange);
        }    
        break;
    }

    case cmd_txdvbs2rxbbframe:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("tx/dvbs2/rxbbframeip", mcast_rxiface);
            break;
        }
        setbbframemcast(svalue);
        publish("tx/dvbs2/rxbbframeip", mcast_rxiface);
        break;
    }

    case cmd_txdvbs2tsourcemode:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("tx/dvbs2/tssourcemode",(float) m_tssource);
            break;
        }
        settssource(atoi(svalue), NULL);
        publish("tx/dvbs2/tssourcemode",(float) m_tssource);
        break;
    }

    case cmd_txdvbs2tsourceip:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("tx/dvbs2/tssourceaddress", m_mcast_ts);
            break;
        }
        if(m_tssource==0)
        {
            settssource(-1,svalue);
            publish("tx/dvbs2/tssourceaddress", m_mcast_ts);
        }    
        break;
    }

     case cmd_txdvbs2tsourcefile:
    {
        if (strcmp(svalue, "?") == 0)
        {

            publish("tx/dvbs2/tssourcefile", m_ts_filename);
            break;
        }
        if(m_tssource==1)
        {
            settssource(-1,svalue);
            publish("tx/dvbs2/tssourcefile", m_ts_filename);
        }    
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
            // update_web_param(,m_SR);
            /*
            if (RunRx)
            {
                RunRx = false; // Dirty trick to let some time to get mutex
                // InitRxChannel(m_latency);
                InitRxChannel(fftsize * 10, 2);
                RunRx = true;
            }
            else
            {
                // InitRxChannel(m_latency);
                InitRxChannel(fftsize * 10, 2);
            }*/
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

    if (strcmp(key, "tx/finalsr") == 0)
    {
        if (atol(svalue) != m_SRtx)
        {
            m_SRtx = atol(svalue);

            fprintf(stderr, "New sr %d\n", m_SRtx);

            BufferLentx = ((58192 / 8) + 8) * 2; // MAX BBFRAME LENGTH aligned 8
                                                 // Should be calculated from mm_srtx
            int nbBuffer = ((m_SRtx / 2000000) / 2) * 8;
            pthread_mutex_lock(&bufpluto_mutextx);
            InitTxChannel(BufferLentx, nbBuffer >= 2 ? nbBuffer : 2); // FIXME
            pthread_mutex_unlock(&bufpluto_mutextx);
            setgsesr(m_SRtx);
        }
    }
    return true;
}

void HandleCommandInit(struct mosquitto *mosq, char *sSerial)
{
    m_mosq = mosq;

    sprintf(sDtRoot, "dt/pluto/%s/", sSerial);
    sprintf(sCmdRoot, "cmd/pluto/%s/", sSerial);

    build_crc8_table();

    udp_init();
    // strcpy(m_iface, "127.0.0.1");
    strcpy(m_iface, "192.168.2.1");

    FILE *fgain = fopen("/mnt/jffs2/agctable.txt", "r");
    if (fgain != NULL)
    {
        fprintf(stderr, "AgcTable ");
        // fgets(svalue,255,fdread);
        for (int i = 0; i < 29; i++)
        {
            fscanf(fgain, "%f,", &TheoricMER[i]);
            fprintf(stderr, "%f,", TheoricMER[i]);
        }
        fprintf(stderr, "\n");
    }

    // udp_set_ip("230.0.0.1:1234", m_iface);
    /*
    remove("/dev/rx1");
    mkfifo("/dev/rx1", 0666);
    fdout = fopen("/dev/rx1", "wb");
    InitRxChannel(20000);
    */
    // rx_mode = rx_mode_stdout;
    // rx_mode = rx_mode_websocket;
    // rx_mode = rx_mode_udp;
    m_rxmode = rx_mode_pass;

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