#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
#include <getopt.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <netinet/in.h> /* IPPROTO_IP, sockaddr_in, htons(),
htonl() */
#include <arpa/inet.h>  /* inet_addr() */
#include <netdb.h>
#include <time.h>
#include <sys/ioctl.h>
#include <queue>
#include "ts_util/pcrpts.h"
#include "./dvbs2neon/dvbs2neon.h"
#include "./dvbsarm/fec100.h"

extern "C"
{
    extern void dvbsenco_init(void);
    extern uchar *dvbsenco(uchar *);
}

using namespace std;
#define WITH_NEON
//#define UDP_BUFF_MAX_BBFRAME (58192 / 8)
#define UDP_BUFF_MAX_BBFRAME 8000

extern int m_Fecmode;
enum
{
    fec_fix,
    fec_variable
};

typedef struct
{
    ssize_t size;
    ssize_t modecod;
    uint8_t bbframe[UDP_BUFF_MAX_BBFRAME];
} buffer_t;

extern queue<buffer_t *> m_bbframe_queue;
extern pthread_mutex_t buffer_mutextx;
extern int m_txmode;
extern uint32_t m_efficiency;
extern int m_FecRange;

struct bbheader
{
    uint8_t matype1;
    uint8_t matype2;
    uint16_t upl;
    uint16_t dfl;
    uint8_t sync;
    uint8_t syncd1;
    uint8_t syncd2;
    uint8_t crc;
};

uchar BBFrameNeonBuff[144000] __attribute__((aligned(128)));
uchar symbolbuff[144 * 1024] __attribute__((aligned(16)));

static DVB2FrameFormat fmt;
enum
{
    longframe,
    shortframe

};
enum
{
    mod_qpsk,
    mod_8psk,
    mod_16apsk,
    mod_32apsk
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
int m_ModeCod = C2_3 + longframe;
uint8_t m_variable_ts_coderate;
#define MAX_QUEUE_ITEM 100
#define MAX_QUEUE_CHANGEMODCOD 2

enum
{
    tssource_udp,
    tssource_file,
    tssource_pattern
};

enum
{
    tx_passtrough,
    tx_iq,
    tx_dvbs2_ts,
    tx_dvbs2_gse,
    tx_test,
    tx_dvbs,
    tx_dvbt

};

int m_tssource = tssource_udp;
char m_mcast_ts[255];
char m_mcast_iface[255];
char m_ts_filename[255];
uint m_ts_filebitrate = 10000;
FILE *fdtsinput = NULL;
int m_FileBitrate = 100000;
int m_LatencySevenPacket = 1000;
size_t Lastpidccerror = 8192;

bool addbbframe(uint8_t *bbframe, size_t len, size_t modcod)
{
    if ((m_txmode != tx_dvbs2_ts)&&(m_txmode != tx_dvbs))
        return false;
    pthread_mutex_lock(&buffer_mutextx);
    buffer_t *newbuf = (buffer_t *)malloc(sizeof(buffer_t));
    newbuf->size = len;
    newbuf->modecod = modcod;
    // newbuf->modecod=m_ModeCod;
    memcpy(newbuf->bbframe, bbframe, len);

    /*
    if ((m_bbframe_queue.size() > MAX_QUEUE_CHANGEMODCOD)&&(m_bbframe_queue.size() < MAX_QUEUE_ITEM))
    {
        fprintf(stderr, "Queue is full, change modecod ! \n");
        DVB2FrameFormat tempmodecode;
        tempmodecode=fmt;
        tempmodecode.fec+=1;
        newbuf->modecod=m_ModeCod+1;
        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&tempmodecode, 0);
    }
    */
    if ((m_bbframe_queue.size() >= MAX_QUEUE_ITEM))
    {
        fprintf(stderr, "MUXTS : Queue is full ! Purging %d bbframe\n", m_bbframe_queue.size());
        // newbuf->modecod=m_ModeCod;
        while (m_bbframe_queue.size() > 1)
        {
            buffer_t *oldestbuf = m_bbframe_queue.front(); // Remove the oldest

            free(oldestbuf);

            m_bbframe_queue.pop();
        }
    }

    /*if (m_bbframe_queue.size() <= MAX_QUEUE_CHANGEMODCOD)
    {

        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    }
    */
   //fprintf(stderr,"Add bbframe of %d\n",newbuf->size);
    m_bbframe_queue.push(newbuf);
    pthread_mutex_unlock(&buffer_mutextx);
    return true;
}

size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen)
{

    size_t rcvlen = recv(sock, b, maxlen, 0 /* MSG_ZEROCOPY*/);
    return rcvlen;
}

u_int16_t udp_init(char *ip, const char *iface, int rx) // interface to multicast from
{

    u_int16_t sock;
    struct sockaddr_in addr;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    /* try to reuse the socket */
    int len = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &len, sizeof(len));

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

    char text[40];
    char *add[2];
    u_int16_t port;
    strcpy(text, ip);
    add[0] = strtok(text, ":");
    add[1] = strtok(NULL, ":");
    if (strlen(add[1]) == 0)
        port = 1314;
    else
        port = atoi(add[1]);
    /* bind the socket on given port */
    memset(&addr, 0, sizeof(addr));           // Clear struct
    addr.sin_family = AF_INET;                // Internet/IP
    addr.sin_addr.s_addr = inet_addr(add[0]); // IP address
    addr.sin_port = htons(port);

    bind(sock, (struct sockaddr *)&addr, sizeof(addr));

    if (rx)
    {
        struct ip_mreq imr;
        memset(&imr, 0, sizeof(struct ip_mreq));
        inet_aton(add[0], (struct in_addr *)&(imr.imr_multiaddr.s_addr));
        // imr.imr_multiaddr = inet_addr(add[0]);
        if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *)&imr,
                       sizeof(struct ip_mreq)) < 0)
        {
            perror("setsockopt: join multicast group with IP_ADD_MEMBERSHIP failed\n");
            // exit(1);
        }
    }
    if (iface != NULL)
    {
        struct in_addr localInterface;
        localInterface.s_addr = inet_addr(iface);
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, (char *)&localInterface, sizeof(localInterface));
    }
    else
    {
        setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, NULL, 0);
    }

    return sock;
}

#define CRC_POLYR 0xD5
uint8_t m_crc_tab_r[256];
void build_crc8_table_r(void)
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
        m_crc_tab_r[i] = crc;
    }
}

uint8_t calc_crc8_r(uint8_t *b, int len)
{
    uint8_t crc = 0;

    for (int i = 0; i < len; i++)
    {
        crc = m_crc_tab_r[b[i] ^ crc];
    }
    return crc;
}

#define MAX_BBFRAME (58192 / 8)

unsigned int BBFrameLenLut2[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0, 16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};
// extern unsigned int BBFrameLenLut[];

u_int16_t recv_ts_sock;

pthread_mutex_t buffer_mutexts;
DVB2FrameFormat tempmodecode;
uint8_t *customsdt;
uint8_t customnullpacket[188];

uint8_t *sdt_fmt(int stream_id, int network_id, int service_id, char *service_provider_name, char *service_name);
void update_cont_counter(uint8_t *b);

void correctcc(uint8_t *tspacket, bool discountinuity)
{
    static uint8_t pid_cc_table[8190];
    static int start = 1;
    unsigned short pid;
    if (start == 1)
    {
        memset(pid_cc_table, 0x10, 8190);
        start = 0;
    }

    memcpy(&pid, tspacket + 1, 2);
    pid = ntohs(pid);
    pid = pid & 0x1fff;
    // fprintf(stderr, "pid entry %d\n", pid);
    if (pid < 8190)
    {
        if (pid_cc_table[pid] == 0x10)
        {
            // fprintf(stderr, "new pid entry %d\n", pid);
            pid_cc_table[pid] = 0;
        }
        else
        {
            if (((tspacket[3] & 0x30) == 0x10) || ((tspacket[3] & 0x30) == 0x30))
                pid_cc_table[pid] = (pid_cc_table[pid] + 1) % 0x10;
            else
                tspacket[5] = (discountinuity) ? (tspacket[5] & 0x7F) | 0x8 : tspacket[5];
            // fprintf(stderr, "pid %d cc %d\n", pid,pid_cc_table[pid]);
        }

        tspacket[3] = (pid_cc_table[pid] | (tspacket[3] & 0xf0));
    }
}

void addneonts(uint8_t *tspacket, size_t length)
{
    unsigned short *bbframeptr = NULL;
    uint8_t *cur_packet = tspacket;

    if (length % 188 != 0)
    {
        fprintf(stderr, "Ts input error len %d\n", length);
        return;
    }
    for (int i = 0; i < length / 188; i++)
    {
        // pthread_mutex_lock(&buffer_mutexts);
        if (cur_packet[0] != 0x47)
        {
            fprintf(stderr, "Ts input error aligned %x\n", cur_packet[0]);
            return;
        }
        if ((m_Fecmode == fec_variable) && GetPid((char *)cur_packet) == 0x1FFF) // Remove TS padding
        {
            cur_packet += 188;

            continue;
            // Nothing to add
        }
        ProcessCorectPCR(cur_packet, 188); // Correct PCR

        if (GetPid((char *)cur_packet) == 0x11) // replace sdt
        {
            bbframeptr = (unsigned short *)dvbs2neon_packet(0, (uint32)(customsdt), 0);
            update_cont_counter(customsdt);
        }
        else if (GetPid((char *)cur_packet) == 0x1FFF)
        {
            bbframeptr = (unsigned short *)dvbs2neon_packet(0, (uint32)(customnullpacket), 0);
        }
        else
        {
            size_t piderror = InspectCC(cur_packet, 188);
            if (piderror != 8192)
                Lastpidccerror = piderror;
            bbframeptr = (unsigned short *)dvbs2neon_packet(0, (uint32)(cur_packet), 0);
        }
        cur_packet += 188;
        if (bbframeptr != NULL)
        {
            static int count = 0;
            // bbframeptr = (unsigned char *)dvbs2neon_control(STREAM0, CONTROL_GET_LAST_BBFRAME, (uint32)BBFrameNeonBuff, 0);

            unsigned short *p16 = (unsigned short *)bbframeptr;
            unsigned short ByteCount = p16[-1];
            uchar *p8 = (uchar *)bbframeptr;
            uchar output_format = p8[-3];
            uchar fec = p8[-5];
            uchar frame_type = p8[-7];
            // fprintf(stderr,"bbframe %d\n",ByteCount);
            int i = 0;

            for (i = 0; i < sizeof(BBFrameLenLut2); i++)
            {
                if (BBFrameLenLut2[i] / 8 == ByteCount)
                    break;
            }

            int coderate;
            if (i >= 11) // longframe
            {
                coderate = i - 11;
            }
            else
                coderate = i;
            extern unsigned char getdvbs2modcod(uint FrameType, uint Constellation, uint CodeRate, uint Pilots);
            unsigned char curmodcod = getdvbs2modcod(fmt.frame_type == 0 ? 0 : 1, fmt.constellation, coderate, fmt.pilots);

            m_variable_ts_coderate = coderate;
            addbbframe((uint8_t *)bbframeptr, ByteCount, curmodcod);

            if ((m_Fecmode == fec_variable) /*&& (m_bbframe_queue.size() >= 2)*/)
            {
                tempmodecode = fmt;
                int fecoffset = (m_bbframe_queue.size() / 2);
                if (fecoffset > m_FecRange)
                {
                    fecoffset = m_FecRange;
                }

                if (fecoffset + tempmodecode.fec > 10)
                    tempmodecode.fec = 10;
                else
                    tempmodecode.fec += fecoffset;
                // fprintf(stderr,"Variable queu %d fec %d\n",m_bbframe_queue.size(),tempmodecode.fec);
                int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&tempmodecode, 0);
            }
            else
            {
                int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
            }
            m_efficiency = dvbs2neon_control(STREAM0, CONTROL_GET_EFFICIENCY, 0, 0);
            /*
              addbbframe((uint8_t *)bbframeptr, ByteCount,m_ModeCod+(count+1)%2);

              tempmodecode.constellation=fmt.constellation;
              tempmodecode.fec=fmt.fec;
              tempmodecode.frame_type=fmt.frame_type;
              tempmodecode.output_format=fmt.output_format;
              tempmodecode.pilots=fmt.pilots;
              tempmodecode.roll_off=fmt.roll_off;
              //fprintf(stderr,"neon %d %d \n",fmt.fec,fmt.frame_type);
              tempmodecode.fec+=count;
              int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&tempmodecode, 0);
              count=(count+1)%2;
              */
            // pthread_mutex_unlock(&buffer_mutexts);
        }
    }
}

void adddvbsframe(uint8_t *tspacket)
{
    static unsigned char Buff[208];
    static unsigned char *pRS204;
    static unsigned char packedSymbol[408]; // 204*2bits/symbol /8 (byte)
    static unsigned char symbols[408 * 4];  // 204*2bits/symbol  (byte)
    static uint16_t Frame[408*4*2];
    uint16 NbIQOutput;
    uint16 NbSymbolsTotal = 0;
    uint16_t *FrameCurrentSymbol=Frame;
    uint16_t magnitude=0x7FFF/sqrt(2);
    memcpy(Buff, tspacket, 188);

    pRS204 = dvbsenco(Buff);
    NbIQOutput = viterbi(pRS204, packedSymbol);
    //fprintf(stderr,"viterbi out %d\n",NbIQOutput);
    for (int k = 0; k < NbIQOutput; k++)
    {
        for (int j = 0; j < 4; j++)
        {
            //symbols[k * 4 + j] = (packedSymbol[k] >> (6 - j * 2)) & 0x3;
            switch((packedSymbol[k] >> (6 - j * 2)) & 0x3)
            {
                case 0: *FrameCurrentSymbol++=magnitude;*FrameCurrentSymbol++=magnitude;break;
                case 1: *FrameCurrentSymbol++=magnitude;*FrameCurrentSymbol++=-magnitude;break;
                case 2: *FrameCurrentSymbol++=-magnitude;*FrameCurrentSymbol++=magnitude;break;
                case 3: *FrameCurrentSymbol++=-magnitude;*FrameCurrentSymbol++=-magnitude;break;
            }
            NbSymbolsTotal++;
            //fprintf(stderr,"Symbol %d -> %d",(packedSymbol[k] >> (6 - j * 2)) & 0x3,*(FrameCurrentSymbol-1));
        }
    }
    //fprintf(stderr,"Nb IQ %d\n",NbSymbolsTotal);
    addbbframe((uint8_t *)Frame,NbSymbolsTotal*4,0); //IQ = 4 bytes
}

void adddvbsts(uint8_t *tspacket, size_t length)
{

       uint8_t *cur_packet = tspacket;

    if (length % 188 != 0)
    {
        fprintf(stderr, "Ts input error len %d\n", length);
        return;
    }
    for (int i = 0; i < length / 188; i++)
    {
        // pthread_mutex_lock(&buffer_mutexts);
        if (cur_packet[0] != 0x47)
        {
            fprintf(stderr, "Ts input error aligned %x\n", cur_packet[0]);
            return;
        }
       
        ProcessCorectPCR(cur_packet, 188); // Correct PCR

        if (GetPid((char *)cur_packet) == 0x11) // replace sdt
        {
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(customsdt), 0);
            adddvbsframe(customsdt);
           
            update_cont_counter(customsdt);
        }
        else if (GetPid((char *)cur_packet) == 0x1FFF)
        {
            adddvbsframe(customnullpacket);
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(customnullpacket), 0);
        }
        else
        {
            size_t piderror = InspectCC(cur_packet, 188);
            if (piderror != 8192)
                Lastpidccerror = piderror;
            adddvbsframe(cur_packet);
            // bbframeptr = (unsigned short *) dvbs2neon_packet(0, (uint32)(cur_packet), 0);
        }
        cur_packet += 188;
       
    }
}

void addts(uint8_t *tspacket, size_t length)
{
    static unsigned char bbframe[MAX_BBFRAME];
    uint16_t framebytes = BBFrameLenLut2[m_ModeCod] / 8; // Need to set it by modcod
    static uint16_t avail = framebytes - 10;
    static uint8_t lastcrc = 0;
    struct bbheader *header = (struct bbheader *)bbframe;
    static int index = 10;
    static uint16_t remain = 0;

    uint16_t tsindex = 0;

    while (tsindex < length)
    {
        uint8_t crc = calc_crc8_r(tspacket + tsindex + 1, 187);
        if (avail >= 188)
        {

            /*
            bbframe[index++] = lastcrc;
            lastcrc = crc;
            avail--;*/
            memcpy(bbframe + index, tspacket + tsindex + 1, 188 - 1);
            tsindex += 188;
            index += 187;
            avail -= 187;
            bbframe[index++] = crc;
            avail--;
        }
        else if (avail == 0) // BBFrame is full
        {
            // sendudpbbframe
            // fprintf(stderr,"Complete full\n");
            if (remain == 0)
                remain = 187;
            header->matype1 = 0xF0; // TS 0.35roff

            header->matype2 = 0;                        // Input Stream Identifier
            header->upl = htons(188 * 8);               // User Packet Length 188
            header->dfl = htons((framebytes - 10) * 8); // Data Field Length
            header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
            header->syncd1 = (remain * 8) >> 8;         // SYNCD
            header->syncd2 = (remain * 8) & 0xFF;       // SYNCD
            header->crc = calc_crc8_r(bbframe, 9);      // CRC

            // udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes);
            addbbframe(bbframe, framebytes, m_ModeCod);

            index = 10;

            avail = framebytes - 10;
        }
        else // Some space in bbframe, full with partial packet
        {

            /*
           bbframe[index++] = lastcrc;
           lastcrc = crc;
          */
            if (remain == 0)
                remain = 187;
            /*
            else
            remain--;
            */
            header->matype1 = 0xF0; // TS 0.35roff

            header->matype2 = 0;                        // Input Stream Identifier
            header->upl = htons(188 * 8);               // User Packet Length 188
            header->dfl = htons((framebytes - 10) * 8); // Data Field Length
            header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
            header->syncd1 = (remain * 8) >> 8;         // SYNCD
            header->syncd2 = (remain * 8) & 0xFF;       // SYNCD
            header->crc = calc_crc8_r(bbframe, 9);      // CRC

            remain = 187 - avail;
            memcpy(bbframe + index, tspacket + tsindex + 1, avail);
            index += avail;
            // fprintf(stderr,"Remain full remain %d %d\n",avail,index);

            /*
            static int count=0;
                for(int i=0;i<  framebytes;i++)
          fprintf(stderr,"%02x ", bbframe[i]);
          fprintf(stderr,"\n************************************************\n");

          if(count++==2)          exit(1);
          */

            // udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes);
            addbbframe(bbframe, framebytes, m_ModeCod);
            // sendudpbbframe

            index = 10;
            memcpy(bbframe + index, tspacket + tsindex + avail + 1, remain);
            index += remain;
            bbframe[index++] = crc;
            tsindex += 188;

            avail = framebytes - 10 - remain - 1;
        }
    }
}

#define MAX_PID 8191
#define TS_PACKET_SIZE 188
#define SYSTEM_CLOCK_FREQUENCY 27000000

int GetTsBitrate()
{
    // Courtsy taken from OpenCaster toolbox , GPL
    int fd_ts; /* File descriptor of ts file */
    u_short pid;
    int byte_read;
    unsigned int pcr_ext = 0;
    unsigned int ibits = 0;
    unsigned long long int pcr_base = 0;
    unsigned long long int ts_packet_count;
    unsigned long long int new_pcr = 0;
    unsigned long long int new_pcr_index = 0;
    unsigned long long int pid_pcr_table[MAX_PID];       /* PCR table for the TS packets */
    unsigned long long int pid_pcr_index_table[MAX_PID]; /* PCR index table for the TS packets */
    unsigned long long int pid_sum[MAX_PID];             /* Sum of each PID TS packets */
    unsigned char ts_packet[TS_PACKET_SIZE];             /* TS packet */

    unsigned long long int TsBitrate = 0;

    /* Start to process the file */
    memset(pid_pcr_table, 0, MAX_PID * (sizeof(unsigned long long int)));
    memset(pid_pcr_index_table, 0, MAX_PID * (sizeof(unsigned long long int)));
    memset(pid_sum, 0, MAX_PID * (sizeof(unsigned long long int)));
    ts_packet_count = 0;
    byte_read = 1;
    int nbnewpcr = 0;
    while (TsBitrate == 0)
    {

        /* Read next packet */

        byte_read = fread(ts_packet, TS_PACKET_SIZE, 1, fdtsinput);
        /* check packet */
        memcpy(&pid, ts_packet + 1, 2);
        pid = ntohs(pid);
        pid = pid & 0x1fff;
        if (pid < MAX_PID)
        {
            if ((ts_packet[3] & 0x20) && (ts_packet[4] != 0) && (ts_packet[5] & 0x10))
            { /* there is a pcr field */
                pcr_base = (((unsigned long long int)ts_packet[6]) << 25) + (ts_packet[7] << 17) + (ts_packet[8] << 9) + (ts_packet[9] << 1) + (ts_packet[10] >> 7);
                pcr_ext = ((ts_packet[10] & 1) << 8) + ts_packet[11];
                if (pid_pcr_table[pid] == 0)
                {
                    pid_pcr_table[pid] = pcr_base * 300 + pcr_ext;
                    pid_pcr_index_table[pid] = (ts_packet_count * TS_PACKET_SIZE) + 10;
                    /*fprintf(stdout, "%llu: pid %d, new pcr is %llu (%f sec)\n",
                                    pid_pcr_index_table[pid],
                                    pid,
                                    pid_pcr_table[pid],
                                    ((double)(pid_pcr_table[pid]) / SYSTEM_CLOCK_FREQUENCY));*/
                }
                else
                {

                    new_pcr = pcr_base * 300 + pcr_ext;
                    new_pcr_index = (ts_packet_count * TS_PACKET_SIZE) + 10;
                    /*fprintf(stderr, "%llu: pid %d, new pcr is %llu (%f sec), pcr delta is %llu, (%f ms), indices delta is %llu bytes,instant ts bit rate is %.10f\n",
                                    new_pcr_index,
                                    pid,
                                    new_pcr,
                                    ((double)(new_pcr) / SYSTEM_CLOCK_FREQUENCY),
                                    new_pcr - pid_pcr_table[pid],
                                    ((double)((new_pcr - pid_pcr_table[pid]) * 1000)) / SYSTEM_CLOCK_FREQUENCY,
                                    new_pcr_index - pid_pcr_index_table[pid],
                                    (((double)(new_pcr_index - pid_pcr_index_table[pid])) * 8 * SYSTEM_CLOCK_FREQUENCY) / ((double)(new_pcr - pid_pcr_table[pid]))
                                    );*/

                    double ftsbitrate = (((double)(new_pcr_index - pid_pcr_index_table[pid])) * 8 * SYSTEM_CLOCK_FREQUENCY) / ((double)(new_pcr - pid_pcr_table[pid]));
                    nbnewpcr++;
                    if (nbnewpcr > 8) // Wait 10 pcr for average
                        TsBitrate = (unsigned long long int)ftsbitrate;
                    unsigned long long int videobitrate = (TsBitrate * pid_sum[pid]) / ts_packet_count;
                    // fprintf(stderr, "Ts %f %ll Video %ll\n", ftsbitrate, TsBitrate, videobitrate);

                    pid_pcr_table[pid] = new_pcr;
                    pid_pcr_index_table[pid] = new_pcr_index;
                }
            }
            pid_sum[pid]++;
            ts_packet_count++;
        }
    }
    fseek(fdtsinput, 0, SEEK_SET);
    return TsBitrate;
}

void *rx_ts_thread(void *arg)
{
    unsigned char tspacket[7 * 188 * 10];

    int length = 0;

    while (true)
    {
        if (m_tssource == tssource_udp)
            length = udp_receive(recv_ts_sock, tspacket, 7 * 188);

        if ((m_tssource == tssource_file) || (m_tssource == tssource_pattern))
        {
            if (fdtsinput == NULL)
            {
                usleep(5000);
                continue;
            }
            if (feof(fdtsinput)) // Loop File
            {
                fseek(fdtsinput, 0, SEEK_SET);
                // correctcc(tspacket,true);
            }
            if (m_bbframe_queue.size() < 10)
            {
                length = fread(tspacket, 1, 7 * 188, fdtsinput);
                // usleep(m_LatencySevenPacket * 1000);
            }
            else
            {
                usleep(m_LatencySevenPacket * 1000);
                continue;
            }
        }
        /*
        int udpsize=0;
        ioctl(recv_ts_sock, FIONREAD, &udpsize);
        while(udpsize>0)
        {
            length += udp_receive(recv_ts_sock, tspacket+length, 7*188);
            ioctl(recv_ts_sock, FIONREAD, &udpsize);
        }
        */

        // fprintf(stderr,"Udp %d / %d\n",length,udpsize);
        if (length > 0)
        {
            pthread_mutex_lock(&buffer_mutexts);
            if (m_txmode == tx_dvbs2_ts)
            {
                addneonts(tspacket, length);
            }
            if (m_txmode == tx_dvbs)
            {
                
                adddvbsts(tspacket, length);
            }
            pthread_mutex_unlock(&buffer_mutexts);
        }
    }
}

void setneonmodcod(uint Constellation, uint CodeRate, uint FrameType, uint Pilots, int filter)
{

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
    fmt.output_format = OUTPUT_FORMAT_BBFRAME;
    if (Pilots)
        fmt.pilots = PILOTS_ON;
    else
        fmt.pilots = PILOTS_OFF;
    if (filter == 0)
        fmt.roll_off = RO_0_20;
    if (filter == 1)
        fmt.roll_off = RO_0_15;

    // FixMe : Modcod should be changed ONLY when a bbframe is complete or at startup
    int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    modulator_mapping(fmt.constellation, CodeRate);

    m_efficiency = dvbs2neon_control(STREAM0, CONTROL_GET_EFFICIENCY, 0, 0);

    // pthread_mutex_unlock(&buffer_mutexts);
}

void setdvbsfec(uint CodeRate)
{
    switch(CodeRate) //Retrict fec allowed in dvbs
    {
        case C1_4:
        case C1_3:
        case C2_5:
        case C1_2: CodeRate=1; m_efficiency=(1e6*188*2)/(2*204);break;

    case C3_5:
    case C2_3: CodeRate=2;m_efficiency=(1e6*188*2*2)/(3*204);break;
    case C3_4 : CodeRate=3;m_efficiency=(1e6*188*3*2)/(4*204);break;
    case C4_5 : 
    case C5_6 :CodeRate=5;m_efficiency=(1e6*188*5*2)/(6*204);break;
    case C8_9 :
    case C9_10 : CodeRate=7;m_efficiency=(1e6*188*7*2)/(8*204);break;
    }
    fprintf(stderr,"DVBS FEC %d\n",CodeRate);
    dvbsenco_init();
    viterbi_init(CodeRate); // 1/2
    
}

void settssource(int tssource, char *arg, uint tsbitrate)
{
    if (tssource != -1)
    {
        m_tssource = tssource;
        fprintf(stderr, "Change source %d\n", m_tssource);
    }
    switch (m_tssource)
    {
    case tssource_udp:
    {
        if (arg == NULL)
            break;
        strcpy(m_mcast_ts, arg);
        fprintf(stderr, "Try udp %s\n", m_mcast_ts);
        // fixmme should close previous socket
        recv_ts_sock = udp_init(m_mcast_ts, m_mcast_iface, 1);
    }
    break;
    case tssource_file:
    {
        if (arg != NULL)
        {
            fprintf(stderr, "Setting file ts %s\n", arg);
            strcpy(m_ts_filename, arg);
            if (fdtsinput != NULL)
                fclose(fdtsinput);
            fdtsinput = fopen(m_ts_filename, "rb");
            m_FileBitrate = GetTsBitrate();
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;
            fprintf(stderr, "File bitrate = %d Latency us %d\n", m_FileBitrate, m_LatencySevenPacket * 1000);
        }
        if (tsbitrate > 0)
        {
            fprintf(stderr, "Setting file ts bitrate %d\n", tsbitrate);
            m_FileBitrate = tsbitrate;
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;
        }
    }
    break;
    case tssource_pattern:
    {
        if (fdtsinput != NULL)
            fclose(fdtsinput);
        fdtsinput = fopen("/root/mire.ts", "rb");
        m_FileBitrate = GetTsBitrate();
        if (m_FileBitrate > 0)
            m_LatencySevenPacket = (1000 * 7 * 188 * 8L) / (long)m_FileBitrate;

        fprintf(stderr, "Patern bitrate = %d Latency us %d\n", m_FileBitrate, m_LatencySevenPacket * 1000);
    }
    break;
    }
}

void setpaddingts()
{
    static unsigned char NullPacket[7 * 188] = {0x47, 0x1F, 0xFE, 0x10, 'F', '5', 'O', 'E', 'O'};
    static unsigned char cc = 0;
    for (int k = 0; k < 1; k++)
    {
        memcpy(NullPacket + k * 188, NullPacket, 9);
        NullPacket[3 + k * 188] = 0x10 + cc;
        cc = (cc + 1) % 16;
        for (int i = 9; i < 188; i++)
            NullPacket[i + k * 188] = i;
    }
    pthread_mutex_lock(&buffer_mutexts);

    if(m_txmode == tx_dvbs2_ts)
        addneonts(NullPacket, /*7 **/ 188);
    if(m_txmode == tx_dvbs ) 
    {
        adddvbsts(NullPacket,188);
    }
    pthread_mutex_unlock(&buffer_mutexts);
}

void updatesdt(char *custom)
{
    FILE *cmd = popen("fw_printenv -n call", "r");
    char result[255] = {0x0};
    // fgets(result, sizeof(result), cmd);
    fscanf(cmd, "%s", result);
    if (strcmp(result, "") == 0)
        strcpy(result, "nocall");
    pclose(cmd);
    char sdt[512] = {0x0};

    if (custom)
    {
        sprintf(sdt, "%s%s", custom, result);
    }
    else
    {
        sprintf(sdt, "%s", result);
    }

    // fprintf(stderr,"SDT %s\n",sdt);
    char provider[255] = {0x0};
    sprintf(provider, "PlutoDVB2-%s(F5OEO)", COMIT_FW);
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += provider[i];
    }
    if (sum != 0x34F)
    {
        exit(1);
    }
    // fprintf(stderr,provider);
    customsdt = sdt_fmt(1, 1, 1, provider, sdt);

    char *byte = (char *)&customnullpacket[0];
    *byte++ = 0x47;
    *byte++ = 0x1F;
    *byte++ = 0xFF;
    *byte++ = 0x10;
    strcpy(byte, provider);
    byte += strlen(provider);
    char squeue[50];
    sprintf(squeue, "Queue:%d", m_bbframe_queue.size()); // Fixme ; this should be update regularly not only at start
    strcpy(byte, squeue);
    byte += strlen(squeue);
    sprintf(provider, "Comit:%s", COMIT_FW);
    strcpy(byte, provider);
    byte += strlen(provider);
    *byte++ = 0xFF;
}

#ifndef COMIT_FW
#define COMIT_FW "OUT_OF_TREE"
#endif
static pthread_t p_rxts;
void init_tsmux(char *mcast_ts, char *mcast_iface)
{

    updatesdt("");
    // DVBS2 INIT
    int status1 = dvbs2neon_control(0, CONTROL_RESET_FULL, (uint32)symbolbuff, sizeof(symbolbuff));
    int status2 = dvbs2neon_control(STREAM0, CONTROL_RESET_STREAM, 0, DATAMODE_TS);
    fmt.fec = 0;
    fmt.frame_type = FRAME_NORMAL;
    fmt.output_format = OUTPUT_FORMAT_BBFRAME;
    fmt.pilots = PILOTS_OFF;
    fmt.roll_off = RO_0_20;
    int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);

    // DVBS INIT
    dvbsenco_init();
    viterbi_init(1); // 1/2

    // Other inits
    strcpy(m_mcast_ts, mcast_ts);
    strcpy(m_mcast_iface, mcast_iface);

    recv_ts_sock = udp_init(m_mcast_ts, m_mcast_iface, 1);
    build_crc8_table_r();
    pthread_mutex_init(&buffer_mutexts, NULL);
    pthread_create(&(p_rxts), NULL, &rx_ts_thread, NULL);
}