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

#include "./dvbs2neon/dvbs2neon.h"

using namespace std;
#define WITH_NEON
//#define UDP_BUFF_MAX_BBFRAME (58192 / 8)
#define UDP_BUFF_MAX_BBFRAME 8000
typedef struct
{
    ssize_t size;
    ssize_t modecod;
    uint8_t bbframe[UDP_BUFF_MAX_BBFRAME];
} buffer_t;

extern queue<buffer_t *> m_bbframe_queue;
extern pthread_mutex_t buffer_mutextx;


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
int m_ModeCod = C2_3 + longframe;

#define MAX_QUEUE_ITEM 10
#define MAX_QUEUE_CHANGEMODCOD 2


bool addbbframe(uint8_t *bbframe, size_t len,size_t modcod)
{
    
    pthread_mutex_lock(&buffer_mutextx);
    buffer_t *newbuf = (buffer_t *)malloc(sizeof(buffer_t));
    newbuf->size = len;
    newbuf->modecod=modcod;
    //newbuf->modecod=m_ModeCod;
    memcpy(newbuf->bbframe,bbframe,len);
    
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
        fprintf(stderr, "Queue is full ! Purging %d bbframe\n",m_bbframe_queue.size());
        //newbuf->modecod=m_ModeCod;
        while (m_bbframe_queue.size()>0)
        {
            buffer_t *oldestbuf = m_bbframe_queue.front(); // Remove the oldest
            fprintf(stderr,"before\n");
            free(oldestbuf);
            fprintf(stderr,"after\n");
            m_bbframe_queue.pop();
        }    
    }
    
    /*if (m_bbframe_queue.size() <= MAX_QUEUE_CHANGEMODCOD)
    {
        
        int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    }
    */    
    m_bbframe_queue.push(newbuf);
    pthread_mutex_unlock(&buffer_mutextx);
    return true;
}

inline size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen)
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

// unsigned int BBFrameLenLut[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0,              16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};
extern unsigned int BBFrameLenLut[];

u_int16_t recv_ts_sock;


pthread_mutex_t buffer_mutexts;
DVB2FrameFormat  tempmodecode;
void addneonts(uint8_t *tspacket, size_t length)
{
    unsigned char *bbframeptr = NULL;
    uint8_t *cur_packet = tspacket;
    
    for (int i = 0; i < length / 188; i++)
    {
        //pthread_mutex_lock(&buffer_mutexts);    
        bbframeptr = (unsigned char *)dvbs2neon_packet(0, (uint32)(cur_packet), 0);
        cur_packet += 188;
        if (bbframeptr != NULL)
        {
            static int count=0;
            bbframeptr = (unsigned char *)dvbs2neon_control(STREAM0, CONTROL_GET_LAST_BBFRAME, (uint32)BBFrameNeonBuff, 0);

            unsigned short *p16 = (unsigned short *)bbframeptr;
            unsigned short ByteCount = p16[-1];
            uchar *p8 = (uchar *)bbframeptr;
            uchar output_format = p8[-3];
            uchar fec = p8[-5];
            uchar frame_type = p8[-7];
            //fprintf(stderr,"bbframe %d\n",ByteCount);
          
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
            
            //pthread_mutex_unlock(&buffer_mutexts);
        }
    }
    
}

void addts(uint8_t *tspacket, size_t length)
{
    static unsigned char bbframe[MAX_BBFRAME];
    uint16_t framebytes = BBFrameLenLut[m_ModeCod] / 8; // Need to set it by modcod
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
            addbbframe(bbframe, framebytes,m_ModeCod);

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
            addbbframe(bbframe, framebytes,m_ModeCod);
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

#ifdef OLD
void *rx_ts_thread(void *arg)
{

    size_t read_length;
    // char recv_ts_ip[] = "230.10.0.1:1234";
    // recv_ts_sock = udp_init(recv_ts_ip, "192.168.1.39", 1);
    unsigned char bbframe[MAX_BBFRAME];
    unsigned char tspacket[7 * 188];

    uint16_t remain = 0;
    uint16_t framebytes = BBFrameLenLut[m_ModeCod] / 8; // Need to set it by modcod
    uint16_t avail = framebytes - 10;
    uint8_t lastcrc = 0;
    struct bbheader *header = (struct bbheader *)bbframe;
    fprintf(stderr, "Starting ts input \n");
    unsigned char NullPacket[188] = {0x47, 0x1F, 0xFE, 0x10, 'F', '5', 'O', 'E', 'O'};
    for (int i = 9; i < 188; i++)
        NullPacket[i] = i;
    int index = 10;

    header->matype1 = 0xF0; // Generic continuous 0.35roff

    header->matype2 = 0;                        // Input Stream Identifier
    header->upl = htons(188 * 8);               // User Packet Length 188
    header->dfl = htons((framebytes - 10) * 8); // Data Field Length
    header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
    header->syncd1 = 0x00;                      // SYNCD
    header->syncd2 = 0x00;                      // SYNCD
    header->crc = calc_crc8_r(bbframe, 9);      // CRC

    while (true)
    {
        int length = 0;

        length = udp_receive(recv_ts_sock, tspacket, 7 * 188);

        /*
        for(int i=0;i<7;i++)
        {
            memcpy(tspacket+i*188,NullPacket,188);
        }
        length=7*188;
          */
        // fprintf(stderr,"avail %d index %d\n",avail,index);

        if (length % 188 != 0)
        {
            fprintf(stderr, "ts not 188 aligned\n");
        }
        else
        {
            // fprintf(stderr, "Rx ts %d\n",length);
        }
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
                header->matype1 = 0xC0; // TS 0.35roff

                header->matype2 = 0;                        // Input Stream Identifier
                header->upl = htons(188 * 8);               // User Packet Length 188
                header->dfl = htons((framebytes - 10) * 8); // Data Field Length
                header->sync = 0x47;                        // SYNC - Copy of the user packet Sync byte
                header->syncd1 = (remain * 8) >> 8;         // SYNCD
                header->syncd2 = (remain * 8) & 0xFF;       // SYNCD
                header->crc = calc_crc8_r(bbframe, 9);      // CRC

                // udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes);
                addbbframe(bbframe, framebytes);

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
                header->matype1 = 0xC0; // TS 0.35roff

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
                addbbframe(bbframe, framebytes);
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
}
#endif

void *rx_ts_thread(void *arg)
{
    unsigned char tspacket[7 * 188];

    int length = 0;
    while (true)
    {
        length = udp_receive(recv_ts_sock, tspacket, 7 * 188);
        pthread_mutex_lock(&buffer_mutexts);
        #ifdef  WITH_NEON
        addneonts(tspacket, length);
        #else
         addts(tspacket, length);
        #endif
       
       pthread_mutex_unlock(&buffer_mutexts);
    }
}

void setneonmodcod(uint Constellation, uint CodeRate, uint FrameType)
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
    fmt.output_format = OUTPUT_FORMAT_SYMBOLS; // OUTPUT_FORMAT_BBFRAME is segfault
    fmt.pilots = PILOTS_OFF;
    fmt.roll_off = RO_0_35;

    // fprintf(stderr, "Trying to set dvbs2neon \n");
    //pthread_mutex_lock(&buffer_mutexts);
    // FixMe : Modcod should be changed ONLY when a bbframe is complete or at startup
    int status = dvbs2neon_control(STREAM0, CONTROL_SET_PARAMETERS, (uint32)&fmt, 0);
    modulator_mapping(fmt.constellation, CodeRate);
  
  
        
    //pthread_mutex_unlock(&buffer_mutexts);
}

void settsmodcode()
{
}

void setpaddingts()
{
    static unsigned char NullPacket[7 * 188] = {0x47, 0x1F, 0xFE, 0x10, 'F', '5', 'O', 'E', 'O'};
    static unsigned char cc = 0;
    for (int k = 0; k < 7; k++)
    {
        memcpy(NullPacket + k * 188, NullPacket, 9);
        NullPacket[3 + k * 188] = 0x10 + cc;
        cc = (cc + 1) % 16;
        for (int i = 9; i < 188; i++)
            NullPacket[i + k * 188] = i;
    }
    pthread_mutex_lock(&buffer_mutexts);

    #ifdef  WITH_NEON
    addneonts(NullPacket, 7 * 188);
    #else
    addts(NullPacket, 7 * 188);
    #endif
    pthread_mutex_unlock(&buffer_mutexts);
}

static pthread_t p_rxts;
void init_mux(char *mcast_ts, char *mcast_iface)
{
    int status1 = dvbs2neon_control(0, CONTROL_RESET_FULL, (uint32)symbolbuff, sizeof(symbolbuff));
    int status2 = dvbs2neon_control(STREAM0, CONTROL_RESET_STREAM, 0, 0);
    recv_ts_sock = udp_init(mcast_ts, NULL /*mcast_iface*/, 1);
    build_crc8_table_r();
    pthread_mutex_init(&buffer_mutexts, NULL);
    pthread_create(&(p_rxts), NULL, &rx_ts_thread, NULL);
}