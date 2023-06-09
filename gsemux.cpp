/*
  ===========================================================================

  Copyright (C) 2022 Evariste F5OEO


  GSE_MUX is free software: you can redistribute it and/or modify
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

//

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

#include <jansson.h>

#include <sys/types.h>
#include <sys/select.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h> /* IPPROTO_IP, sockaddr_in, htons(),
htonl() */
#include <arpa/inet.h>  /* inet_addr() */
#include <netdb.h>
#include <time.h>
#include <sys/ioctl.h>
// TUN/tap
#include <net/if.h> /* for IFNAMSIZ */
#include <linux/if_tun.h>
#include <queue>
/* GSE includes */
extern "C"
{
#include "constants.h"
#include "encap.h"
#include "deencap.h"
#include "refrag.h"
}
using namespace std;

/* DEBUG macro */
#define DEBUG(is_debug, out, format, ...)        \
    do                                           \
    {                                            \
        if (is_debug)                            \
            fprintf(out, format, ##__VA_ARGS__); \
    } while (0)

// ************************** External -> plutostream

extern unsigned int BBFrameLenLut[];
extern uint8_t calc_crc8_r(uint8_t *b, int len);
extern void build_crc8_table_r();
extern int m_ModeCod;
extern u_int16_t udp_init(char *ip, const char *iface, int rx); // interface to multicast from
size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen);
extern int m_Fecmode;
extern int m_txmode;
extern unsigned char getdvbs2modcod(uint FrameType, uint Constellation, uint CodeRate, uint Pilots);

enum
{
    fec_fix,
    fec_variable
};

#define UDP_BUFF_MAX_BBFRAME 8000
typedef struct
{
    ssize_t size;
    ssize_t modecod;
    uint8_t bbframe[UDP_BUFF_MAX_BBFRAME];
} buffer_t;

extern queue<buffer_t *> m_bbframe_queue;
extern pthread_mutex_t buffer_mutextx;
//******************************End external

uint8_t m_gsemodcod = 0;

uint m_gseconstellation = 0;
uint m_gsecoderate = 0;
uint m_gseframetype = 0;
uint m_gsepilots = 0;

uint8_t m_variable_gse_coderate = 0;

size_t m_framelen = 0;
char tun_name[] = "gse0";
int is_debug = 0;
int tun;
long m_tun_read_timeout = 100000; // 10ms
uint m_gsesr = 1000000;
char m_mcast_rxgse[255];
char m_mcast_rxiface[255];

uint32_t m_MaxBBFrameByte = 0;
uint32_t m_UsedBBFrameByte = 0;

#define MAX_BBFRAME (58192 / 8)

/* The maximal size of data that can be received on the virtual interface */
#define TUNTAP_BUFSIZE 1518

/* The maximal size of a GSE packet */
#define MAX_GSE_SIZE 4096

/* GSE parameters */
#define QOS_NBR 255
#define FIFO_SIZE 4 // Max GSE packet for a one PDU
#define BBHEADER_LENGTH 10

char sInterfaceToPluto[255];

inline size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen)
{

    size_t rcvlen = 0;
    rcvlen = recv(sock, b, maxlen, 0 /* MSG_ZEROCOPY*/);
    return rcvlen;
}

int tun_create(char *name, const char *ip)
{
    struct ifreq ifr;
    int fd, err;

    /* open a file descriptor on the kernel interface */
    char stunpath[255];
    // sprintf(stunpath,"/dev/net/%s",name);
    sprintf(stunpath, "/dev/net/tun");
    if ((fd = open(stunpath, O_RDWR)) < 0)
    {
        fprintf(stderr, "Tun error\n");
        return fd;
    }
    /* flags: IFF_TUN   - TUN device (no Ethernet headers)
     *        IFF_TAP   - TAP device
     *        IFF_NO_PI - Do not provide packet information */
    bzero(&ifr, sizeof(ifr));
    strncpy(ifr.ifr_name, name, IFNAMSIZ);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_flags = IFF_TUN;
    //|IFF_UP
    /* create the TUN interface */
    if ((err = ioctl(fd, TUNSETIFF, (void *)&ifr)) < 0)
    {
        close(fd);
        return err;
    }

    /* increase txqueuelen from default 500 */
    // interface_ioctl( SIOCSIFTXQLEN, name,[] ( ifreq &ifr ) { ifr.ifr_qlen = 1000; } );

    int sockfd;
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    // Read interface flags
    if (ioctl(sockfd, SIOCGIFFLAGS, &ifr) < 0)
    {
        fprintf(stderr, "ifdown: shutdown ");
    }
    ifr.ifr_flags |= IFF_UP;
    if ((err = ioctl(sockfd, SIOCSIFFLAGS, (void *)&ifr)) < 0)
    {
        close(fd);
        return err;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    socket(addr.sin_family, SOCK_DGRAM, 0);
    inet_pton(addr.sin_family, ip, &addr.sin_addr);
    ifr.ifr_addr = *(struct sockaddr *)&addr;
    ioctl(sockfd, SIOCSIFADDR, &ifr);

    return fd;
}

// Write IP extracted from Longmynd bbframe
int write_to_tun(int fd, gse_vfrag_t *vfrag)
{
    int ret;

    ret = write(fd, gse_get_vfrag_start(vfrag), gse_get_vfrag_length(vfrag));
    if (ret < 0)
    {
        fprintf(stderr, "write failed: %s (%d)\n", strerror(errno), errno);
        goto error;
    }

    DEBUG(is_debug, stderr, "%u bytes written on fd %d\n", ret, fd);

    return 0;
error:
    return 1;
}

/**
 * @brief Read data from the TUN interface to be encapsulated in bbframe
 *
 * Data read by this function contains a 4-byte header that gives the protocol
 * of the data.
 *
 *   +-----+-----+-----+-----+
 *   |  0  |  0  |  Protocol |
 *   +-----+-----+-----+-----+
 *
 * Protocol = 0x0800 for IPv4
 *            0x86dd for IPv6
 *
 * @param fd         The TUN file descriptor to read data from
 * @param vfrag      The virtual fragment where to store the data
 * @return           0 in case of success, a non-null value otherwise
 */

int read_from_tun(int fd, uint8_t *ip_pdu, long timeout) // timeout in us
{
    int ret;
    int read_length;
    struct timespec tv;
    fd_set readfds;
    sigset_t sigmask;
    sigaddset(&sigmask, SIGTERM);
    sigaddset(&sigmask, SIGINT);
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    tv.tv_sec = timeout / 1000000L;
    timeout = timeout % 1000000L;
    tv.tv_nsec = timeout * 1000L;

    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    ret = pselect(fd + 1, &readfds, NULL, NULL, timeout > 0 ? &tv : NULL, &sigmask);
    if (ret < 0)
    {
        fprintf(stderr, "Error on UDP select: %s", strerror(errno));
        return -1;
    }
    /* timeout */
    else if (ret == 0)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);

        size_t diff_us = (now.tv_sec - start.tv_sec) * 1000000L;
        diff_us += (now.tv_nsec - start.tv_nsec) / 1000L;
        // fprintf(stderr,"Diff %u \n",diff_us);
        // fprintf(stderr, "Timeout: %d\n",diff_us );
        return -1;
    }
    /* There is data */
    else
    {
        read_length = read(fd, ip_pdu, GSE_MAX_PDU_LENGTH);

        if (read_length < 0)
        {
            fprintf(stderr, "read failed: %s (%d)\n", strerror(errno), errno);
            return -1;
        }
        // fprintf(stderr, "Get tun %d\n", read_length);
        return (read_length);
    }
}

inline void udp_send(u_int16_t sock, char *ip, unsigned char *b, int len)
{

    u_int16_t port;
    struct sockaddr_in m_client;
    char text[40];
    char *add[2];
    strcpy(text, ip);
    add[0] = strtok(text, ":");
    add[1] = strtok(NULL, ":");
    if (strlen(add[1]) == 0)
        port = 1314;
    else
        port = atoi(add[1]);
    // Construct the client sockaddr_in structure
    memset(&m_client, 0, sizeof(m_client));       // Clear struct
    m_client.sin_family = AF_INET;                // Internet/IP
    m_client.sin_addr.s_addr = inet_addr(add[0]); // IP address
    m_client.sin_port = htons(port);              // server socket

    int sent = sendto(sock, b, len, 0 /*MSG_ZEROCOPY*/, (struct sockaddr *)&m_client, sizeof(m_client));
}
enum
{
    tx_passtrough,
    tx_iq,
    tx_dvbs2_ts,
    tx_dvbs2_gse

};

// Receive BBFrame from Longmynd and defrag to tun
u_int16_t m_recv_bbframe_sock = 0;
void *rx_bbframe_thread(void *arg)
{

    size_t read_length;
    // char recv_bbframe_ip[] = "230.0.0.1:1234";
    // recv_bbframe_sock = udp_init(recv_bbframe_ip, "192.168.1.104", 1);
    m_recv_bbframe_sock = udp_init(m_mcast_rxgse, m_mcast_rxiface, 1);
    gse_deencap_t *deencap = NULL;
    gse_deencap_init(QOS_NBR, &deencap);
    gse_deencap_set_offsets(deencap, 4, 0);
    gse_vfrag_t *vfrag_pkt = NULL;
    gse_vfrag_t *pdu = NULL;
    uint8_t label_type;
    uint8_t label[6];
    uint16_t protocol;
    uint16_t gse_length;
    unsigned int local_pdu;
    static unsigned char BBframe[58192 / 8];
    int ret;

    while (true)
    {
        if (m_txmode != tx_dvbs2_gse)
        {
            usleep(1000);
            continue;
        }

        int length = 0;
        int index = 10;
        length = udp_receive(m_recv_bbframe_sock, BBframe, MAX_BBFRAME);
        // fprintf(stderr,"udep rcv %d on socket %d\n",length,m_recv_bbframe_sock);

        if (length < 12)
            continue;
        index = 10;
        while (index < length - 10)
        {
            ret = gse_create_vfrag(&vfrag_pkt, GSE_MAX_PACKET_LENGTH + 4, 0, 0);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error when creating reception fragment: %s\n",
                        gse_get_status((gse_status_t)ret));
            }
            int fraglen = (((int)BBframe[index] & 0xF) << 8) + (int)(BBframe[index + 1]) + 2;
            // fprintf(stderr,"Frag len %d Index %d\n",fraglen,index);
            ret = gse_copy_data(vfrag_pkt, BBframe + index, fraglen);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error copy %s\n", gse_get_status((gse_status_t)ret));
            }
            ret = gse_deencap_packet(vfrag_pkt, deencap, &label_type, label, &protocol,
                                     &pdu, &gse_length);
            if ((ret > GSE_STATUS_OK) && (ret != GSE_STATUS_PDU_RECEIVED))
            {
                fprintf(stderr, "Error when de-encapsulating GSE packet : %s\n",
                        gse_get_status((gse_status_t)ret));
            }
            if (ret == GSE_STATUS_PDU_RECEIVED)
            {

                ret = gse_shift_vfrag(pdu, -4, 0);
                if (ret > GSE_STATUS_OK)
                {
                    fprintf(stderr, "Error when shifting PDU #%u: %s\n",
                            local_pdu, gse_get_status((gse_status_t)ret));
                }
                /* build the TUN header */
                gse_get_vfrag_start(pdu)[0] = 0;
                gse_get_vfrag_start(pdu)[1] = 0;
                protocol = htons(protocol);
                memcpy(&gse_get_vfrag_start(pdu)[2], &protocol, 2);

                /* write the IP packet on the virtual interface */
                ret = write_to_tun(tun, pdu);
                if (ret != GSE_STATUS_OK)
                {
                    fprintf(stderr, "write_to_tun failed\n");
                    // goto free_pdu;
                }

                gse_free_vfrag(&pdu);
            }
            index += fraglen;
        }
    }
}

#define MAX_BBFRAME (58192 / 8)

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

/*
unsigned int BBFrameLenLut[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0,
                                16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};
*/

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

#define MAX_QUEUE_ITEM 200

bool addgsebbframe(uint8_t *bbframe, size_t len, size_t modcod)
{
    if (m_txmode != tx_dvbs2_gse)
    {

        return false;
    }
    pthread_mutex_lock(&buffer_mutextx);
    buffer_t *newbuf = (buffer_t *)malloc(sizeof(buffer_t));
    newbuf->size = len;
    newbuf->modecod = modcod;
    memcpy(newbuf->bbframe, bbframe, len);

    if ((m_bbframe_queue.size() >= MAX_QUEUE_ITEM))
    {
        fprintf(stderr, "MUXGSE : Queue is full ! Purging %d bbframe\n", m_bbframe_queue.size());

        while (m_bbframe_queue.size() > 1)
        {
            buffer_t *oldestbuf = m_bbframe_queue.front(); // Remove the oldest

            free(oldestbuf);

            m_bbframe_queue.pop();
        }
    }

    m_bbframe_queue.push(newbuf);
    pthread_mutex_unlock(&buffer_mutextx);
    return true;
}

void setpaddinggse()
{

    static uint8_t BBFrameNull[MAX_BBFRAME];

    struct bbheader *header = (struct bbheader *)BBFrameNull;
    header->matype1 = 0x72; // TS 0.35roff

    header->matype2 = 0;                       // Input Stream Identifier
    header->upl = htons(0 * 8);                // User Packet Length 188
    header->dfl = htons(0 * 8);                // Data Field Length
    header->sync = 0;                          // SYNC - Copy of the user packet Sync byte
    header->syncd1 = 0;                        // SYNCD
    header->syncd2 = 0;                        // SYNCD
    header->crc = calc_crc8_r(BBFrameNull, 9); // CRC

    // size_t len = BBFrameLenLut[m_gsemodcod]/8;
    // uint16_t framebytes = BBFrameLenLut[(m_gseframetype == 0 ? 11 : 0) + m_gsecoderate] / 8;
    // m_gsemodcod = getdvbs2modcod(m_gseframetype, m_gseconstellation, m_gsecoderate, m_gsepilots);

    uint16_t framebytes = BBFrameLenLut[0] / 8; // ShortFrame 1/4
    uint8_t gsemodcod = getdvbs2modcod(1 /*short*/, m_gseconstellation, 0, m_gsepilots);
    //fprintf(stderr, "gse padding\n");
    BBFrameNull[10] = 0; // gse padding
    m_MaxBBFrameByte += framebytes;
    m_UsedBBFrameByte += 0;

    addgsebbframe(BBFrameNull, framebytes, gsemodcod);
}

u_int16_t send_bbframe_sock;
char send_bbframe_ip[] = "230.0.0.2:1234";
// Receive IP from tun and frag to udp to be processed by pluto_stream
// https://github.com/HAMNET-Access-Protocol/HNAP4PlutoSDR/pull/59
void *rx_tun_thread(void *arg)
{
    float feceffiency[] = {1 / 4.0, 1 / 3.0, 2 / 5.0, 1 / 2.0, 3 / 5.0, 2 / 3.0, 3 / 4.0, 4 / 5.0, 5 / 6.0, 8 / 9.0, 9 / 10.0};

    gse_encap_t *encap = NULL;
    gse_encap_init(4, FIFO_SIZE, &encap);

    // build_crc8_table();
    gse_status_t ret = 0;
    size_t tempcoderate = 0;
    while (true)
    {
        // int len = read_from_tun(tun, ippacket);
        if (m_txmode != tx_dvbs2_gse)
        {
            usleep(1000);
            continue;
        }
        gse_vfrag_t *vfrag_pdu = NULL;

        uint16_t framebytes = BBFrameLenLut[((m_gseframetype == 0 ? 11 : 0) + m_gsecoderate + tempcoderate) % 22] / 8;
        m_gsemodcod = getdvbs2modcod(m_gseframetype, m_gseconstellation, (m_gsecoderate + tempcoderate) % 11, m_gsepilots);

        m_tun_read_timeout = (framebytes * 8 * 1000L) / (m_gsesr / 4000 * (m_gseconstellation + 2) * feceffiency[(m_gsecoderate + tempcoderate) % 11]);
        m_tun_read_timeout += 5000; // Margin of 5 ms
        // fprintf(stderr,"SR %d Framebit %d Eff %f Timeout %ld\n",m_gsesr/4,(framebytes * 8 ),feceffiency[(m_gsecoderate+tempcoderate)%11],m_tun_read_timeout);
        //  uint16_t framebytes = m_framelen;
        unsigned char ippacket[GSE_MAX_PDU_LENGTH];
        unsigned char bbframe[MAX_BBFRAME];
        unsigned char *data = bbframe + 10; // BBHeader
        uint8_t *end = bbframe + framebytes;
        struct bbheader *header = (struct bbheader *)bbframe;

        uint16_t avail = framebytes - 10;

        while (avail)
        {
            gse_vfrag_t *gse_pkt_vfrag;

            ret = gse_encap_get_packet(&gse_pkt_vfrag, encap, avail < GSE_MAX_PACKET_LENGTH ? avail : GSE_MAX_PACKET_LENGTH, 0);
            if (ret == GSE_STATUS_OK)
            {
                size_t gse_pkt_size = gse_get_vfrag_length(gse_pkt_vfrag);

                 //fprintf(stderr, "GSE: Got a packet with %lu bytes from previous run\n", gse_pkt_size);

                memcpy(data, gse_get_vfrag_start(gse_pkt_vfrag), gse_pkt_size);

                data += gse_pkt_size;
                avail -= gse_pkt_size;

                gse_free_vfrag(&gse_pkt_vfrag);
            }
            else if (ret == GSE_STATUS_FIFO_EMPTY)
            {
                break;
            }
            else if (ret == GSE_STATUS_LENGTH_TOO_SMALL)
            {
                fprintf(stderr, "GSE: BBFRAME only has %hu bytes available, cannot fit a GSE packet in there\n", avail);

                break;
            }
            else
            {
                fprintf(stderr, "Error when getting GSE packet: %s\n", gse_get_status(ret));
            }
        }

        int len = 0;
        // Feed in more data if we already drained the FIFO, otherwise skip

        while ((avail>GSE_MAX_HEADER_LENGTH) && ret != GSE_STATUS_LENGTH_TOO_SMALL && ((len = read_from_tun(tun, ippacket, m_tun_read_timeout)) > 0))
        {

            uint8_t *pdu = ippacket + 4; // Tunnel header size
            ssize_t pdu_size = len - 4;  // Tunnel header size
            uint16_t *protocol = (uint16_t *)&ippacket[2];
            gse_vfrag_t *pdu_vfrag;

            ret = gse_create_vfrag_with_data(&pdu_vfrag, pdu_size, GSE_MAX_HEADER_LENGTH, GSE_MAX_TRAILER_LENGTH, pdu, pdu_size);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error when creating PDU virtual fragment: %s\n", gse_get_status(ret));
            }

            ret = gse_encap_receive_pdu(pdu_vfrag, encap, NULL, GSE_LT_NO_LABEL, ntohs(*protocol), 0);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error when encapsulating PDU: %s\n", gse_get_status(ret));
            }

            // fprintf(stderr, "GSE: Ingested a PDU with %lu bytes. Protocol: 0x%04X\n", pdu_size, ntohs(*protocol));

            // in_ip.read(1);

            while (avail >GSE_MAX_HEADER_LENGTH)
            {
                gse_vfrag_t *gse_pkt_vfrag;

                ret = gse_encap_get_packet(&gse_pkt_vfrag, encap, avail < GSE_MAX_PACKET_LENGTH ? avail : GSE_MAX_PACKET_LENGTH, 0);
                if (ret == GSE_STATUS_OK)
                {
                    size_t gse_pkt_size = gse_get_vfrag_length(gse_pkt_vfrag);

                    // fprintf(stderr, "GSE: Got a packet with %lu bytes\n", gse_pkt_size);

                    memcpy(data, gse_get_vfrag_start(gse_pkt_vfrag), gse_pkt_size);

                    data += gse_pkt_size;
                    avail -= gse_pkt_size;

                    gse_free_vfrag(&gse_pkt_vfrag);
                }
                else if (ret == GSE_STATUS_FIFO_EMPTY)
                {
                    // fprintf(stderr, "GSE: Drained encapsulator FIFO, going to check for more PDUs\n");

                    break;
                }
                else if (ret == GSE_STATUS_LENGTH_TOO_SMALL)
                {
                    fprintf(stderr, "GSE: BBFRAME only has %hu bytes available, cannot fit a GSE packet in there\n", avail);

                    break;
                }
                else
                {
                    fprintf(stderr, "Error when getting GSE packet: %s\n", gse_get_status(ret));
                }
            }
        }
        if (len <= 0)
        {
            // fprintf(stderr, "timeout\n");
        }

        if (!avail) // BBFRAME was fully filled up with data
        {
            // fprintf(stderr, "GSE: BBFRAME fully filled\n");
        }
        else if (ret == GSE_STATUS_LENGTH_TOO_SMALL) // BBFRAME was filled to the max (i.e. there is some space left but we cannot put anything useful in there)
        {
            fprintf(stderr, "GSE: BBFRAME filled (best-effort). Remaining %hu bytes\n", avail);
        }
        else if (avail != framebytes - 10) // BBFRAME has space that could be used but was not due to lack of data
        {
            // fprintf(stderr, "GSE: BBFRAME space wasted. Remaining %hu bytes\n", avail);
        }

        data += avail;          // Required to pass the check below (if(data != end))
                                // EN 302 307-1 section 5.1.6 Base-Band Header insertion
        header->matype1 = 0x72; // Generic continuous 0.20roff

        header->matype2 = 0;                                // Input Stream Identifier
        header->upl = htons(0 * 8);                         // User Packet Length (0 for Generic continuous)
        header->dfl = htons((framebytes - 10 - avail) * 8); // Data Field Length
        header->sync = 0x00;                                // SYNC - Copy of the user packet Sync byte
        header->syncd1 = 0x00;                              // SYNCD
        header->syncd2 = 0x00;                              // SYNCD
        header->crc = calc_crc8_r(bbframe, 9);              // CRC

        // fprintf(stderr, "dfl %d \n",(framebytes - 10 - avail) );

        if (data != end)
        {
            fprintf(stderr, "GSE: BBFRAME END ISSUE\n");
            break;
        }
        if (framebytes - avail > 10)
        {
            // fprintf(stderr, "send udp %d \n",framebytes-avail);
            // for (int i = 0; i < 10; i++) fprintf(stderr, "%x ", bbframe[i]);
            // fprintf(stderr, " -> CRC %x\n",calc_crc8(bbframe, 9));
            // m_gsemodcod=m_gseframetype + m_gseconstellation + m_gsecoderate+tempcoderate;

            // m_gsemodcod=getdvbs2modcod(m_gseframetype,m_gseconstellation,(m_gsecoderate+tempcoderate)%11,m_gsepilots);
            m_MaxBBFrameByte += framebytes;
            m_UsedBBFrameByte += framebytes - avail;

            addgsebbframe(bbframe, framebytes /*framebytes - avail*/, m_gsemodcod);

            // fprintf(stderr, "BBframe efficiency %d \n", ((framebytes - avail) * 100) / framebytes);
            if (m_Fecmode == fec_variable)
            {

                tempcoderate = (m_bbframe_queue.size() / 2);
                if (m_gsecoderate + tempcoderate > 9)
                    tempcoderate = 9 - m_gsecoderate;

                // fprintf(stderr, "gse variable : tempcoderate %d coderate = %d\n", tempcoderate, m_gsecoderate + tempcoderate);
            }
            else
            {
                tempcoderate = 0;
            }
            m_variable_gse_coderate = m_gsecoderate + tempcoderate;
        }
        else
        {
            /*
            if(len<=0)
            {
                setpaddinggse();
            }
            */
        }
    }
}

void setgsemodcod(uint Constellation, uint CodeRate, uint FrameType, uint Pilots)
{
    m_gseconstellation = Constellation;
    m_gsecoderate = CodeRate;
    m_gseframetype = FrameType;
    m_gsepilots = Pilots;
}

void setgsesr(uint sr)
{
    m_gsesr = sr;
}

void setbbframemcast(char *mcast_rx)
{
    m_recv_bbframe_sock = udp_init(mcast_rx, m_mcast_rxiface, 1);
    fprintf(stderr, "Receive longmynd on %s interface %s sock=%d\n", mcast_rx, m_mcast_rxiface, m_recv_bbframe_sock);
}

static pthread_t p_rxbbframe;
static pthread_t p_rxtun;

void init_gsemux(char *mcast_rxgse, char *mcast_rxiface, char *tunip, long tun_read_timeout)
{

    // build_crc8_table();
    build_crc8_table_r();
    tun = tun_create(tun_name, tunip);
    if (tun < 0)
    {
        fprintf(stderr, "%s creation failed (try sudo?)\n", tun_name);
        exit(0);
    }

    strcpy(m_mcast_rxgse, mcast_rxgse);
    strcpy(m_mcast_rxiface, mcast_rxiface);
    m_tun_read_timeout = tun_read_timeout;
    pthread_create(&(p_rxbbframe), NULL, &rx_bbframe_thread, NULL);
    pthread_create(&(p_rxtun), NULL, &rx_tun_thread, NULL);
}

void end_gsemux()
{
    pthread_cancel(p_rxbbframe);
    pthread_cancel(p_rxtun);
}
