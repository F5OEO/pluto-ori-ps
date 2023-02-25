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

/* GSE includes */
#include "constants.h"
#include "encap.h"
#include "deencap.h"
#include "refrag.h"

#define PROGRAM_VERSION "0.0.1"
static int want_quit = 0;
char tun_name[] = "gse0";
int is_debug = 1;
int tun;
long tun_read_timeout = 10000; // 10ms

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

void print_usage()
{

    fprintf(stderr,
            "bbgse -%s [-i ip] [-o iface to pluto]\n\
Usage:\bbgse \n\
\n",
            PROGRAM_VERSION);

} /* end function print_usage */

/* DEBUG macro */
#define DEBUG(is_debug, out, format, ...)        \
    do                                           \
    {                                            \
        if (is_debug)                            \
            fprintf(out, format, ##__VA_ARGS__); \
    } while (0)

int tun_create(char *name, const char *ip)
{
    struct ifreq ifr;
    int fd, err;

    /* open a file descriptor on the kernel interface */
    if ((fd = open("/dev/net/tun", O_RDWR)) < 0)
        return fd;

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

    ret = pselect(fd + 1, &readfds, NULL, NULL, timeout > 0 ? &tv : NULL, &sigmask);
    if (ret < 0)
    {
        fprintf(stderr, "Error on UDP select: %s", strerror(errno));
        return -1;
    }
    /* timeout */
    else if (ret == 0)
    {
        // fprintf(stderr, "Timeout: %s", strerror(errno));
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

inline size_t udp_receive(u_int16_t sock, unsigned char *b, unsigned int maxlen)
{

    size_t rcvlen = recv(sock, b, maxlen, 0 /* MSG_ZEROCOPY*/);
    return rcvlen;
}

// Global signal handler for trapping SIGINT, SIGTERM, and SIGQUIT
static void signal_handler(int signal)
{

    want_quit = 1;
}

// Receive BBFrame from Longmynd and defrag to tun
void *rx_bbframe_thread(void *arg)
{
    u_int16_t recv_bbframe_sock;
    size_t read_length;
    char recv_bbframe_ip[] = "230.0.0.1:1234";
    recv_bbframe_sock = udp_init(recv_bbframe_ip, "192.168.1.104", 1);

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

    while (want_quit == 0)
    {
        int length = 0;
        int index = 10;
        length = udp_receive(recv_bbframe_sock, BBframe, MAX_BBFRAME);
        // fprintf(stderr,"udep rcv %d\n",length);
        if (length < 12)
            break;
        index = 10;
        while (index < length - 10)
        {
            ret = gse_create_vfrag(&vfrag_pkt, GSE_MAX_PACKET_LENGTH + 4, 0, 0);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error when creating reception fragment: %s\n",
                        gse_get_status(ret));
            }
            int fraglen = (((int)BBframe[index] & 0xF) << 8) + (int)(BBframe[index + 1]) + 2;
            // fprintf(stderr,"Frag len %d Index %d\n",fraglen,index);
            ret = gse_copy_data(vfrag_pkt, BBframe + index, fraglen);
            if (ret > GSE_STATUS_OK)
            {
                fprintf(stderr, "Error copy %s\n", gse_get_status(ret));
            }
            ret = gse_deencap_packet(vfrag_pkt, deencap, &label_type, label, &protocol,
                                     &pdu, &gse_length);
            if ((ret > GSE_STATUS_OK) && (ret != GSE_STATUS_PDU_RECEIVED))
            {
                fprintf(stderr, "Error when de-encapsulating GSE packet : %s\n",
                        gse_get_status(ret));
            }
            if (ret == GSE_STATUS_PDU_RECEIVED)
            {

                ret = gse_shift_vfrag(pdu, -4, 0);
                if (ret > GSE_STATUS_OK)
                {
                    fprintf(stderr, "Error when shifting PDU #%u: %s\n",
                            local_pdu, gse_get_status(ret));
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

#define CRC_POLY 0xAB
// Reversed
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
                crc = (crc << 1) ^ CRC_POLY;
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

unsigned int BBFrameLenLut[] = {3072, 5232, 6312, 7032, 9552, 10632, 11712, 12432, 13152, 14232, 0,
                                16008, 21408, 25728, 32208, 38688, 43040, 48408, 51648, 53840, 57472, 58192};

int m_ModeCod = C2_3 + 11;

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

u_int16_t send_bbframe_sock;
char send_bbframe_ip[] = "230.0.0.2:1234";
// Receive IP from tun and frag to udp to be processed by pluto_stream
// https://github.com/HAMNET-Access-Protocol/HNAP4PlutoSDR/pull/59
void *rx_tun_thread(void *arg)
{

    fprintf(stderr, "Sent gse to %s\n", send_bbframe_ip);
    gse_encap_t *encap = NULL;
    gse_encap_init(4, FIFO_SIZE, &encap);

    // build_crc8_table();
    gse_status_t ret = 0;
    while (want_quit == 0)
    {
        // int len = read_from_tun(tun, ippacket);
        gse_vfrag_t *vfrag_pdu = NULL;
        uint16_t framebytes = framebytes = BBFrameLenLut[m_ModeCod] / 8; // Need to set it by modcod
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

                // fprintf(stderr, "GSE: Got a packet with %lu bytes from previous run\n", gse_pkt_size);

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

        while (avail && ret != GSE_STATUS_LENGTH_TOO_SMALL && ((len = read_from_tun(tun, ippacket, tun_read_timeout)) > 0))
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

            while (avail)
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
            fprintf(stderr, "GSE: BBFRAME space wasted. Remaining %hu bytes\n", avail);
        }

        data += avail;          // Required to pass the check below (if(data != end))
                                // EN 302 307-1 section 5.1.6 Base-Band Header insertion
        header->matype1 = 0x70; // Generic continuous 0.35roff

        header->matype2 = 0;                                // Input Stream Identifier
        header->upl = htons(0 * 8);                         // User Packet Length (0 for Generic continuous)
        header->dfl = htons((framebytes - 10 - avail) * 8); // Data Field Length
        header->sync = 0x00;                                // SYNC - Copy of the user packet Sync byte
        header->syncd1 = 0x00;                              // SYNCD
        header->syncd2 = 0x00;                              // SYNCD
        header->crc = calc_crc8_r(bbframe, 9);              // CRC

        if (data != end)
            fprintf(stderr, "GSE: BBFRAME END ISSUE\n");

        if (framebytes - avail > 10)
        {
            // fprintf(stderr, "send udp %d \n",framebytes-avail);
            // for (int i = 0; i < 10; i++) fprintf(stderr, "%x ", bbframe[i]);
            // fprintf(stderr, " -> CRC %x\n",calc_crc8(bbframe, 9));
            udp_send(send_bbframe_sock, send_bbframe_ip, bbframe, framebytes - avail);
        }
        else
        {
            // fprintf(stderr, "Lost %d \n",framebytes-avail);
        }
    }
}


int main(int argc, char **argv)
{
    int ret;
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGPIPE, signal_handler);

    int a;
    int anyargs = 0;
    char sTunInterface[255];
    while (1)
    {
        a = getopt(argc, argv, "i:o:t:m:");

        if (a == -1)
        {
            if (anyargs)
                break;
            else
                a = 'h'; // print usage and exit
        }
        anyargs = 1;

        switch (a)
        {
        case 'i': // ip
            strcpy(sTunInterface, optarg);

            break;
        case 'o': // output interface to pluto
            strcpy(sInterfaceToPluto, optarg);
        case 't': // tun timeout
            tun_read_timeout = atol(optarg);
            break;
        case 'm': // modcod
            m_ModeCod = atoi(optarg);
            break;

        case 'h': // help
            print_usage();
            exit(0);
            break;
        }
    }
    fprintf(stderr, "Tun %s Send 230.0.0.2 to pluto through interface  %s\n", sTunInterface, sInterfaceToPluto);

    tun = tun_create(tun_name, sTunInterface);
    if (tun < 0)
    {
        fprintf(stderr, "%s creation failed (try sudo?)\n", tun_name);
        exit(0);
    }

    static pthread_t p_rxbbframe;
    static pthread_t p_rxtun;
    static pthread_t p_rxts;
    build_crc8_table();
    build_crc8_table_r();
    send_bbframe_sock = udp_init(send_bbframe_ip, sInterfaceToPluto, 0);
    pthread_create(&(p_rxbbframe), NULL, &rx_bbframe_thread, NULL);
    pthread_create(&(p_rxtun), NULL, &rx_tun_thread, NULL);
   
    while (want_quit == 0)
    {
        usleep(1000);
    }
}
