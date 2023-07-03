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
#include <mutex>
#include <unordered_set>
#include <CivetServer.h>
#include <NE10.h>

using namespace std;

/* DEBUG macro */
#define DEBUG(is_debug, out, format, ...)        \
    do                                           \
    {                                            \
        if (is_debug)                            \
            fprintf(out, format, ##__VA_ARGS__); \
    } while (0)

// ************************** External -> plutostream

// ================= CIVETWEB ================================

class WsStartHandler : public CivetHandler
{
public:
    bool
    handleGet(CivetServer *server, struct mg_connection *conn)
    {

        mg_printf(conn,
                  "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: "
                  "close\r\n\r\n");

        mg_printf(conn, "<!DOCTYPE html>\n");
        mg_printf(conn, "<html>\n<head>\n");
        mg_printf(conn, "<meta charset=\"UTF-8\">\n");
        mg_printf(conn, "<title>Embedded websocket example</title>\n");

        /* mg_printf(conn, "<script type=\"text/javascript\"><![CDATA[\n"); ...
         * xhtml style */
        mg_printf(conn, "<script>\n");
        mg_printf(
            conn,
            "var i=0\n"
            "function load() {\n"
            "  var wsproto = (location.protocol === 'https:') ? 'wss:' : 'ws:';\n"
            "  connection = new WebSocket(wsproto + '//' + window.location.host + "
            "'/websocket');\n"
            "  websock_text_field = "
            "document.getElementById('websock_text_field');\n"
            "  connection.onmessage = function (e) {\n"
            "    websock_text_field.innerHTML=e.data;\n"
            "    i=i+1;"
            "    connection.send(i);\n"
            "  }\n"
            "  connection.onerror = function (error) {\n"
            "    alert('WebSocket error');\n"
            "    connection.close();\n"
            "  }\n"
            "}\n");
        /* mg_printf(conn, "]]></script>\n"); ... xhtml style */
        mg_printf(conn, "</script>\n");
        mg_printf(conn, "</head>\n<body onload=\"load()\">\n");
        mg_printf(
            conn,
            "<div id='websock_text_field'>No websocket connection yet</div>\n");
        mg_printf(conn, "</body>\n</html>\n");

        return 1;
    }
};


void extract_between_quotes(char *s, char *dest)
{
    int in_quotes = 0;
    *dest = 0;
    while (*s != 0)
    {
        if (in_quotes)
        {
            if (*s == '"')
                return;
            dest[0] = *s;
            dest[1] = 0;
            dest++;
        }
        else if (*s == '"')
            in_quotes = 1;
        s++;
    }
}

class WebSocketHandler : public CivetWebSocketHandler
{

private:
    /// Lock protecting \c connections_.
    std::mutex connectionsLock_;

    /// Set of connected clients to the entry point.
    std::unordered_set<mg_connection *> connections_;

public:
    virtual bool handleConnection(CivetServer *server,
                                  const struct mg_connection *conn)
    {
         fprintf(stderr,"WS connected\n");

        return true;
    }

    virtual void handleReadyState(CivetServer *server,
                                  struct mg_connection *conn)
    {
        // printf("WS ready\n");
        {
            std::lock_guard<std::mutex> l(connectionsLock_);
            connections_.insert(conn);
        }
        //update_web_param();
        // process_param(span);
        //  process_param(centrefreq);
        /*
         send(centrefreq, strlen(centrefreq),MG_WEBSOCKET_OPCODE_TEXT);
        send(span, strlen(centrefreq),MG_WEBSOCKET_OPCODE_TEXT);
        */
    }

    virtual bool handleData(CivetServer *server,
                            struct mg_connection *conn,
                            int bits,
                            char *data,
                            size_t data_len)
    {
        // printf("WS got %lu bytes: ", (long unsigned)data_len);
        // fwrite(data, 1, data_len, stdout);
        // printf("\n");
        char message[255];
        strncpy(message, data, data_len);
        char name[255];
        extract_between_quotes(message, name);
        fprintf(stderr, "Message received : %s : name %s ", message, name);
        if (strcmp(name, "freq") == 0)
        {
            char *pch;
            pch = strstr(message, ":");
            char value[255];
            strncpy(value, pch + 1, strlen(pch - 2));
            // set_frequency_span(atof(value), span);
        }

        return (data_len < 4);
    }

    virtual void handleClose(CivetServer *server,
                             const struct mg_connection *conn)
    {

        std::lock_guard<std::mutex> l(connectionsLock_);

        auto it = connections_.find(const_cast<struct mg_connection *>(conn));
        if (it != connections_.end())
            connections_.erase(it);
        // printf("WS closed\n");
    }

    void process(uint16_t *bin, int fftlen)
    {

        send((char *)bin, fftlen *2, MG_WEBSOCKET_OPCODE_BINARY);
    }

    void process_param(char *param)
    {
        fprintf(stderr, "Param : %s\n", param);
        send(param, strlen(param), MG_WEBSOCKET_OPCODE_TEXT);
    }

    void send(char *data, int len, int MessageType)
    {
        std::lock_guard<std::mutex> l(connectionsLock_);
        std::vector<struct mg_connection *> errors;
        for (auto it : connections_)
        {
            if (0 >= mg_websocket_write(it, MessageType, data, len))
                errors.push_back(it);
        }

        for (auto it : errors)
            connections_.erase(it);
    }
};

WebSocketHandler h_websocket;

void jsonize_param(const char *name, float param, char *jsonresult)
{
    char paramjson[255];

    if (strlen(jsonresult) == 0)
    {
        sprintf(paramjson, "{\"%s\":%.0f", name, param);
    }
    else
    {
        sprintf(paramjson, ",\"%s\":%.0f", name, param);
    }

    strcat(jsonresult, paramjson);
}

void update_web_param(float freqrx,float span)
{
    char webmessage[255];
    strcpy(webmessage, "");
    
        jsonize_param("center", 745e6 /*freqrx*/, webmessage);
        
        jsonize_param("span", 10e6 /*span*/, webmessage);
        
        strcat(webmessage, "}");
        fprintf(stderr, "Webmessage = %s\n", webmessage);
       
    h_websocket.process_param(webmessage);
}

uint16_t m_fftsize = 0;
uint16_t m_average = 1;

ne10_fft_cpx_int16_t *iqsample = NULL;
ne10_fft_cpx_int16_t *fftsample = NULL;

float *power = NULL;
uint16_t *powerdb = NULL;
ne10_fft_cfg_int16_t cfg;
float *hanning_window_const = NULL;

void PrepareFFT()
{

    iqsample = (ne10_fft_cpx_int16_t *)malloc(m_fftsize * sizeof(ne10_fft_cpx_int16_t));
    fftsample = (ne10_fft_cpx_int16_t *)malloc(m_fftsize * sizeof(ne10_fft_cpx_int16_t));
    power = (float *)malloc(m_fftsize * sizeof(float));
    powerdb = (uint16_t *)malloc(m_fftsize * sizeof(uint16_t));
    cfg = ne10_fft_alloc_c2c_int16(m_fftsize);
    hanning_window_const = (float *)malloc(m_fftsize * sizeof(float));

    for (int i = 0; i < m_fftsize; i++)
    {
        hanning_window_const[i] = 0.5 * (1.0 - cos(2 * M_PI * (((double)i) / m_fftsize)));
    }
}

void iqtofft(short *bufferiq, uint16_t RxSize)
{
    static size_t average_iteration = 0;

    //fprintf(stderr, "FFT %d \n",RxSize);
    if (RxSize % m_fftsize != 0)
    {
        fprintf(stderr, "Rx fft is not size aligned %d\n",RxSize);
        return;
    }

    for (size_t i = 0; i < RxSize/m_fftsize; i++)
    {

        if (average_iteration == 0)
        {
            for (size_t k = 0; k < m_fftsize; k++)
        {
            power[k]=0.0;
        }
            
        }
        
       /*for (size_t k = 0; k < m_fftsize; k++)
        {
            bufferiq[2*k]=bufferiq[2*k]*hanning_window_const[k];
            bufferiq[2*k+1]=bufferiq[2*k+1]*hanning_window_const[k];
        } 
         */  
        // Perfom FFT
        ne10_fft_c2c_1d_int16(fftsample, (ne10_fft_cpx_int16_t *)bufferiq + i * m_fftsize, cfg, 0, 0);
        for (size_t k = 0; k < m_fftsize; k++)
        {
             //power[k]+=sqrt(fftsample[k].i*fftsample[k].i+fftsample[k].r*fftsample[k].r)/m_average;
             if(k<m_fftsize/2)
             {
                power[k+m_fftsize/2]+=sqrt(fftsample[k].i*fftsample[k].i+fftsample[k].r*fftsample[k].r);
             }
             else
             {
                power[k-m_fftsize/2]+=sqrt(fftsample[k].i*fftsample[k].i+fftsample[k].r*fftsample[k].r);
             }

        }

        if((average_iteration+1)%m_average==0)
        {
            
            uint32_t max=0;
            static float floor=0xFFFFFF;
            for (size_t k = 0; k < m_fftsize; k++)
            {
                
            //if(power[k]<floor) floor=power[k]/(m_average*m_fftsize);
            if(power[k]<floor) floor=power[k];
            if(power[k]>max) max=power[k];
            }

            for (size_t k = 0; k < m_fftsize; k++)
            {
                
                //Should be divided by m_fftsize and m_average for unity gain    
                powerdb[ k]=(100.0*log10(power[k]-floor));
                //fprintf(stderr,"%d \n",powerdb[ k]);        
            }
            //fprintf(stderr,"Min %d Max %d\n",floor,max);    
            h_websocket.process(powerdb, m_fftsize);
            usleep(20000);
        }
       average_iteration= (average_iteration+1)%m_average;

    }
}

#define WS_PORT "7681"
#define DOCUMENT_ROOT "."

CivetServer *server; // <-- C++ style start

void init_fft(uint16_t fft_size, uint16_t average)
{
    m_fftsize = fft_size;
    m_average = average;

    // CivetWeb INIT
    mg_init_library(MG_FEATURES_WEBSOCKET);
    if (mg_check_feature(MG_FEATURES_WEBSOCKET) > 0)
        fprintf(stderr, "WebSocket Enable\n");
    else
           fprintf(stderr, "ERROR : WebSocket is not enable\n");     
    const char *options[] = {
        "document_root", DOCUMENT_ROOT, "listening_ports", WS_PORT, 0};
        
    std::vector<std::string> cpp_options;
    for (int i = 0; i < (sizeof(options) / sizeof(options[0]) - 1); i++)
    {
        cpp_options.push_back(options[i]);
    }
    server=new CivetServer(cpp_options);

    WsStartHandler h_ws;
    server->addHandler("/ws", h_ws);
    server->addWebSocketHandler("/websocket", h_websocket);
    // End of CivetWeb INIT

    if (ne10_init() != NE10_OK)
    {
        fprintf(stderr, "Failed to initialise Ne10.\n");
        return ;
    }
    if (ne10_HasNEON() == NE10_OK)
    {
        fprintf(stderr, "Init Neon10 with NEON\n");
    }
    PrepareFFT();
}
