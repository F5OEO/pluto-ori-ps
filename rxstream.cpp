/*
  RX IIO buffer thread: reads samples, routes to stdout/UDP/WebFFT.
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include <iio.h>

#include "streamstate.h"
#include "iiostream.h"
#include "rxstream.h"
#include "iqtofft.h"

static uint64_t _timestamp_ns(void)
{
    struct timespec tp;

    if (clock_gettime(CLOCK_REALTIME, &tp) != 0)
        return (0);

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
    if (burstsizerx == 0)
    {
        nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));

        if (nsamples_rx < 0)
            fprintf(stderr, "Error refilling Rx buf %d\n", (int)nsamples_rx);

        *RxBuffer = (short *)iio_buffer_start(m_rxbuf);

        uint32_t val = 0;
        int ret = iio_device_reg_read(m_rx, 0x80000088, &val);
        if (val & 4)
        {
            fflush(stderr);
            Underflow++;
            iio_device_reg_write(m_rx, 0x80000088, val);
        }
    }
    else
    {
        bool underflow = false;
        {
            nsamples_rx = iio_buffer_refill(m_rxbuf) / (2 * sizeof(short));
            uint32_t val = 0;
            int ret = iio_device_reg_read(m_rx, 0x80000088, &val);
            if (val & 4)
            {
                underflow = true;
                iio_device_reg_write(m_rx, 0x80000088, val);
            }
            else
                underflow = false;
        }
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
    case rx_mode_stdout:
    {
        InitRxChannel(200000);
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
    //init_fft(fftsize, 10);
    fprintf(stderr,"Thread rx before pipe\n");
    fdout = fopen("/dev/rx1", "wb+");
    fprintf(stderr,"Thread rx after pipe\n");

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
                uint64_t T0 = _timestamp_ns();

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
                        break;

                    if (RxSize != BufferLenrx)
                        fprintf(stderr, "Read Error %d\n", RxSize);

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
            
            /*
            case rx_mode_websocket:
            {
                size_t bin;

                if(m_sweep==0)
                {
                    RxSize = direct_rx_samples(&RxBuffer);
                    if(RxSize!=0)
                    {
                        publishwebfft(iqtofft(RxBuffer, RxSize,0,&bin),bin);
                        usleep(40000);
                    }
                }
                else
                {
                    uint16_t *powerdb=nullptr;
                    for(size_t i=0;i<m_sweep;i++)
                    {
                        RecallFastlockTune(i);
                        RxSize = direct_rx_samples(&RxBuffer);
                        RxSize = direct_rx_samples(&RxBuffer);

                        if(RxSize!=0)
                        {
                            if(i==0)
                                powerdb= iqtofft(RxBuffer, RxSize,i,&bin);
                            else
                                iqtofft(RxBuffer, RxSize,i,&bin);
                        }
                    }
                    usleep(40000);
                    if(RxSize)
                        publishwebfft(powerdb,bin*m_sweep);
                }
            }
            break;
            */
            case rx_mode_pass:
            {
                usleep(100000);
            }
            break;
            }

            pthread_mutex_unlock(&buffer_mutexrx);
            current_time = _timestamp_ns();
            time_first = current_time;
        }
        else
        {
            usleep(m_latency);
        }
    }

    return NULL;
}
