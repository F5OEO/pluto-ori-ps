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

#include <pthread.h>

#include <jansson.h>

//#include "ad9363.h"

#include "mqtthandlecommand.h"
#include "iiofshelper.h"
#include <math.h>
#include "nco_conf.h" // Oscimp nco

char sCmdRoot[255];
char sDtRoot[255];
struct mosquitto *m_mosq;
size_t m_finalrxsr = 0;
size_t m_finaltxsr = 0;
size_t m_adcdacsr = 0;

enum
{
    cs16,
    cs8,
    cs16fft
};
size_t m_format = cs16;

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
    char pubkey[512];
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
    // fprintf(stderr,"%s %s\n",iio_key,svalue);
    char pubkey[255];
    sprintf(pubkey, "%s%s", sDtRoot, mqttkey);

    mosquitto_publish(m_mosq, NULL, pubkey, strlen(svalue), svalue, 2, false);
    return true;
}

void GetKey(char *iio_key, char *svalue)
{
    FILE *fdread = NULL;
    fdread = fopen(iio_key, "r");
    fscanf(fdread, "%s", svalue); // To avoid getting units
    fclose(fdread);
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

float set_filter_gain(double *filter, float gain, int ntaps)
{
    double sum = 0;
    double max = 0;
    for (int i = 0; i < ntaps; i++)
    {
        sum += filter[i];
    }
    gain = gain / sum;
    for (int i = 0; i < ntaps; i++)
    {
        filter[i] = filter[i] * gain;
        if (fabs(filter[i]) > fabs(max))
            max = filter[i];
    }
    return max;
}

void build_lpf_filter(double *filter, float bw, int ntaps)
{
    double a;
    double B = bw;               // filter bandwidth
    double t = -(ntaps - 1) / 2; // First tap
    // Create the filter
    for (int i = 0; i < ntaps; i++)
    {
        if (t == 0)
            a = 2.0 * B;
        else
            a = 2.0 * B * sin(M_PI * t * B) / (M_PI * t * B);
        filter[i] = (double)a;
        t = t + 1.0;
    }
    set_filter_gain(filter, 1.0, ntaps);
}

void build_rrc_filter(double *filter, float rolloff, int ntaps, int samples_per_symbol)
{
    double a, b, c, d;
    double B = rolloff + 0.0001;    // Rolloff factor .0001 stops infinite filter coefficient with 0.25 rolloff
    double t = -(ntaps - 1) / 2;    // First tap
    double Ts = samples_per_symbol; // Samples per symbol
    // Create the filter
    for (int i = 0; i < (ntaps); i++)
    {
        a = 2.0 * B / (M_PI * sqrt(Ts));
        b = cos((1.0 + B) * M_PI * t / Ts);
        // Check for infinity in calculation (computers don't have infinite precision)
        if (t == 0)
            c = (1.0 - B) * M_PI / (4 * B);
        else
            c = sin((1.0 - B) * M_PI * t / Ts) / (4.0 * B * t / Ts);

        d = (1.0 - (4.0 * B * t / Ts) * (4.0 * B * t / Ts));
        // filter[i] = (b+c)/(a*d);//beardy
        filter[i] = (float)(a * (b + c) / d); // nasa
        t = t + 1.0;
    }
    set_filter_gain(filter, 1.0, ntaps);
}

void load_ad9363fir(double *fir, int taps, int ratio, bool enable, float gain, int firgain, bool tx)
{

    int buffsize = 8192;
    char *buf = (char *)malloc(buffsize);
    int clen = 0;
    if (tx) // Fir Tx gain {-6,0}
    {
        clen += snprintf(buf + clen, buffsize - clen, "TX 3 GAIN %d INT %d\n", firgain, ratio); // The filter provides a fixed +6dB gain to maximize dynamic range, so the programmable gain is typically set to -6dB to produce a net gain of 0dB
        //fprintf(stderr, "TX 3 GAIN %d INT %d\n", firgain, ratio);
    }
    else // Fir Rx gain {-12,-6,0,+6}
    {
        clen += snprintf(buf + clen, buffsize - clen, "RX 3 GAIN %d DEC %d\n", firgain, ratio); // The filter provides a fixed +6dB gain to maximize dynamic range, so the programmable gain is typically set to -6dB to produce a net gain of 0dB
        //fprintf(stderr, "RX 3 GAIN %d DEC %d\n", firgain, ratio);
    }

    short coeffir = tx ? 0x7FFF : 0x7FFF;
    for (int i = 0; i < taps; i++)
    {

        clen += snprintf(buf + clen, buffsize - clen, "%d\n", (short)(coeffir * fir[i] * gain)); // Fixme ! 1FFF instead of 0x3FFF seems better but should have to be inspect
        // fprintf(stderr,"%d\n", (short)(coeffir * fir[i] * gain));
    }
    clen += snprintf(buf + clen, buffsize - clen, "\n");

    // fprintf(stderr,"Coef:%s\n",buf);

    FILE *fdwrite = NULL;
    fdwrite = fopen("/sys/bus/iio/devices/iio:device0/filter_fir_config", "wb");
    fwrite(buf, clen, 1, fdwrite);
    fclose(fdwrite);
    // Seems that enable only one has some board effect to other
    /*
    if(tx&&enable)
      SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "1"); //Enable FIR TX
   if((!tx)&&enable)
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "1");  //Enable FIR RX
    */
    free(buf);
}

enum
{
    typerrc,
    typelpf,
    unity
};

/*
The last/first digital filter in the RX/TX signal path is a programmable poly-phase FIR filter. The RX/TX FIR filter can decimate/interpolate by a factor of 1, 2, or 4, or it can be bypassed if not needed. The filter taps are configurable in groups of 16 between a minimum of 16 and a maximum of 128 taps. The RX FIR also has a programmable gain of -12dB, -6dB, 0dB, or +6dB. The filter provides a fixed +6dB gain to maximize dynamic range, so the programmable gain is typically set to -6dB to produce a net gain of 0dB. The TX FIR also has a programmable gain setting of 0dB or -6dB. Be aware there are some limitations in terms of the maximum number of taps supported by the different clock ratios, please consult the AD9361 manual for more details. The AD9361 device driver will warn if a programmed filter doesn’t match the limits.
*/

void setad9363filter(int ratio, int hardupsample, float rolloff, int type, float digitalgain, bool db6boost, bool enable, bool tx)
{

    double fir[128];

    int NbTaps;
    if (hardupsample > 1)
    {
        NbTaps = 127;
        // NbTaps = 63;
    }
    else
    {
        NbTaps = 63;
    }

    switch (type)
    {
    case typerrc:
    {

        build_rrc_filter(fir, rolloff, NbTaps, hardupsample);
    }
    break;
    case typelpf:
    {
        build_lpf_filter(fir, (rolloff) / (float)(hardupsample), NbTaps);
    }
    break;
    }
    float gain = digitalgain; // log2(2 * hardupsample) * digitalgain;

    fir[NbTaps] = 0.0;
    if (tx)
        load_ad9363fir(fir, NbTaps + 1, hardupsample, enable, gain, db6boost ? 0 : -6, tx);
    else
        load_ad9363fir(fir, NbTaps + 1, hardupsample, enable, gain, db6boost ? 0 : -6, tx);
}

/*
bool ComputeSR(int tx,char *svalue)
{
    if (strcmp(svalue, "?") == 0)
    {
        if (m_finalrxsr != 0) // Fixme : Should be calculated with sr/fpga flag and ad decim
        {
              publish("rx/finalsr", m_finalrxsr);
              publish("tx/finalsr", m_finaltxsr);

        }
        publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "sr");
        //publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "rx/sr");

        return true; // This is a request status
    }

    float MinADC = 25e6 / 12;
    float MaxADC = 61.4e6;
    float fpgadecim = 32.0;
    float ad9363decim = 4.0;

     float RequestSR = atof(svalue);
     if (RequestSR==m_finalsr[tx]) return true; //Same SR as before
     if (RequestSR > MaxADC) // >61.4M
    {
        return false;
    }
    if (RequestSR < MinADC / fpgadecim / ad9363decim) // < 16K
    {
        return false;
    }
    if (RequestSR >= MinADC) // High bitrate, no fpga but ad9363ecim if possible
    {
        if (RequestSR < MaxADC / 4)
        {

            //ad9363decim=4.0;
            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
        else if (RequestSR < MaxADC / 2)
        {

            //ad9363decim=2.0;
            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
        else
        {

            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
    }
    else if (RequestSR >= MinADC / 4) // We use AD decim
    {
        ad9363decim = 4.0;
        fpgadecim = 1.0;
    }
    else
    {
        if (RequestSR >= MinADC / 32)
        {
            ad9363decim = 1.0;
            fpgadecim = 32.0;
        }
        else
        {
            ad9363decim = 4.0;
            fpgadecim = 32.0;

        }
    }

}
*/
bool ComputeRxSR(char *svalue)
{

    if (strcmp(svalue, "?") == 0)
    {
        if (m_finalrxsr != 0) // Fixme : Should be calculated with sr/fpga flag and ad decim
        {

            publish("rx/finalsr", m_finalrxsr);
        }
        publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "sr");
        // publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "rx/sr");

        return true; // This is a request status
    }
    float MinADC = 25e6 / 12;
    float MaxADC = 61.4e6;
    float fpgadecim = 32.0;
    float ad9363decim = 4.0;

    bool result = true;
    float RequestSR;
    float ADCSR;

    RequestSR = atof(svalue);

    if (RequestSR > MaxADC) // >61.4M
    {
        return false;
    }
    if (RequestSR < MinADC / fpgadecim / ad9363decim) // < 16K
    {
        return false;
    }

    if (RequestSR >= MinADC) // High bitrate, no fpga but ad9363ecim if possible
    {
        if (RequestSR < MaxADC / 4)
        {

            // ad9363decim=4.0;
            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
        else if (RequestSR < MaxADC / 2)
        {

            // ad9363decim=2.0;
            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
        else
        {

            ad9363decim = 1.0;
            fpgadecim = 1.0;
        }
    }
    else if (RequestSR >= MinADC / 4) // We use AD decim
    {
        ad9363decim = 4.0;
        fpgadecim = 1.0;
    }
    else
    {
        if (RequestSR >= MinADC / 32)
        {
            ad9363decim = 1.0;
            fpgadecim = 32.0;
        }
        else
        {
            ad9363decim = 4.0;
            fpgadecim = 32.0;
        }
    }

    // fprintf(stderr,"SR -> ad9363 %.0f fpga %.0f \n",ad9363decim,fpgadecim);
    ADCSR = RequestSR * fpgadecim * ad9363decim;
    char sSR[255];
    sprintf(sSR, "%.0f", ADCSR);
    m_adcdacsr = RequestSR * fpgadecim;
    // First set result SR
    SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR); // RX Fpga

    //  SSCANF de /sys/bus/iio/devices/iio:device0/rx_path_rates
    // NbTaps = ADC/RXSAMP * 16 !!!!! FixMe for better filtering

    // AD9363Decim
    if (ad9363decim > 1.0)
    {
        setad9363filter(1, ad9363decim, 1.0 / ad9363decim, typelpf, 1.0, false, true, false);
        setad9363filter(1, ad9363decim, 1.0 / ad9363decim, typelpf, 1.0, true, true, true); // We need also to do tx for now

        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "1");

        sprintf(sSR, "%.0f", ADCSR / ad9363decim);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    }
    else
    {
        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "0");
        // SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "0"); //Disable  ADFIR
        // SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "0"); //Disable  ADFIR
    }

    if (fpgadecim > 1.0)
    {
        sprintf(sSR, "%.0f", ADCSR / 8 / ad9363decim);
        fprintf(stderr, "adcsr %f fpga %s\n", ADCSR, sSR);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    else
    {
        sprintf(sSR, "%.0f", ADCSR / ad9363decim);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    // Automatic Analog LPF bandwidth
    if (RequestSR > 200e3)
    {
        sprintf(sSR, "%.0f", RequestSR);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_rf_bandwidth", sSR);
    }
    else
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_rf_bandwidth", "200000"); // Mini bandwidth
    m_finalrxsr = RequestSR;
    publish("finalsr", m_finalrxsr);
    publish("rx/finalsr", m_finalrxsr);
    publish("rx/fpgadecim", (float)fpgadecim);
    publish("rx/ad9361decim", (float)ad9363decim);
    publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "sr");
    publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "rx/sr");

    return result;
}

bool ComputeTxSR(char *svalue)
{
    if (strcmp(svalue, "?") == 0)
    {
        if (m_finaltxsr != 0) // Fixme : Should be calculated with sr/fpga flag and ad interpol
        {
            // publish("finalsr", m_finaltxsr);
            publish("tx/finalsr", m_finaltxsr);
        }
        publishstatus("/sys/bus/iio/devices/iio:device0/out_voltage_sampling_frequency", "sr");
        // publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "tx/sr");
        return true; // This is a request status
    }
#define FPGA_INTERPOL (8.0)
    float MinDAC = 25e6 / 12;
    float MaxDAC = 61.4e6;
    float fpgainterpol = FPGA_INTERPOL; // FPGA INTERPOL
    float ad9363interpol = 4.0;

    bool result = true;
    float RequestSR;
    float DACSR;

    RequestSR = atof(svalue); 

    if (RequestSR > MaxDAC) // >61.4M
    {
        return false;
    }
    if (RequestSR < MinDAC / fpgainterpol / ad9363interpol) // < 16K
    {
        // OR NEED a soft upsampler
        return false;
    }

    if (RequestSR >= MinDAC) // High bitrate, no fpga but ad9363ecim if possible
    {
        if (RequestSR < MaxDAC / 4)
        {

            // ad9363decim=4.0;
            ad9363interpol = 1.0;
            fpgainterpol = 1.0;
        }
        else if (RequestSR < MaxDAC / 2)
        {

            // ad9363decim=2.0;
            ad9363interpol = 1.0;
            fpgainterpol = 1.0;
        }
        else
        {

            ad9363interpol = 1.0;
            fpgainterpol = 1.0;
        }
    }
    else if (RequestSR >= MinDAC / 4) // We use AD decim
    {
        ad9363interpol = 4.0;
        fpgainterpol = 1.0;
        // ad9363interpol = 1.0;
        //      fpgainterpol =FPGA_INTERPOL;
    }
    else
    {
        if (RequestSR >= MinDAC / FPGA_INTERPOL)
        {
            ad9363interpol = 1.0;
            fpgainterpol = FPGA_INTERPOL;
        }
        else
        {

            ad9363interpol = 4.0;
            fpgainterpol = FPGA_INTERPOL;
        }
    }

    DACSR = RequestSR * fpgainterpol * ad9363interpol;
    char sSR[255];
    sprintf(sSR, "%.0f", DACSR);
    m_adcdacsr = RequestSR * fpgainterpol;
    // First set result SR

    char sMute[10];
    GetKey("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", sMute);
    // Mute if we are using interpol
    if (fpgainterpol * ad9363interpol > 1)
        SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", "1");
    SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
    SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga

    // AD9363Interpol
    if (ad9363interpol > 1.0)
    {
        // Work for DVBS2 mode ! Fixme to see with general IQ
        setad9363filter(1, ad9363interpol, 2.0 / ((float)ad9363interpol), typelpf, 3.0, true, true, true);

        // setad9363filter(1, ad9363interpol, 0.7, typerrc, 1.0, false,true,true);
        setad9363filter(1, ad9363interpol, 1, typelpf, 1.0, false, true, false); // WE need also to do it on RX

        // setad9363filter(1, ad9363interpol, 0.35, typerrc, 1.0, false,true,true);
        // setad9363filter(1, ad9363interpol, 0.35, typerrc, 1.0, false,true,false);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "1");
        // SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "1");
        // SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "1");

        sprintf(sSR, "%.0f", DACSR / ad9363interpol);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    }
    else
    {
        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "0");
        // SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "0"); //Disable  ADFIR
        // SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "0"); //Disable  ADFIR
    }

    if (fpgainterpol > 1.0)
    {
        sprintf(sSR, "%.0f", DACSR / 8 / ad9363interpol);
        fprintf(stderr, "dacsr %f fpga %s\n", DACSR, sSR);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    else
    {
        sprintf(sSR, "%.0f", DACSR / ad9363interpol);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    // Automatic Analog LPF bandwidth
    if (RequestSR > 200e3)
    {
        sprintf(sSR, "%.0f", RequestSR);
        SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_rf_bandwidth", sSR);
    }
    else
        SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_rf_bandwidth", "200000"); // Mini bandwidth
    m_finaltxsr = RequestSR;
    // Unmute if we are not muted before
    if (atoi(sMute) == 0)
        SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", "0");
    publish("tx/finalsr", m_finaltxsr);
    publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "sr");
    publishstatus("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", "tx/sr");
    publish("tx/fpgainterpol", (float)fpgainterpol);
    publish("tx/ad9363interpol", (float)ad9363interpol);

    return result;
}

//Special case where we want to rx at max bandwidth
bool ComputeTxSRDVBS2(char *svalue)
{
    if (strcmp(svalue, "?") == 0)
    {
       
        if (m_finaltxsr != 0) // Fixme : Should be calculated with sr/fpga flag and ad interpol
        {
            // publish("finalsr", m_finaltxsr);
            publish("tx/finalsr", m_finaltxsr);
        }
        publishstatus("/sys/bus/iio/devices/iio:device0/out_voltage_sampling_frequency", "sr");
        
        
        // publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "tx/sr");
        return true; // This is a request status
    }
#define FPGA_INTERPOL (8.0)
#define FPGA_DVBS2 4
    float MinDAC = 25e6 / 12;
    float MaxDAC = 61.4e6;
    float fpgainterpol = FPGA_INTERPOL; // FPGA INTERPOL
    float ad9363interpol = 4.0;

    bool result = true;
    float RequestSR;
    float DACSR;

    RequestSR = atof(svalue); 

    if (RequestSR > MaxDAC) // >61.4M
    {
        return false;
    }
    if (RequestSR < MinDAC / fpgainterpol / ad9363interpol) // < 16K
    {
        // OR NEED a soft upsampler
        return false;
    }

    if (RequestSR >= MinDAC) // High bitrate, no fpga but ad9363ecim if possible
    {
        if (RequestSR < MaxDAC / 8)
        {

            
            ad9363interpol = 1.0;
            fpgainterpol = FPGA_INTERPOL;
        }
        else if (RequestSR < MaxDAC / 4)
        {

            // ad9363decim=2.0;
            ad9363interpol = 4.0;
            fpgainterpol = 1.0;
        }
        else
        {

            ad9363interpol = 1.0;
            fpgainterpol = 1.0;
        }
    }
    else if (RequestSR >= MinDAC / FPGA_INTERPOL) // We use AD decim
    {
        ad9363interpol = 1.0;
        fpgainterpol = 1.0;
        // ad9363interpol = 1.0;
        fpgainterpol =FPGA_INTERPOL;
    }
    else
    {
        
            ad9363interpol = 4.0;
            fpgainterpol = FPGA_INTERPOL;
        
    }

    DACSR = RequestSR * fpgainterpol * ad9363interpol;
    char sSR[255];
    sprintf(sSR, "%.0f", DACSR);
    m_adcdacsr = RequestSR * fpgainterpol;
    // First set result SR

    char sMute[10];
    GetKey("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", sMute);
    // Mute if we are using interpol
    if (fpgainterpol * ad9363interpol > 1)
        SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", "1");
    SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
    SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga

    // AD9363Interpol
    if (ad9363interpol > 1.0)
    {
        // Work for DVBS2 mode ! Fixme to see with general IQ
        setad9363filter(1, ad9363interpol, 2.0 / ((float)ad9363interpol), typelpf, 3.0, true, true, true);

        // setad9363filter(1, ad9363interpol, 0.7, typerrc, 1.0, false,true,true);
        setad9363filter(1, ad9363interpol, 1, typelpf, 1.0, false, true, false); // WE need also to do it on RX

        // setad9363filter(1, ad9363interpol, 0.35, typerrc, 1.0, false,true,true);
        // setad9363filter(1, ad9363interpol, 0.35, typerrc, 1.0, false,true,false);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "1");
        // SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "1");
        // SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "1");

        sprintf(sSR, "%.0f", DACSR / ad9363interpol);
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", sSR);
    }
    else
    {
        SendCommand("/sys/bus/iio/devices/iio:device0/in_out_voltage_filter_fir_en", "0");
        // SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_filter_fir_en", "0"); //Disable  ADFIR
        // SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_filter_fir_en", "0"); //Disable  ADFIR
    }

    if (fpgainterpol > 1.0)
    {
        sprintf(sSR, "%.0f", DACSR / 8 / ad9363interpol);
        //fprintf(stderr, "dacsr %f fpga %s\n", DACSR, sSR);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    else
    {
        sprintf(sSR, "%.0f", DACSR / ad9363interpol);
        SendCommand("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", sSR); // TX Fpga
        SendCommand("/sys/bus/iio/devices/iio:device3/in_voltage_sampling_frequency", sSR);  // RX Fpga
    }

    // Automatic Analog LPF bandwidth TX
    if (RequestSR/FPGA_DVBS2 > 200e3)
    {
        sprintf(sSR, "%.0f", RequestSR/FPGA_DVBS2);
        SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_rf_bandwidth", sSR);
       
    }
    else
    {
        
        SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage_rf_bandwidth", "200000"); // Mini bandwidth
       
    }    

    // Automatic Analog LPF bandwidth RX
    if (RequestSR*fpgainterpol> 200e3)
    {
        sprintf(sSR, "%.0f",RequestSR*fpgainterpol);
        
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_rf_bandwidth", sSR); //Set it also to rx
    }
    else
    {
        
        SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage_rf_bandwidth","200000");
    }

    m_finaltxsr = RequestSR;
    // Unmute if we are not muted before
    if (atoi(sMute) == 0)
        SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", "0");
    publish("tx/finalsr", m_finaltxsr);
    publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage_sampling_frequency", "sr");
    publishstatus("/sys/bus/iio/devices/iio:device2/out_voltage_sampling_frequency", "tx/sr");
    publish("tx/fpgainterpol", (float)fpgainterpol);
    publish("tx/ad9363interpol", (float)ad9363interpol);

    return result;
}

size_t SetFormat(size_t requestformat)
{
    // size_t current_value=readiiohex("/sys/bus/iio/devices/iio:device3/direct_reg_access");
    size_t value = ReadRegister(0x79020000 + 0xBC);
    switch (requestformat)
    {
    case cs16:
    {

        WriteRegister(0x79020000 + 0xBC, (value & 0xFFF1) | 0);
        break;
    }
    case cs8:
    {

        WriteRegister(0x79020000 + 0xBC, (value & 0xFFF1) | 2);

        break;
    }
    case cs16fft:
    {
        WriteRegister(0x79020000 + 0xBC, (value & 0xFFF1) | 4);
        break;
    }
    default:
        return m_format;
    }
    return requestformat;
}

char strcmd[][255] = {"listcmd", "rx/frequency", "rx/modegain", "rx/gain", "rx/sr", "rx/format", "rx/mute",
                      "tx/frequency", "tx/gain", "tx/sr", "tx/format", "tx/mute", "tx/nco", ""};
enum defidx
{
    listcmd,
    cmd_rxfrequency,
    cmd_rxmodegain,
    cmd_rxgain,
    cmd_rxsr,
    cmd_rxformat,
    cmd_rxmute,
    cmd_txfrequency,
    cmd_txgain,
    cmd_txsr,
    cmd_txformat,
    cmd_txmute,
    cmd_txnco
};

bool publishcmd()
{
    char svalue[2500];
    sprintf(svalue,"");
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        strcat(svalue, strcmd[i]);
        strcat(svalue, ",");
    }
    publish("listcmd_ctrl", (char *)svalue);
    // mosquitto_publish(m_mosq, NULL, "listcmd", strlen(svalue), svalue, 2, false);
    return true;
}

void PubTelemetry()
{
    FILE *fdserial = NULL;
    char sSerial[255];
    fdserial = fopen("/sys/firmware/devicetree/base/model", "r");
    fgets(sSerial, 255, fdserial);
    fclose(fdserial);
    publish("system/device", sSerial);
    // RSSI
    publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage0_rssi", "rx/rssi");
    // AD TEMPERATURE
    publishstatus("/sys/bus/iio/devices/iio:device0/in_temp0_input", "temperature_ad");

    // Publish complete status by sending cmd with ?
    char svalue[2500];
    sprintf(svalue, "");
    for (int i = 0; strcmp(strcmd[i], "") != 0; i++)
    {
        HandleCommand(strcmd[i], "?");
    }
}

bool HandleCommand(char *key, char *soriginvalue)
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
    char svalue[255];
    // fprintf(stderr,"Handle command %s %s\n",key,svalue);
    if (toupper(soriginvalue[strlen(soriginvalue) - 1]) == 'M')
    {
        soriginvalue[strlen(soriginvalue) - 1] = 0;
        float fvalue = atof(soriginvalue) * 1e6;
        sprintf(svalue, "%.0f", fvalue);
    }
    else if (toupper(soriginvalue[strlen(soriginvalue) - 1]) == 'K')
    {
        soriginvalue[strlen(soriginvalue) - 1] = 0;
        float fvalue = atof(soriginvalue) * 1e3;
        sprintf(svalue, "%.0f", fvalue);
    }
    else
    {
        float fvalue = atof(soriginvalue);
        if (fvalue != 0)
        {
            if((fvalue - floor(fvalue))!=0)
               sprintf(svalue, "%.2f", fvalue);
            else   
            sprintf(svalue, "%.0f", fvalue);
        }    
        else
            strcpy(svalue, soriginvalue); // Not a numerical value
    }
    switch (cmdidx)
    {
    case listcmd:
    {
        publishcmd();
        break;
    }
    case cmd_rxfrequency:
    {
        if (strcmp(svalue, "?") != 0)
            SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage0_RX_LO_frequency", svalue);
        publishstatus("/sys/bus/iio/devices/iio:device0/out_altvoltage0_RX_LO_frequency", key);
        break;
    }

    case cmd_rxmute:
    {
        SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage0_RX_LO_powerdown", svalue);
        publishstatus("/sys/bus/iio/devices/iio:device0/out_altvoltage0_RX_LO_powerdown", key);
        break;
    }

    case cmd_rxmodegain: // {manual,slow_attack,hybrid,fast_attack}
    {
        if (strcmp(svalue, "?") != 0)
            SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage0_gain_control_mode", svalue);
        publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage0_gain_control_mode", key);
        break;
    }
    case cmd_rxgain:
    {
        // First go to manual gain
        if (strcmp(svalue, "?") != 0)
            SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage0_gain_control_mode", "manual");
        publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage0_gain_control_mode", strcmd[cmd_rxmodegain]);

        // THen set manual gain
        if (strcmp(svalue, "?") != 0)
            SendCommand("/sys/bus/iio/devices/iio:device0/in_voltage0_hardwaregain", svalue);
        publishstatus("/sys/bus/iio/devices/iio:device0/in_voltage0_hardwaregain", key);
        break;
    }

    case cmd_rxsr:
    {
        ComputeRxSR(svalue);
        break;
    }

    case cmd_rxformat:
    {
        if (strcmp(svalue, "?") == 0)
        {

            size_t value = ReadRegister(0x79020000 + 0xBC);
            // fprintf(stderr,"Reg %x\n",value);
            m_format = log2(value & 0xE);
            publish("rx/format", (float)m_format);
            break;
        }

        int requestformat = atoi(svalue);
        m_format = SetFormat(requestformat);
        publish("rx/format", m_format);
        break;
    }
    case cmd_txfrequency:
    {
        if (strcmp(svalue, "?") != 0)
            SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_frequency", svalue);
        publishstatus("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_frequency", key);
        break;
    }

    case cmd_txmute:
    {
        if (strcmp(svalue, "?") != 0)
        {
            SendCommand("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", svalue);
            if (strcmp(svalue, "1")) // ptt off
            {
                SendCommand("/sys/kernel/debug/iio/iio:device0/direct_reg_access", "0x26 0x10");
                SendCommand("/sys/kernel/debug/iio/iio:device0/direct_reg_access", "0x27 0x00");
            }
            else // ptt on
            {
                SendCommand("/sys/kernel/debug/iio/iio:device0/direct_reg_access", "0x26 0x10");
                SendCommand("/sys/kernel/debug/iio/iio:device0/direct_reg_access", "0x27 0x50");
            }
        }
        publishstatus("/sys/bus/iio/devices/iio:device0/out_altvoltage1_TX_LO_powerdown", key);
        break;
    }

    case cmd_txsr:
    {
        ComputeTxSRDVBS2(svalue);
        break;
    }

    case cmd_txgain:
    {
        if (strcmp(svalue, "?") != 0)
        {
            SendCommand("/sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain", svalue);
            
        }
        publishstatus("/sys/bus/iio/devices/iio:device0/out_voltage0_hardwaregain", key);
        break;
    }
    case cmd_txnco:
    {
#define NCO_ACCUM_SIZE 28
        static float m_nco = 0;
        if (strcmp(svalue, "?") != 0)
        {
            m_nco = atof(svalue);
            fprintf(stderr, "adcdacsr %d\n", m_adcdacsr);
            // Fixme! NCO is before ad9361 iterpolator : so could be outside of band
            nco_counter_send_conf("/dev/nco00", m_adcdacsr, m_nco >= 0 ? m_nco : m_adcdacsr + m_nco,
                                  NCO_ACCUM_SIZE, 0, 1, 0); // 0, 1, 1 => offset, PINC HW/SF, POFF HW/SF;
        }
        publish("tx/nco", m_nco);

        break;
    }
    }

    return true;
}

void HandleCommandInit(struct mosquitto *mosq, char *sSerial)
{
    m_mosq = mosq;
    sprintf(sDtRoot, "dt/pluto/%s/", sSerial);
    SendCommand("/sys/bus/iio/devices/iio:device0/calib_mode", "manual_tx_quad");
    // iio_device_attr_write(dev, "calib_mode", "manual_tx_quad");
}