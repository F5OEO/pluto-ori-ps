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

#include "mymqtt.h"
#include <jansson.h>

#include "mqtthandlecommand.h"


#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h> /* IPPROTO_IP, sockaddr_in, htons(),
htonl() */
#include <arpa/inet.h>	/* inet_addr() */
#include <netdb.h>
#include <time.h>
#include <sys/ioctl.h>
using namespace std;

#define PROGRAM_VERSION "0.0.1"
static int want_quit = 0;
char sSerial[255];

void print_usage()
{

	fprintf(stderr,
			"pluto_mqtt_ctrl -%s\n\
Usage:\npluto_mqtt_ctrl \n\
\n",
			PROGRAM_VERSION);

} /* end function print_usage */

/* Callback called when the client receives a message. */
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{

    
    char *key = msg->topic;
	char *svalue = (char *)msg->payload;

    FILE *fdserial=NULL;
    
    //fprintf(stderr,"%x %s %s\n",mosq,key,svalue);

    char ValidCommand[512];
    sprintf(ValidCommand,"cmd/pluto/%s/",sSerial);

   
	if (strncmp(key, ValidCommand,strlen(ValidCommand)) == 0)
	{
       
		HandleCommand(key+strlen(ValidCommand),svalue);
	}
    else
    {
        //fprintf(stderr,"Invalid command %s ->%s\n",key,ValidCommand);
    }
}



   
// Global signal handler for trapping SIGINT, SIGTERM, and SIGQUIT
static void signal_handler(int signal)
{
	want_quit = 1;
	
}

int main(int argc, char **argv)
{
    signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGQUIT, signal_handler);
	signal(SIGPIPE, signal_handler);

    

    mqttinit();
    size_t count=0;
    
    /*
     FILE *fdserial=NULL;
    
    fdserial=fopen("/sys/kernel/config/usb_gadget/composite_gadget/strings/0x409/serialnumber","r");
    
    fscanf(fdserial,"%s",sSerial);
    fclose(fdserial);
    */
    //fprintf(stderr,"line1:%sline2%s\n",sSerial,sSerial);
    
FILE *cmd=popen("fw_printenv -n call", "r");
    char result[255]={0x0};
    //fgets(result, sizeof(result), cmd); 
    fscanf(cmd,"%s",result); 
    if(strcmp(result,"")==0)
    {
        strcpy(result,"nocall");
        
    } 
    pclose(cmd);

    strcpy(sSerial,result);

    //fprintf(stderr,"KeyRoot %s\n",sSerial);
    
    HandleCommandInit(mosq,sSerial);

       
    while(want_quit!=1)
    {
        usleep(100000);
        if(count==10)
        {
            count=0;
            PubTelemetry();
        }
        count++;
    }
    mqttend();
    return 0;
}    
