#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <errno.h>
#include "Dvb.h"
#include "tp.h"

int si_service_desc_fmt( uint8_t *b, si_service_descriptor *sd )
{
	int len = 0;
	b[len++] = SI_DESC_SERVICE;
	len++;
	b[len++] = sd->type;

	b[len++] = sd->provider_length;
	memcpy(&b[len],sd->provider, sd->provider_length );
	len += sd->provider_length;

	b[len++] = sd->name_length;
	memcpy(&b[len],sd->name, sd->name_length );
	len += sd->name_length;
	b[1] = len-2;

	return len;
}

int add_si_descriptor( uint8_t *b, si_desc *d )
{
    int len = 0;

    switch( d->tag )
    {
        case SI_DESC_SERVICE:
            d->sd.tag = d->tag;
            len = si_service_desc_fmt( b, &d->sd );
            break;
       
        default:
            break;
    }
    return len;
}


void update_cont_counter( uint8_t *b )
{
        uint8_t c;

        c = b[3]&0x0F;
        c = (c+1)&0x0F;
        b[3] = (b[3]&0xF0) | c;
}

// Format and send a Transport packet 188.
// Pass the buffer to send, the length of the buffer
// and the type (audio, video, system)
//
// This is for PES packets in transport streams
//
//
// Format a transport packet
//
int tp_fmt( uint8_t *b, tp_hdr *hdr )
{
        int len;

        b[0] = 0x47;
        b[1] = 0;
        if( hdr->transport_error_indicator)     b[1] |= 0x80;
        if( hdr->payload_unit_start_indicator ) b[1] |= 0x40;
        if( hdr->transport_priority )           b[1] |= 0x20;
        // Add the 13 bit pid
        b[1] |= (hdr->pid >> 8 );
        b[2]  = (hdr->pid & 0xFF);
        b[3]  = (hdr->transport_scrambling_control << 6);
        b[3] |= (hdr->adaption_field_control << 4);
        b[3] |= (hdr->continuity_counter)&0x0F;
        len = 4;
        return len;
}

uint32_t dvb_crc32_calc( uint8_t *b, int len )
{
    int i,n,bit;
    unsigned long crc = 0xFFFFFFFF;

    for( n = 0; n < len; n++ )
    {
        for( i=0x80; i; i>>=1 )
        {
            bit = ( ( ( crc&0x80000000 ) ? 1 : 0 ) ^ ( (b[n]&i) ? 1 : 0 ) );
            crc <<= 1;
            if(bit) crc ^= 0x04C11DB7;
        }
    }
    return crc;
}

int crc32_add( uint8_t *b, int len )
{
    unsigned long crc = dvb_crc32_calc( b, len );

    b[len++] = (crc>>24)&0xFF;
    b[len++] = (crc>>16)&0xFF;
    b[len++] = (crc>>8)&0xFF;
    b[len++] = (crc&0xFF);
    return len;
}


uint8_t m_sdt_seq;
uint8_t sdt_pkt[188];

int f_sdts( uint8_t *b, sdt_section *p )
{
    int len,i;
    b[0] = p->service_id>>8;
    b[1] = p->service_id&0xFF;
    b[2] = 0xFC;
    if( p->eit_schedule_flag ) b[2] |= 0x02;
    if( p->eit_present_follow_flag ) b[2] |= 0x01;
    b[3] = p->running_status<<5;
    if( p->free_ca_mode ) b[3] |= 0x10;
    len = 5;
    for( i = 0; i < p->nr_descriptors; i++ )
    {
        len += add_si_descriptor( &b[len], &p->descriptor[i] );
    }
    b[3] |= (len-5)>>8;
    b[4]  = (len-5)&0xFF;
    return len;
}
int f_sdt( uint8_t *b, service_description_section *p )
{	
    int i,len;
    b[0] = 0x42;
    b[1] = 0xF0;
    if( p->section_syntax_indicator ) b[1] |= 0x80;
    b[1] |= 0x40;
    b[1] |= 0x30;
    b[3] = p->transport_stream_id>>8;
    b[4] = p->transport_stream_id&0xFF;
    b[5] = 0xC0;
    b[5] |= p->version_number<<1;
    if( p->current_next_indicator ) b[5] |= 0x01;
    b[6]  = p->section_number;
    b[7]  = p->last_section_number;
    b[8]  = p->original_network_id>>8;
    b[9] = p->original_network_id&0xFF;
    b[10] = 0xFF;
    len = 11;
    for( i = 0; i <= p->last_section_number; i++ )
    {
        len += f_sdts( &b[len], &p->section[i]);
    }
    // length from the field after the length and including the CRC
    b[1]  |= ((len-3+CRC_32_LEN)>>8);
    b[2]   = (len-3+CRC_32_LEN)&0xFF;
    len = crc32_add( b, len );

    return len;
}
//
// Format the header and payload of the SDT packet
//
uint8_t* sdt_fmt( int stream_id, int network_id, int service_id, char *service_provider_name, char *service_name )
{
    int i,len;
    tp_hdr hdr;
    service_description_section sds;

	hdr.transport_error_indicator    = TRANSPORT_ERROR_FALSE;
    hdr.payload_unit_start_indicator = PAYLOAD_START_TRUE;
    hdr.transport_priority           = TRANSPORT_PRIORITY_LOW;
    hdr.pid                          = SDT_PID;//PAT table pid
    hdr.transport_scrambling_control = SCRAMBLING_OFF;
    hdr.adaption_field_control       = ADAPTION_PAYLOAD_ONLY;
    hdr.continuity_counter           = m_sdt_seq = 0;
    len = tp_fmt( sdt_pkt, &hdr );

    // Payload start immediately
    sdt_pkt[len++] = 0;

    // Add the payload
    sds.transport_stream_id    = stream_id;
    sds.version_number         = 2;
    sds.current_next_indicator = 1;
    sds.original_network_id    = network_id;
    sds.section_number         = 0;
    sds.last_section_number    = 0;

    // First section
    sds.section[0].service_id              = service_id;
    sds.section[0].eit_schedule_flag       = 1;
    sds.section[0].eit_present_follow_flag = 1;
    sds.section[0].running_status          = 4;
    sds.section[0].free_ca_mode            = 0;
    sds.section[0].nr_descriptors          = 1;
    // Add the descriptor to the section
    sds.section[0].descriptor[0].tag      = SI_DESC_SERVICE;
    sds.section[0].descriptor[0].sd.type  = SVC_DIGITAL_TV;

    memcpy( sds.section[0].descriptor[0].sd.provider, service_provider_name, strlen(service_provider_name));
    sds.section[0].descriptor[0].sd.provider_length = (uint8_t)strlen(service_provider_name);

    memcpy( sds.section[0].descriptor[0].sd.name, service_name, strlen(service_name));
    sds.section[0].descriptor[0].sd.name_length = (uint8_t)strlen(service_name);

    len += f_sdt( &sdt_pkt[len], &sds );
    // PAD out the unused bytes
    for( i = len; i < TP_LEN; i++ )
    {
        sdt_pkt[i] = 0xFF;
    }
    return sdt_pkt;
}
