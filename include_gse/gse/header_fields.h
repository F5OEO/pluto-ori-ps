/*
 *
 * This piece of software is an implementation of the Generic Stream
 * Encapsulation (GSE) standard defined by ETSI for Linux (or other
 * Unix-compatible OS). The library may be used to add GSE
 * encapsulation/de-encapsulation capabilities to an application.
 *
 *
 * Copyright © 2016 TAS
 *
 *
 * This file is part of the GSE library.
 *
 *
 * The GSE library is free software : you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY, without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 */

/****************************************************************************/
/**
 *   @file          header_fields.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: COMMON
 *
 *   @brief         Function related to header fields
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef HEADER_FIELD_H
#define HEADER_FIELD_H

#include <stdint.h>
#include <endian.h>
#include <stdbool.h>

#include "virtual_fragment.h"

/**
 * @defgroup gse_head_access GSE header fields access API
 */

/****************************************************************************
 *
 *   STRUCTURES AND TYPES
 *
 ****************************************************************************/

/**> The extension header Type field */
typedef struct
{
#if __BYTE_ORDER == __BIG_ENDIAN
  uint8_t null_1:4; /**< unused bits */
  uint8_t null_2:1; /**< unused bits */
  uint8_t h_len:3;  /**< H-LEN field */
  uint8_t h_type;   /**< H-TYPE field */
#elif __BYTE_ORDER == __LITTLE_ENDIAN
  uint8_t h_len:3;  /**< H-LEN field */
  uint8_t null_2:1; /**< unused bits */
  uint8_t null_1:4; /**< unused bits */
  uint8_t h_type;   /**< H-TYPE field */
#else
#error "Please fix <bits/endian.h>"
#endif
} __attribute__((packed)) gse_ext_type_t;


/**
 * @brief The different values of extensions headers or LLC
 *
 * GSE extension headers work the same way as ULE extension headers. Protocol
 * types below 1536 are either extension headers or LLC. The list of extension
 * headers and LLC is maintained by the IANA.
 *
 * @see https://tools.ietf.org/html/rfc4326#section-4.4
 * @see https://www.iana.org/assignments/ule-next-headers/ule-next-headers.xhtml
 */
enum gse_ext_type
{
	GSE_EXT_TEST_SNDU          = 0,    /**< RFC4326 */
	GSE_EXT_BRIDGED_SNDU       = 1,    /**< RFC4326 */
	GSE_EXT_TS_CONCAT          = 2,    /**< RFC5163 */
	GSE_EXT_PDU_CONCAT         = 3,    /**< RFC5163 */
	/* 4-128 	Unassigned */
	GSE_EXT_LL_GSE_NCR         = 129,  /**< EN 301 545-2 [Hans-Peter_Lexow] */
	GSE_EXT_LL_RCS_L2S         = 130,  /**< EN 301 545-2 [Hans-Peter_Lexow] */
	GSE_EXT_LL_RCS_DCP         = 131,  /**< TS 101 545-3 [Hans-Peter_Lexow] */
	GSE_EXT_LL_RCS_1           = 132,  /**< EN 301 545-2 [Gorry_Fairhurst] */
	GSE_EXT_LL_RCS_TRANSEC_SYS = 133,  /**< EN 301 545-1 [Gorry_Fairhurst] */
	GSE_EXT_LL_RCS_TRANSEC_PAY = 134,  /**< EN 301 545-1 [Gorry_Fairhurst] */
	GSE_EXT_DVB_GSE_LLC        = 135,  /**< ETSI TS 102 606-2 [Alexander_Adolf] */
	/* 136-143 	Unassigned */
	/* 144-159 	Reserved for Private Use 	[RFC7280] */
	/* 160-199 	Unassigned */
	GSE_EXT_LL_RCS_FEC_FDT     = 200,  /**< EN 301 790 V1.5.1 [Laurence_Duquerroy] */
	/* 201-255 	Unassigned */
	GSE_EXT_PADDING            = 256,  /**< RFC4326 */
	GSE_EXT_TIMESTAMP          = 257,  /**< RFC5163 */
	/* 258-449 	Unassigned */
	GSE_EXT_LL_RCS_FEC_ADT     = 450,  /**< EN 301 790 V1.5.1 [Laurence_Duquerroy] */
	GSE_EXT_LL_CRC32           = 451,  /**< EN 301 790 V1.5.1 [Laurence_Duquerroy] */
	/* 452-511 	Unassigned */
};

/****************************************************************************
 *
 *   FUNCTIONS
 *
 ****************************************************************************/

/**
 *  @brief   Get the GSE packet Start Indicator field value
 *
 *  @param   packet            a pointer to the beginning of the GSE packet
 *  @param   start_indicator   OUT: the Start Indicator value on success,
 *                                  otherwise the value is not reliable
 *
 *  @return
 *                             - success/informative code among:
 *                               - \ref GSE_STATUS_OK
 *                             - warning/error code among:
 *                               - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_start_indicator(unsigned char *packet,
                                     uint8_t *start_indicator);

/**
 *  @brief   Get the GSE packet End Indicator field value
 *
 *  @param   packet          a pointer to the beginning of the GSE packet
 *  @param   end_indicator   OUT: the End Indicator value on success,
 *                                otherwise the value is not reliable
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_end_indicator(unsigned char *packet,
                                   uint8_t *end_indicator);

/**
 *  @brief   Get the GSE packet Label Type field value
 *
 *  @param   packet       a pointer to the beginning of the GSE packet
 *  @param   label_type   OUT: the Label Type value on success,
 *                             otherwise the value is not reliable
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_label_type(unsigned char *packet, uint8_t *label_type);

/**
 *  @brief   Get the GSE packet GSE Length field value
 *
 *  @param   packet       a pointer to the beginning of the GSE packet
 *  @param   gse_length   OUT: the GSE Length value on success,
 *                             otherwise the value is not reliable
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_gse_length(unsigned char *packet, uint16_t *gse_length);

/**
 *  @brief   Get the GSE packet Frag ID field value
 *
 *  @param   packet    a pointer to the beginning of the GSE packet
 *  @param   frag_id   OUT: the Frag Id value on success,
 *                          otherwise the value is not reliable
 *
 *  @return
 *                     - success/informative code among:
 *                       - \ref GSE_STATUS_OK
 *                       - \ref GSE_STATUS_FIELD_ABSENT
 *                     - warning/error code among:
 *                       - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_frag_id(unsigned char *packet, uint8_t *frag_id);

/**
 *  @brief   Get the GSE packet Total Length field value
 *
 *  @param   packet         a pointer to the beginning of the GSE packet
 *  @param   total_length   OUT: the Total Length value on success,
 *                               otherwise the value is not reliable
 *
 *  @return
 *                          - success/informative code among:
 *                            - \ref GSE_STATUS_OK
 *                            - \ref GSE_STATUS_FIELD_ABSENT
 *                          - warning/error code among:
 *                            - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_total_length(unsigned char *packet, uint16_t *total_length);

/**
 *  @brief   Get the GSE packet Protocol Type field value
 *
 *  @param   packet          a pointer to the beginning of the GSE packet
 *  @param   protocol_type   OUT: the Protocol Type value on success,
 *                                otherwise the value is not reliable
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                             - \ref GSE_STATUS_FIELD_ABSENT
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_protocol_type(unsigned char *packet,
                                   uint16_t *protocol_type);

/**
 *  @brief   Get the GSE packet Label field value
 *
 *  @param   packet  a pointer to the beginning of the GSE packet
 *  @param   label   OUT: the Label value on success,
 *                        otherwise the value is not reliable
 *  Be careful, get the label length before exploiting the returned value
 *
 *  @return
 *                   - success/informative code among:
 *                     - \ref GSE_STATUS_OK
 *                     - \ref GSE_STATUS_FIELD_ABSENT
 *                   - warning/error code among:
 *                     - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_get_label(unsigned char *packet, uint8_t label[6]);

/**
 *  @brief   Check header extensions validity and get the last type field
 *
 *  @param   extension      The extensions data
 *  @param   ext_length     IN: at least the extensions length
 *                          OUT: the real extension length 
 *  @param   extension_type The type of the first extension
 *  @param   protocol_type  OUT: The protocol type carried by the last extension
 *                               Type field
 *
 *  @return
 *                   - success/informative code among:
 *                     - \ref GSE_STATUS_OK
 *                   - warning/error code among:
 *                     - \ref GSE_STATUS_NULL_PTR
 *                     - \ref GSE_STATUS_INVALID_EXTENSIONS
 *
 *  @ingroup gse_head_access
 */
gse_status_t gse_check_header_extension_validity(unsigned char *extension,
                                                 size_t *ext_length,
                                                 uint16_t extension_type,
                                                 uint16_t *protocol_type);

/**
 *  @brief   Whether the protocol type is LLC or not
 *
 *  @param   protocol_type The type of the protocol
 *
 *  @return  true if protocol is some LLC, false otherwise
 *
 *  @ingroup gse_head_access
 */
bool gse_is_llc(const uint16_t protocol_type)
	__attribute__((warn_unused_result));

/**
 *  @brief   Whether the protocol type is some extension header or not
 *
 *  @param   protocol_type The type of the protocol
 *
 *  @return  true if protocol is some extension header, false otherwise
 *
 *  @ingroup gse_head_access
 */
bool gse_is_ext_hdr(const uint16_t protocol_type)
	__attribute__((warn_unused_result));

#endif
