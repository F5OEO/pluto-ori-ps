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
 *   @file          deencap.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: DEENCAPSULATION
 *
 *   @brief         GSE deencapsulation public functions definition
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef GSE_DEENCAP_H
#define GSE_DEENCAP_H

#include <stdint.h>

#include "virtual_fragment.h"
#include "deencap_header_ext.h"

struct gse_deencap_s;
typedef struct gse_deencap_s gse_deencap_t;

/**
 * @defgroup gse_deencap GSE deencapsulation API
 */

/****************************************************************************
 *
 *   FUNCTION PROTOTYPES
 *
 ****************************************************************************/

/* Deencapsulation structure management */

/**
 *  @brief   Initialize the deencapsulation structure
 *
 *  The function returns an allocated deencapsulation structure which is a
 *  table of deencapsulation contexts.
 *
 *  @param   qos_nbr   Number of qos values
 *  @param   deencap   OUT: Structure of de-encapsulation contexts on success,
 *                          NULL on error or warning
 *
 *  @return
 *                     - success/informative code among:
 *                       - \ref GSE_STATUS_OK
 *                     - warning/error code among:
 *                       - \ref GSE_STATUS_MALLOC_FAILED
 *                       - \ref GSE_STATUS_INVALID_QOS
 *                       - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_deencap
 */
gse_status_t gse_deencap_init(uint8_t qos_nbr, gse_deencap_t **deencap);

/**
 *  @brief   Release the encapsulation structure
 *
 *  @param   deencap   Structure of de-encapsulation contexts
 *
 *  @return
 *                     - success/informative code among:
 *                       - \ref GSE_STATUS_OK
 *                     - warning/error code among:
 *                       - \ref GSE_STATUS_NULL_PTR
 *                       - \ref GSE_STATUS_FRAG_NBR
 *
 *  @ingroup gse_deencap
 */
gse_status_t gse_deencap_release(gse_deencap_t *deencap);

/**
 *  @brief   Set the offsets applied on the returned virtual buffer which will
 *           contain the received PDU
 *
 *  The offsets are expressed in bytes.
 *
 *  @param   deencap       Structure of de-encapsulation contexts
 *  @param   head_offset   Offset applied on the beginning of the PDU (default: 0)
 *  @param   trail_offset  Offset applied on the end of the PDU (default: 0)
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_deencap
 */
gse_status_t gse_deencap_set_offsets(gse_deencap_t *deencap,
                                     size_t head_offset,
                                     size_t trail_offset);

/* Deencapsulation functions */

/**
 *  @brief   Deencapsulate a PDU from one or more GSE packets
 *
 *  If the complete PDU is deencapsulated, label_type, label, protocol
 *  and the PDU itself are returned, else only GSE Length is returned.
 *
 *  @warning Data is always destroyed by the function.
 *
 *  @param   data           The data containing packet to deencapsulate.
 *                          Data can contain more than one GSE packet but only
 *                          the first GSE packet will be deencapsulated
 *  @param   deencap        The deencapsulation context structure
 *  @param   label_type     OUT: The label type field value\n
 *                          Only '00' is implemented
 *  @param   label          OUT: The packet label if return code is PDU
 *  @param   protocol       OUT: The PDU protocol if return code is PDU
 *  @param   pdu            OUT: The PDU if return code is
 *                               \ref GSE_STATUS_PDU_RECEIVED,
 *                               NULL otherwise
 *  @param   packet_length  OUT: The length of the GSE packet on success
 *                                except padding detected (in bytes)
 *
 *  @return
 *                          - success/informative code among:
 *                            - \ref GSE_STATUS_OK
 *                            - \ref GSE_STATUS_PADDING_DETECTED
 *                            - \ref GSE_STATUS_DATA_OVERWRITTEN
 *                            - \ref GSE_STATUS_PDU_RECEIVED
 *                          - warning/error code among:
 *                            - \ref GSE_STATUS_NULL_PTR
 *                            - \ref GSE_STATUS_PACKET_TOO_SMALL
 *                            - \ref GSE_STATUS_INVALID_GSE_LENGTH
 *                            - \ref GSE_STATUS_EMPTY_FRAG
 *                            - \ref GSE_STATUS_FRAG_NBR
 *                            - \ref GSE_STATUS_MALLOC_FAILED
 *                            - \ref GSE_STATUS_INVALID_LT
 *                            - \ref GSE_STATUS_INTERNAL_ERROR
 *                            - \ref GSE_STATUS_INVALID_HEADER
 *                            - \ref GSE_STATUS_CRC_FRAGMENTED
 *                            - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                            - \ref GSE_STATUS_FRAG_PTRS
 *                            - \ref GSE_STATUS_EXTENSION_NOT_SUPPORTED
 *                            - \ref GSE_STATUS_INVALID_LABEL
 *                            - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                            - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                            - \ref GSE_STATUS_DATA_TOO_LONG
 *                            - \ref GSE_STATUS_INVALID_QOS
 *                            - \ref GSE_STATUS_CTX_NOT_INIT
 *                            - \ref GSE_STATUS_NO_SPACE_IN_BUFF
 *                            - \ref GSE_STATUS_INVALID_DATA_LENGTH
 *                            - \ref GSE_STATUS_INVALID_CRC
 *                            - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_deencap
 */
gse_status_t gse_deencap_packet(gse_vfrag_t *data, gse_deencap_t *deencap,
                                uint8_t *label_type, uint8_t label[6],
                                uint16_t *protocol, gse_vfrag_t **pdu,
                                uint16_t *packet_length);

/**
 *  @brief   Signal that a new BB Frame has been received
 *
 *  This function allows to detect timeouts on reception contexts.
 *  A timeout is detected when 256 BB Frames has been received without getting
 *  a complete PDU in the context.\n
 *  If this function is not implemented, timeouts will not be detected but this
 *  won't have a dramatic incident because context will be overwritten if
 *  necessary.
 *
 *  @param   deencap       The deencapsulation context structure
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_deencap
 */
gse_status_t gse_deencap_new_bbframe(gse_deencap_t *deencap);

/**
 *  @brief  Set the callback that read header extensions
 *
 *  @param  encap     The deencapsulation context
 *  @param  callback  The callback
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_ext
 */
gse_status_t gse_deencap_set_extension_callback(gse_deencap_t *deencap,
                                                gse_deencap_read_header_ext_cb_t callback,
                                                void *opaque);

#endif
