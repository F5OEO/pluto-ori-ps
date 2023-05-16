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
 *   @file          encap.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: ENCAPSULATION CONTEXT
 *
 *   @brief         GSE encapsulation public functions definition
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *   @author        Audric Schiltknecht / Viveris Technologies
 *   @author        Joaquin Muguerza / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef GSE_ENCAP_H
#define GSE_ENCAP_H

#include <stdint.h>

#include "virtual_fragment.h"
#include "encap_header_ext.h"

struct gse_encap_s;
/** Encapsulation structure type definition */
typedef struct gse_encap_s gse_encap_t;

/**
 * @defgroup gse_encap GSE encapsulation API
 */

/****************************************************************************
 *
 *   FUNCTION PROTOTYPES
 *
 ****************************************************************************/

/* Encapsulation initialization and release functions */

/**
 *  @brief   Initialize the encapsulation structure
 *
 *  @param   qos_nbr        number of qos values
 *  @param   fifo_size      size of FIFOs
 *  @param   encap          OUT: Encapsulation structure on success,
 *                               NULL on error or warning
 *
 *  @return
 *                          - success/informative code among:
 *                            - \ref GSE_STATUS_OK
 *                          - warning/error code among:
 *                            - \ref GSE_STATUS_NULL_PTR
 *                            - \ref GSE_STATUS_QOS_NBR_NULL
 *                            - \ref GSE_STATUS_FIFO_SIZE_NULL
 *                            - \ref GSE_STATUS_MALLOC_FAILED
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_init(uint8_t qos_nbr, size_t fifo_size,
                            gse_encap_t **encap);

/**
 *  @brief   Release the encapsulation structure
 *
 *  @param   encap   Encapsulation structure
 *
 *  @return
 *                   - success/informative code among:
 *                     - \ref GSE_STATUS_OK
 *                   - warning/error code among:
 *                     - \ref GSE_STATUS_NULL_PTR
 *                     - \ref GSE_STATUS_PTHREAD_MUTEX
 *                     - \ref GSE_STATUS_FRAG_NBR
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_release(gse_encap_t *encap);

/**
 *  @brief   Set the offsets applied on each GSE packet (for usage with copy only)
 *
 *  The offsets are expressed in bytes.
 *  In case of warning or error, the encapsulation context is unchanged.
 *
 *  @param   encap         Encapsulation structure
 *  @param   head_offset   Offset applied on the beginning of each GSE packet
 *  @param   trail_offset  Offset applied on the end of each GSE packet
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_set_offsets(gse_encap_t *encap,
                                   size_t head_offset, size_t trail_offset);

/* Encapsulation functions */

/**
 *  @brief   Receive a PDU which is stored in a virtual buffer
 *
 *
 *  In the zero copy case the header offset of the virtual fragment (PDU)
 *  should be large enough for user offset (no GSE data) + header length +
 *  extensions
 *
 *  @warning In case of warning or error, the PDU is destroyed.
 *
 *  @param   pdu            The PDU to encapsulate
 *  @param   encap          The encapsulation context structure
 *  @param   label          The packet label
 *  @param   label_type     The label type field value
 *                          only '00' is implemented
 *  @param   protocol       The PDU protocol
 *  @param   qos            The QoS value of the PDU
 *
 *  @return
 *                          - success/informative code among:
 *                            - \ref GSE_STATUS_OK
 *                          - warning/error code among:
 *                            - \ref GSE_STATUS_NULL_PTR
 *                            - \ref GSE_STATUS_INVALID_LT
 *                            - \ref GSE_STATUS_PDU_LENGTH
 *                            - \ref GSE_STATUS_INVALID_QOS
 *                            - \ref GSE_STATUS_PTHREAD_MUTEX
 *                            - \ref GSE_STATUS_FIFO_FULL
 *                            - \ref GSE_STATUS_WRONG_PROTOCOL
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_receive_pdu(gse_vfrag_t *pdu, gse_encap_t *encap,
                                   uint8_t label[6], uint8_t label_type,
                                   uint16_t protocol, uint8_t qos);

/**
 *  @brief   Get a GSE packet from the encapsulation context structure
 *
 *  This function should not be called if the previous GSE packet has not
 *  been destroyed (with gse_free_vfrag) except for the first packet.
 *
 *  @param   packet          OUT: The GSE packet on success,
 *                                NULL on error or warning
 *  @param   encap           The encapsulation context structure
 *  @param   desired_length  The desired length for the packet (in bytes)
 *  @param   qos             The QoS of the packet
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *                             - \ref GSE_STATUS_INVALID_QOS
 *                             - \ref GSE_STATUS_FIFO_EMPTY
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_LENGTH_TOO_HIGH
 *                             - \ref GSE_STATUS_LENGTH_TOO_SMALL
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_INTERNAL_ERROR;
 *                             - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                             - \ref GSE_STATUS_FRAG_PTRS
 *                             - \ref GSE_STATUS_INVALID_GSE_LENGTH
 *                             - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                             - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                             - \ref GSE_STATUS_DATA_TOO_LONG
 *                             - \ref GSE_STATUS_MALLOC_FAILED
 *                             - \ref GSE_STATUS_EMPTY_FRAG
 *                             - \ref GSE_STATUS_FRAG_NBR
 *                             - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_get_packet(gse_vfrag_t **packet, gse_encap_t *encap,
                                  size_t desired_length, uint8_t qos);

/**
 *  @brief   Get a GSE packet from the encapsulation context structure
 *
 *  This function does not use zero copy strategy. Thus, it could be called
 *  without destroying the previous GSE packets.
 *
 *  @param   packet          OUT: The GSE packet on success,
 *                                NULL on error or warning
 *  @param   encap           The encapsulation context structure related
 *  @param   desired_length  The desired length for the packet (in bytes)
 *  @param   qos             The QoS of the packet
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                             - \ref GSE_STATUS_FIFO_EMPTY
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *                             - \ref GSE_STATUS_INVALID_QOS
 *                             - \ref GSE_STATUS_FIFO_EMPTY
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_LENGTH_TOO_HIGH
 *                             - \ref GSE_STATUS_LENGTH_TOO_SMALL
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_INTERNAL_ERROR;
 *                             - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                             - \ref GSE_STATUS_FRAG_PTRS
 *                             - \ref GSE_STATUS_INVALID_GSE_LENGTH
 *                             - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                             - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                             - \ref GSE_STATUS_DATA_TOO_LONG
 *                             - \ref GSE_STATUS_MALLOC_FAILED
 *                             - \ref GSE_STATUS_EMPTY_FRAG
 *                             - \ref GSE_STATUS_FRAG_NBR
 *                             - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_get_packet_copy(gse_vfrag_t **packet, gse_encap_t *encap,
                                       size_t desired_length, uint8_t qos);

/**
 *  @brief   Get a GSE packet from the encapsulation context structure -
 *           No allocation mode
 *
 *  This function should not be called if the previous GSE packet has not
 *  been destroyed (with gse_free_vfrag_no_alloc) except for the first packet.
 *
 *  @param   packet          OUT: The GSE packet on success,
 *                                NULL on error or warning
 *  @param   encap           The encapsulation context structure
 *  @param   desired_length  The desired length for the packet (in bytes)
 *  @param   qos             The QoS of the packet
 *
 *  @return
 *                           - success/informative code among:
 *                             - \ref GSE_STATUS_OK
 *                           - warning/error code among:
 *                             - \ref GSE_STATUS_NULL_PTR
 *                             - \ref GSE_STATUS_INVALID_QOS
 *                             - \ref GSE_STATUS_FIFO_EMPTY
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_LENGTH_TOO_HIGH
 *                             - \ref GSE_STATUS_LENGTH_TOO_SMALL
 *                             - \ref GSE_STATUS_PTHREAD_MUTEX
 *                             - \ref GSE_STATUS_INTERNAL_ERROR;
 *                             - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                             - \ref GSE_STATUS_FRAG_PTRS
 *                             - \ref GSE_STATUS_INVALID_GSE_LENGTH
 *                             - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                             - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                             - \ref GSE_STATUS_DATA_TOO_LONG
 *                             - \ref GSE_STATUS_MALLOC_FAILED
 *                             - \ref GSE_STATUS_EMPTY_FRAG
 *                             - \ref GSE_STATUS_FRAG_NBR
 *                             - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_encap
 */
gse_status_t gse_encap_get_packet_no_alloc(gse_vfrag_t **packet, gse_encap_t *encap,
                                           size_t desired_length, uint8_t qos);
gse_status_t gse_encap_get_packet_no_alloc_old(gse_vfrag_t *packet, gse_encap_t *encap,
                                           size_t desired_length, uint8_t qos);

/**
 *  @brief  Set the callback that build header extensions
 *
 *  @param  encap     The encapsulation context
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
gse_status_t gse_encap_set_extension_callback(gse_encap_t *encap,
                                              gse_encap_build_header_ext_cb_t callback,
                                              void *opaque);

#endif
