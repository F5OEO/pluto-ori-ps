/****************************************************************************/
/**
 *   @file          encap_header_ext.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: ENCAPSULATION CONTEXT
 *
 *   @brief         GSE public functions for header extensions encapsulation
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef GSE_ENCAP_EXT_H
#define GSE_ENCAP_EXT_H

#include <stdint.h>

#include "virtual_fragment.h"
#include "status.h"
#include "constants.h"

/**
 * @defgroup gse_ext GSE header extensions API
 */

/****************************************************************************
 *
 *   FUNCTION PROTOTYPES
 *
 ****************************************************************************/

/**
 *  @brief   Callback used to build GSE header extensions
 *
 *  @param   ext            The beginning of the header extensions
 *  @param   length         IN: The maximum available space for the extension
 *                          OUT: The extensions length
 *                               0 if there is no extension
 *  @param   extension_type OUT: The type of extension (this will be the value
 *                               of the protocol_type field of the GSE header)
 *                               protocol_type if there is no extension
 *  @param   protocol_type  The type of PDU (this shall be the Type field of
 *                          the last header extension)
 *  @param   opaque         The user specific data
 *
 *  @return                 0 on success, -1 on failure
 *
 *  @ingroup gse_ext
 */
typedef int (*gse_encap_build_header_ext_cb_t)(unsigned char *ext,
                                               size_t *length,
                                               uint16_t *extension_type,
                                               uint16_t protocol_type,
                                               void *opaque);

/**
 *  @brief   Add header extensions to a GSE packet
 *
 *  Extensions can be append only on complete PDU, in the case of first fragment
 *  the CRC will be modified and we don't have access on it here, thus the
 *  temporary CRC is returned and gse_ext_update_crc should be called on
 *  subsequent fragments.\n\
 *  The packet can be fragmented if the PDU is too long, thus frag will be
 *  returned.\n\
 *  packet internal buffer can be reallocated if it is too small.\n\
 *  At least head_offset and trail_offset are applied on packet and on frag.\n\
 *  The idea to avoid reallocation could be to decrease the initial head_offset
 *  (the value applied when packet was created) from length extension,
 *  thus the header could be moved in the virtual fragment\n\
 *
 *  @warning In case of error, if frag is NULL the data contained in the packet
 *           given as parameter are reinitialized but the virtual buffer
 *           properties could have changed. If frag is not NULL, the packet was
 *           fragmented into packet and frag but data are still valid
 *
 *
 *  @param  packet          IN: The GSE packet without extension
 *                          OUT: The GSE packet with extensions
 *                          (the vfrag can be reallocated if necessary)
 *                          (it is a fragment if frag != NULL)
 *  @param  frag            OUT: A GSE fragment if packet was fragmented in order
 *                               to append the extension, NULL otherwise
 *  @param  crc             OUT: the temporary CRC if packet was a first fragment
 *                               (i.e. if GSE_STATUS_PARTIAL_CRC is returned)
 *                               you should use the \ref gse_encap_update_crc
 *                               function on subsequent fragments
 *  @param  callback        The callback used to build extensions
 *  @param  max_length      Maximum length of the GSE packet (will be set to
 *                          packet length if smaller, 0 for default)
 *  @param  head_offset     The header offset to apply on packet and on frag
 *  @param  trail_offset    The trailer offset to apply on packet and on frag
 *  @param  qos             The qos associated to the desired GSE packet
 *  @param  opaque          The user specific data used by the callback
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                           - \ref GSE_STATUS_EXTENSION_UNAVAILABLE
 *                           - \ref GSE_STATUS_PARTIAL_CRC
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_LENGTH_TOO_HIGH
 *                           - \ref GSE_STATUS_LENGTH_TOO_SMALL
 *                           - \ref GSE_STATUS_REFRAG_UNNECESSARY
 *                           - \ref GSE_STATUS_INVALID_GSE_LENGTH
 *                           - \ref GSE_STATUS_INVALID_LT
 *                           - \ref GSE_STATUS_INTERNAL_ERROR;
 *                           - \ref GSE_STATUS_INVALID_HEADER
 *                           - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                           - \ref GSE_STATUS_FRAG_PTRS
 *                           - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                           - \ref GSE_STATUS_MALLOC_FAILED
 *                           - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                           - \ref GSE_STATUS_DATA_TOO_LONG
 *                           - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                           - \ref GSE_STATUS_INVALID_QOS
 *                           - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_ext
 */
gse_status_t gse_encap_add_header_ext(gse_vfrag_t *packet,
                                      gse_vfrag_t **frag,
                                      uint32_t *crc,
                                      gse_encap_build_header_ext_cb_t callback,
                                      size_t max_length,
                                      size_t head_offset,
                                      size_t trail_offset,
                                      uint8_t qos,
                                      void *opaque);

/**
 *  @brief   Update the CRC for each fragment of context and overwrite the CRC of
 *           the last fragment
 *
 *
 *  @param  packet          IN: The GSE packet
 *                          OUT: The GSE packet or the GSE packet with the
 *                               new CRC in the case of a last fragment
 *  @param  crc             IN: The initial CRC
 *                              (returned by \ref gse_encap_add_header_ext or
 *                               by the previous call to this function)
 *                          OUT: the temporary CRC if packet was not a last fragment
 *                               (i.e. if GSE_STATUS_PARTIAL_CRC is returned)
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                           - \ref GSE_STATUS_PARTIAL_CRC
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *
 *  @ingroup gse_ext
 */
gse_status_t gse_encap_update_crc(gse_vfrag_t *packet,
                                  uint32_t *crc);


#endif
