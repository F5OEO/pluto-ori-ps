/****************************************************************************/
/**
 *   @file          deencap_header_ext.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: DEENCAPSULATION CONTEXT
 *
 *   @brief         GSE public functions for header extensions deencapsulation
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef GSE_DEENCAP_EXT_H
#define GSE_DEENCAP_EXT_H

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
  * @brief   Callback used to read GSE header extensions
  *
  * @param   ext            The beginning of the header extensions
  * @param   length         IN: The maximum available length for the extension
  *                         OUT: The extensions length
  *                              0 if there is no extension
  * @param   protocol_type  OUT: The type of PDU (this shall be the Type field of
  *                              the last header extension)
  * @param   extension_type The type of extension (this is the value of the
  *                         protocol_type field of the GSE header)
  * @param   opaque         The user specific data
  *
  * @return                 0 on success, -1 on failure
  *
  * @ingroup gse_ext
  */
typedef int (*gse_deencap_read_header_ext_cb_t)(unsigned char *ext,
                                                size_t *length,
                                                uint16_t *protocol_type,
                                                uint16_t extension_type,
                                                void *opaque);

/**
 *  @brief   Read header extensions from a GSE packet
 *
 *  @param  packet     The GSE packet without extension
 *  @param  callback   The callback used to build extensions
 *  @param  opaque     The user specific data used by the callback
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                           - \ref GSE_STATUS_EXTENSION_UNAVAILABLE
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_INTERNAL_ERROR;
 *                           - \ref GSE_STATUS_INVALID_LT
 *                           - \ref GSE_STATUS_EXTENSION_CB_FAILED
 *
 *  @ingroup gse_ext
 */
gse_status_t gse_deencap_get_header_ext(unsigned char *packet,
                                        gse_deencap_read_header_ext_cb_t callback,
                                        void *opaque);


#endif
