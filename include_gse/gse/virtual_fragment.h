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
 *   @file          virtual_fragment.h
 *
 *          Project:     GSE LIBRARY
 *
 *          Company:     THALES ALENIA SPACE
 *
 *          Module name: VIRTUAL FRAGMENT
 *
 *   @brief         Prototypes of elements used by the virtual fragment
 *
 *   @author        Julien BERNARD / Viveris Technologies
 *   @author        Audric Schiltknecht / Viveris Technologies
 *   @author        Joaquin Muguerza / Viveris Technologies
 *
 */
/****************************************************************************/


#ifndef VIRTUAL_FRAGMENT_H
#define VIRTUAL_FRAGMENT_H

#include <string.h>

#include "status.h"

/** Get the minimum between two values */
#define MIN(x, y)  (((x) < (y)) ? (x) : (y))

/**
 * @defgroup gse_virtual_fragment GSE virtual fragment API
 */

/****************************************************************************
 *
 *   STRUCTURES AND TYPES
 *
 ****************************************************************************/

/** Virtual buffer */
typedef struct
{
  unsigned char *start; /**< Point on the beginning of the virtual buffer */
  unsigned char *end;   /**< Point on the end of the virtual buffer */
  size_t length;        /**< Length of the virtual buffer (in bytes)*/
  unsigned int vfrag_count; /**< Number of virtual fragments
                                 This value should not be greater than 2 */
} gse_vbuf_t;

/** Virtual fragment: represent a subpart of a virtual buffer */
typedef struct
{
  gse_vbuf_t *vbuf;     /**< The virtual buffer to which the fragment belongs */
  unsigned char *start; /**< Point on the beginning of the virtual fragment */
  unsigned char *end;   /**< Point on the end of the virtual fragment */
  size_t length;        /**< length of the virtual fragment (in bytes)*/
} gse_vfrag_t;

/****************************************************************************
 *
 *   FUNCTION PROTOTYPES
 *
 ****************************************************************************/

/**
 *  @brief   Create an empty virtual fragment
 *
 *  The length of the virtual buffer containing the fragment will be
 *  max_length + head_offset + trail_offset.\n
 *  All length are expressed in bytes.\n
 *  For a GSE encapsulation usage, the header offset should at least be the
 *  maximum header length + maximum extension length and the trailer offset
 *  should at least be the CRC length.\n
 *
 *  @param   vfrag         OUT: The virtual fragment on success,
 *                              NULL on error
 *  @param   max_length    The maximum length of the fragment
 *  @param   head_offset   The offset applied before the fragment
 *  @param   trail_offset  The offset applied after the fragment
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                           - \ref GSE_STATUS_MALLOC_FAILED
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_create_vfrag(gse_vfrag_t **vfrag, size_t max_length,
                              size_t head_offset, size_t trail_offset);

/**
 *  @brief   Create a virtual fragment containing data
 *
 *  The length of the virtual buffer containing the fragment will be
 *  max_length + head_offset + trail_offset.\n
 *  All length are expressed in bytes.\n
 *  For a GSE encapsulation usage, the header offset should at least be the
 *  maximum header length and the trailer offset should at least be the CRC
 *  length.\n
 *
 *  @param   vfrag        OUT: The virtual fragment on success,
 *                             NULL on error
 *  @param   max_length   The maximum length of the fragment
 *  @param   head_offset  The offset applied before the fragment
 *  @param   trail_offset The offset applied after the fragment
 *  @param   data         The data to write in the virtual fragment
 *  @param   data_length  The length of the data
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                          - \ref GSE_STATUS_MALLOC_FAILED
 *                          - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                          - \ref GSE_STATUS_DATA_TOO_LONG
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_create_vfrag_with_data(gse_vfrag_t **vfrag, size_t max_length,
                                        size_t head_offset, size_t trail_offset,
                                        unsigned char const *data,
                                        size_t data_length);

/**
 *  @brief   Copy data in a virtual fragment
 *
 *  In case of warning or error, the virtual fragment is unchanged.
 *
 *  @param   vfrag       The virtual fragment
 *  @param   data        The data to write in the virtual fragment
 *  @param   data_length The length of the data (in bytes)
 *
 *  @return
 *                       - success/informative code among:
 *                         - \ref GSE_STATUS_OK
 *                       - warning/error code among:
 *                         - \ref GSE_STATUS_NULL_PTR
 *                         - \ref GSE_STATUS_MULTIPLE_VBUF_ACCESS
 *                         - \ref GSE_STATUS_DATA_TOO_LONG
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_copy_data(gse_vfrag_t *vfrag, unsigned char const* data,
                           size_t data_length);

/**
 *  @brief   Transform a buffer into a virtual fragment
 *
 *  The virtual buffer containing the fragment will contain the allocated
 *  memory of the buffer.\n
 *  The allocated space in the buffer must at least be
 *  max_length + head_offset + trail_offset.\n
 *  All length are expressed in bytes.\n
 *
 *  @param   vfrag        OUT: The virtual fragment on success,
 *                             NULL on error
 *  @param   buffer       The buffer to transform
 *  @param   head_offset  The offset applied before the data in the buffer
 *  @param   trail_offset The offset applied after the data in the buffer
 *  @param   data_length  The length of the data in the buffer
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_MALLOC_FAILED
 *                          - \ref GSE_STATUS_INTERNAL_ERROR
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_create_vfrag_from_buf(gse_vfrag_t **vfrag, unsigned char *buffer,
                                       unsigned int head_offset, unsigned int trail_offset,
                                       unsigned int data_length);
/**
 *  @brief   Create an empty virtual fragment - No allocation mode
 *
 *  @param   vfrag         OUT: The virtual fragment on success,
 *                              NULL on error
 *  @param   alloc_vbuf    Whether to allocate the virtual buffer container or
 *                         not (only the container, does not allocate the buffer
 *                         itself).
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                           - \ref GSE_STATUS_MALLOC_FAILED
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_allocate_vfrag(gse_vfrag_t **vfrag, int alloc_vbuf);

/**
 *  @brief   Affect a buffer to an allocated virtual fragment - No allocation
 *  mode
 *
 *  The virtual buffer containing the fragment will contain the allocated
 *  memory of the buffer.\n
 *  The allocated space in the buffer must at least be
 *  max_length + head_offset + trail_offset.\n
 *  All length are expressed in bytes.\n
 *
 *  @param   vfrag        OUT: The virtual fragment on success,
 *                             NULL on error
 *  @param   buffer       The buffer to transform
 *  @param   head_offset  The offset applied before the data in the buffer
 *  @param   trail_offset The offset applied after the data in the buffer
 *  @param   data_length  The length of the data in the buffer
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_MALLOC_FAILED
 *                          - \ref GSE_STATUS_INTERNAL_ERROR
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_affect_buf_vfrag(gse_vfrag_t *vfrag, unsigned char *buffer,
                                  unsigned int head_offset, unsigned int trail_offset,
                                  unsigned int data_length);

/**
 *  @brief   Free a virtual fragment
 *
 *  @param   vfrag         IN: The virtual fragment that will be destroyed
 *                         OUT: NULL
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_FRAG_NBR
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_free_vfrag(gse_vfrag_t **vfrag);

/**
 *  @brief   Free a virtual fragment - No allocation mode
 *
 *  @param   vfrag         IN: The virtual fragment that will be destroyed
 *                         OUT: NULL
 *  @param   reset         If true, only reset the virtual fragment, do not free
 *                         allocated memory
 *  @param   free_vbuf     If true, free allocated memory for the virtual
 *                         buffer.
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_FRAG_NBR
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_free_vfrag_no_alloc(gse_vfrag_t **vfrag, int reset, int free_vbuf);

/**
 *  @brief   Create a virtual fragment from an existing one
 *
 *  In case of warning or error, the virtual fragment is unchanged.
 *  @warning If the father length is smaller than the wanted length, the length
 *           of the duplicated fragment will be the father length.
 *
 *  @param   vfrag        The duplicated virtual fragment
 *  @param   father       The virtual fragment which will be duplicated
 *  @param   length       The length of the duplicated virtual fragment (in bytes)
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_EMPTY_FRAG
 *                          - \ref GSE_STATUS_FRAG_NBR
 *                          - \ref GSE_STATUS_MALLOC_FAILED
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_duplicate_vfrag(gse_vfrag_t **vfrag, gse_vfrag_t *father, size_t length);

/**
 *  @brief   Create a virtual fragment from an existing one - No allocation mode
 *
 *  In case of warning or error, the virtual fragment is unchanged.
 *  @warning If the father length is smaller than the wanted length, the length
 *           of the duplicated fragment will be the father length.
 *
 *  @param   vfrag        The duplicated virtual fragment
 *  @param   father       The virtual fragment which will be duplicated
 *  @param   length       The length of the duplicated virtual fragment (in bytes)
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_EMPTY_FRAG
 *                          - \ref GSE_STATUS_FRAG_NBR
 *                          - \ref GSE_STATUS_MALLOC_FAILED
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_duplicate_vfrag_no_alloc(gse_vfrag_t **vfrag,
                                          gse_vfrag_t *father,
                                          size_t length);

/**
 *  @brief   Shift the virtual fragment
 *
 *  All length are expressed in bytes.
 *  In case of warning or error, the virtual fragment is unchanged.
 *
 *  @param   vfrag        The virtual fragment
 *  @param   start_shift  The shift value to apply on the beginning of the
 *                        fragment
 *  @param   end_shift    The shift value to apply on the end of the fragment
 *
 *  @return
 *                        - success/informative code among:
 *                          - \ref GSE_STATUS_OK
 *                        - warning/error code among:
 *                          - \ref GSE_STATUS_NULL_PTR
 *                          - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *                          - \ref GSE_STATUS_FRAG_PTRS
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_shift_vfrag(gse_vfrag_t *vfrag, int start_shift, int end_shift);

/**
 *  @brief   Reset a virtual fragment to its created state
 *
 *  All length are expressed in bytes.
 *  In case of warning or error, the virtual fragment is unchanged.
 *
 *  @param   vfrag         The virtual fragment
 *  @param   length        OUT: The length of the fragment (can eventually be 0)
 *  @param   head_offset   The offset applied before the fragment
 *  @param   trail_offset  The offset applied after the fragment
 *
 *  @return                success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_OFFSET_TOO_HIGH
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_reset_vfrag(gse_vfrag_t *vfrag, size_t *length,
                             size_t head_offset, size_t trail_offset);

/**
 *  @brief   Get the pointer on the beginning of a virtual fragment
 *
 *  In case of warning or error, the virtual fragment is unchanged.
 *
 *  @param   vfrag  Virtual fragment
 *
 *  @return         A pointer on the start of the virtual fragment on success,
 *                  NULL on failure
 *
 *  @ingroup gse_virtual_fragment
 */
unsigned char *gse_get_vfrag_start(gse_vfrag_t *vfrag);

/**
 *  @brief   Get the length of a virtual fragment (in bytes)
 *
 *  Check if vfrag is not NULL before using this function.
 *
 *  @param   vfrag  Virtual fragment
 *
 *  @return         The length of the virtual fragment
 *
 *  @ingroup gse_virtual_fragment
 */
size_t gse_get_vfrag_length(gse_vfrag_t *vfrag);

/**
 *  @brief   Set the length of a virtual fragment (in bytes)
 *
 *  In case of warning or error, the virtual fragment is unchanged.
 *
 *  @param   vfrag  The virtual fragment
 *  @param   length The length of the data that were put in the fragment
 *
 *  @return
 *                  - success/informative code among:
 *                    - \ref GSE_STATUS_OK
 *                  - warning/error code among:
 *                    - \ref GSE_STATUS_NULL_PTR
 *                    - \ref GSE_STATUS_PTR_OUTSIDE_BUFF
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_set_vfrag_length(gse_vfrag_t *vfrag, size_t length);

/**
 *  @brief   Get the length available in buffer before virtual fragment (in bytes)
 *
 *  @param   vfrag  The virtual fragment
 *
 *  @return         The header offset
 */
size_t gse_get_vfrag_available_head(gse_vfrag_t *vfrag);

/**
 *  @brief   Get the length available in buffer after virtual fragment (in bytes)
 *
 *  @param   vfrag  The virtual fragment
 *
 *  @return         The trailer offset
 */
size_t gse_get_vfrag_available_trail(gse_vfrag_t *vfrag);

/**
 *  @brief   Reallocate a virtual fragment internal buffer to increase
 *           its available length
 *
 *  The length of the virtual buffer containing the fragment will be
 *  max_length + head_offset + trail_offset.\n
 *  All length are expressed in bytes.\n
 *
 *  @param   vfrag         IN: The virtual fragment to reallocate
 *                         OUT: The virtual fragment with buffer reallocated
 *                              on success, the old virtual fragment otherwise
 *  @param   start_offset  The offset where data should start
 *                         (should be greater or equal to head_offset)
 *  @param   max_length    The maximum length of the fragment
 *  @param   head_offset   The offset applied before the fragment
 *  @param   trail_offset  The offset applied after the fragment
 *
 *  @return
 *                         - success/informative code among:
 *                           - \ref GSE_STATUS_OK
 *                         - warning/error code among:
 *                           - \ref GSE_STATUS_NULL_PTR
 *                           - \ref GSE_STATUS_BUFF_LENGTH_NULL
 *                           - \ref GSE_STATUS_MALLOC_FAILED
 *                           - \ref GSE_STATUS_BAD_OFFSETS
 *
 *  @ingroup gse_virtual_fragment
 */
gse_status_t gse_reallocate_vfrag(gse_vfrag_t *vfrag,
                                  size_t start_offset, size_t max_length,
                                  size_t head_offset, size_t trail_offset);


#endif
