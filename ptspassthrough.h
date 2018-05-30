/*****************************************************************************
 * ptspassthrough.h
 *****************************************************************************
 * Copyright (C) 2018
 *
 * Authors: Yusuke Nakamura <muken.the.vfrmaniac@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *****************************************************************************/
 
#ifndef PTSPASSTHROUGH_H
#define PTSPASSTHROUGH_H

#include <stdint.h>
#include <stdarg.h>

#define PTSPASSTHROUGH_VERSION_MAJOR 1
#define PTSPASSTHROUGH_VERSION_MINOR 1
#define PTSPASSTHROUGH_VERSION_MICRO 0

typedef enum
{
    PTSPASSTHROUGH_ERR_MORE_OUTPUTS   = -6,
    PTSPASSTHROUGH_ERR_MORE_INPUTS    = -5,
    PTSPASSTHROUGH_ERR_INVALID_DATA   = -4,
    PTSPASSTHROUGH_ERR_PATCH_WELCOME  = -3,
    PTSPASSTHROUGH_ERR_FUNCTION_PARAM = -2,
    PTSPASSTHROUGH_ERR_MEMORY_ALLOC   = -1,
    PTSPASSTHROUGH_ERR_NONE           = 0,
} ptspassthrough_err_t;

typedef enum
{
    PTSPASSTHROUGH_LOG_QUIET   = 0,
    PTSPASSTHROUGH_LOG_FATAL   = 4,
    PTSPASSTHROUGH_LOG_ERROR   = 8,
    PTSPASSTHROUGH_LOG_WARNING = 12,
    PTSPASSTHROUGH_LOG_INFO    = 16,
    PTSPASSTHROUGH_LOG_DEBUG   = 20,
} ptspassthrough_log_level_t;

/* The top-level opaque handler of ptspassthrough. */
typedef struct ptspassthrough_tag ptspassthrough_t;

/* Callback for user-defined loggers. */
typedef void (*ptspassthrough_logging_callback)( void *handle, ptspassthrough_log_level_t level, const char *format, va_list args );

/* Create a ptspassthrough instance. */
ptspassthrough_err_t ptspassthrough_create( ptspassthrough_t **hp );

/* Destroy a ptspassthrough instance. */
void ptspassthrough_destroy( ptspassthrough_t **hp );

/* Set up ptspassthrough fundamentally.
 *   num_pts_buffer      : the capacity of input PTS in the ring buffer
 *   decoder_delay       : the maximum delay in picture units at decoding due to picture reordering
 *   timestamp_increment : the timestamp increment for getting index from PTS generated by the encoder */
ptspassthrough_err_t ptspassthrough_setup( ptspassthrough_t *h, unsigned int num_pts_buffer, unsigned int decoder_delay, int64_t timestamp_increment );

/* Put input PTS on the internal ring buffer and change given PTS and DTS into manageable ones for ptspassthrough. */
ptspassthrough_err_t ptspassthrough_put_pts_on_buffer( ptspassthrough_t *h, int64_t *dts, int64_t *pts );

/* Restore input PTS and generate appropriate DTS from the internal ring buffer.
 * 'cfr_pts' is PTS produced by the encoder and is used as an index to access the buffer. */
ptspassthrough_err_t ptspassthrough_restore_pts_from_buffer( ptspassthrough_t *h, int64_t cfr_pts, int64_t *dts, int64_t *pts );

/* Register the logging callback and the pointer to associated user-defined handler. */
ptspassthrough_err_t ptspassthrough_register_logging_callback( ptspassthrough_t *h, ptspassthrough_logging_callback callback, void *handle );

/* Set level which is the threshold for logging.
 * Each message is sent to the log when its level assigned internally is less than or equal to the 'level'.
 * If you don't set level explicitly, it is inferred to be equal to 'PTSPASSTHROUGH_LOG_INFO'. */
ptspassthrough_err_t ptspassthrough_set_log_level( ptspassthrough_t *h, ptspassthrough_log_level_t level );

#endif
