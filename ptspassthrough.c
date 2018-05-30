/*****************************************************************************
 * ptspassthrough.c
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "ptspassthrough.h"


struct ptspassthrough_tag
{
    int64_t     *pts_buffer;
    unsigned int num_pts_buffer;

    /* the maximum delay at decoding due to picture reordering */
    unsigned int num_dec_delay;

    /* Timestamp increment for getting index from PTS generated by the encoder.
     * Due to this, ptspassthrough assume that the encoder produces timestamps of constant frame rate. */
    int64_t      timestamp_increment;

    /* logger */
    ptspassthrough_logging_callback log_callback;
    ptspassthrough_log_level_t      log_level;
    void                           *log_handle;

    /* Internal variables which users can't changes directly. */
    uint64_t in_count;
    uint64_t out_count;
};

static void ptspassthrough_log_default( void *handle, ptspassthrough_log_level_t level, const char *format, va_list args )
{
    const char *prefix = NULL;
    FILE *dst = handle;

    switch( level )
    {
        case PTSPASSTHROUGH_LOG_FATAL:
            prefix = "Fatal";
            break;
        case PTSPASSTHROUGH_LOG_ERROR:
            prefix = "Error";
            break;
        case PTSPASSTHROUGH_LOG_WARNING:
            prefix = "Warning";
            break;
        case PTSPASSTHROUGH_LOG_INFO:
            prefix = "Info";
            break;
        case PTSPASSTHROUGH_LOG_DEBUG:
            prefix = "Debug";
            break;
        default:
            break;
    }

    if( prefix )
        fprintf( dst, "[ptspassthrough][%s] ", prefix );
    else
        fprintf( dst, "[ptspassthrough] " );
    vfprintf( dst, format, args );
}

static void ptspassthrough_log( ptspassthrough_t *h, ptspassthrough_log_level_t level, const char *format, ... )
{
    if( h->log_callback == NULL )
        return;

    if( level > h->log_level )
        return;

    va_list args;
    va_start( args, format );
    h->log_callback( h->log_handle, level, format, args );
    va_end( args );
}

ptspassthrough_err_t ptspassthrough_create( ptspassthrough_t **hp )
{
    if( !hp )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    ptspassthrough_t *h = malloc( sizeof(ptspassthrough_t) );
    if( !h )
        return PTSPASSTHROUGH_ERR_MEMORY_ALLOC;
    memset( h, 0, sizeof(*h) );

    h->log_callback = ptspassthrough_log_default;
    h->log_level    = PTSPASSTHROUGH_LOG_INFO;
    h->log_handle   = stderr;

    *hp = h;

    return PTSPASSTHROUGH_ERR_NONE;
}

void ptspassthrough_destroy( ptspassthrough_t **hp )
{
    if( !hp || !*hp )
        return;

    free( (*hp)->pts_buffer );
    free( *hp );

    *hp = NULL;
}

ptspassthrough_err_t ptspassthrough_setup( ptspassthrough_t *h, unsigned int num_pts_buffer, unsigned int decoder_delay, int64_t timestamp_increment )
{
    if( !h || num_pts_buffer == 0 )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG,
                        "num_pts_buffer=%u, decoder_delay=%u, timestamp_increment=%"PRId64"\n",
                        num_pts_buffer, decoder_delay, timestamp_increment );

    free( h->pts_buffer );
    h->pts_buffer = malloc( num_pts_buffer * sizeof(int64_t) );
    if( !h->pts_buffer )
    {
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_FATAL, "Failed to allocate PTS ring buffer.\n" );
        return PTSPASSTHROUGH_ERR_MEMORY_ALLOC;
    }
    h->num_pts_buffer = num_pts_buffer;
    memset( h->pts_buffer, 0, num_pts_buffer * sizeof(int64_t) );

    h->num_dec_delay       = decoder_delay;
    h->timestamp_increment = timestamp_increment;
    h->in_count            = 0;
    h->out_count           = 0;

    return PTSPASSTHROUGH_ERR_NONE;
}

ptspassthrough_err_t ptspassthrough_put_pts_on_buffer( ptspassthrough_t *h, int64_t *dts, int64_t *pts )
{
    if( !h || !dts || !pts )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    if( h->in_count - h->out_count >= h->num_pts_buffer )
    {
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG,
                            "Need outputting already buffered PTS before inputting PTS due to lack of the size of the ring buffer.\n" );
        return PTSPASSTHROUGH_ERR_MORE_OUTPUTS;
    }

    ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG, "Input(%"PRId64"): DTS=%"PRId64", PTS=%"PRId64"\n", h->in_count, *dts, *pts );

    h->pts_buffer[ h->in_count % h->num_pts_buffer ] = *pts;

    /* HACK: Make any encoder always produce timestamps of constant frame rate if the encoder can takes timestamps. */
    *dts = h->in_count;
    *pts = h->in_count;

    ++ h->in_count;
    return PTSPASSTHROUGH_ERR_NONE;
}

ptspassthrough_err_t ptspassthrough_restore_pts_from_buffer( ptspassthrough_t *h, int64_t cfr_pts, int64_t *dts, int64_t *pts )
{
    if( !h || !dts || !pts )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    /* Restore PTS from buffered PTS. */
    int64_t index = cfr_pts / h->timestamp_increment - h->num_dec_delay;
    if( index < 0 )
    {
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_FATAL, "index=%"PRId64" <- cfr_pts=%"PRId64" (invalid!)\n", index, cfr_pts );
        return PTSPASSTHROUGH_ERR_INVALID_DATA;
    }
    if( h->in_count - index > h->num_pts_buffer )
    {
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_FATAL, "index=%"PRId64" <- cfr_pts=%"PRId64" -> already abandoned old PTS!\n", index, cfr_pts );
        return PTSPASSTHROUGH_ERR_INVALID_DATA;
    }
    *pts = h->pts_buffer[ index % h->num_pts_buffer ];
    ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG, "index=%"PRId64" <- cfr_pts=%"PRId64" -> PTS=%"PRId64"\n", index, cfr_pts, *pts );

    /* Generate DTS from buffered PTS. */
    if( h->out_count >= h->num_dec_delay )
    {
        index = h->out_count - h->num_dec_delay;
        *dts = h->pts_buffer[ index % h->num_pts_buffer ];
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG, "index=%"PRId64" -> DTS=%"PRId64"\n", index, *dts );
    }
    else
    {
        if( h->in_count > h->num_dec_delay )
        {
            int64_t initial_delay_time = h->pts_buffer[ h->num_dec_delay ] - h->pts_buffer[0];
            *dts = h->pts_buffer[ h->out_count % h->num_pts_buffer ] - initial_delay_time;
            ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG, "index=%"PRId64", initial_delay_time=%"PRId64" -> DTS=%"PRId64"\n",
                                h->out_count, initial_delay_time, *dts );
        }
        else
        {
            ptspassthrough_log( h, PTSPASSTHROUGH_LOG_WARNING,
                                "Apparently an output packet is emitted immediately but reordering might be still produced!\n"
                                "Please buffer output packet(s) until a valid DTS is ready by inputting more PTS.\n");
            return PTSPASSTHROUGH_ERR_MORE_INPUTS;
        }
    }

    if( *pts < *dts )
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_ERROR, "Output(%"PRId64"): DTS=%"PRId64", PTS=%"PRId64" (invalid!)\n", h->out_count, *dts, *pts );
    else
        ptspassthrough_log( h, PTSPASSTHROUGH_LOG_DEBUG, "Output(%"PRId64"): DTS=%"PRId64", PTS=%"PRId64"\n", h->out_count, *dts, *pts );

    ++ h->out_count;
    return PTSPASSTHROUGH_ERR_NONE;
}

ptspassthrough_err_t ptspassthrough_register_logging_callback( ptspassthrough_t *h, ptspassthrough_logging_callback callback, void *handle )
{
    if( !h )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    h->log_callback = callback;
    h->log_handle   = handle;

    return PTSPASSTHROUGH_ERR_NONE;
}

ptspassthrough_err_t ptspassthrough_set_log_level( ptspassthrough_t *h, ptspassthrough_log_level_t level )
{
    if( !h || level > PTSPASSTHROUGH_LOG_DEBUG )
        return PTSPASSTHROUGH_ERR_FUNCTION_PARAM;

    h->log_level = level;

    return PTSPASSTHROUGH_ERR_NONE;
}
