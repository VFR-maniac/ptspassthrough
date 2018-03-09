/*****************************************************************************
 * example.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <ptspassthrough.h>

#define NUM_INPUT_RAW_PACKETS (15)
#define NUM_ENCODERS (3)

/* Timestamp */
typedef struct
{
    int64_t dts; /* Decode TimeStamp */
    int64_t pts; /* Presentation TimeStamp */
} timestamp_t;

/* Packet */
typedef struct
{
    uint8_t *data;
    size_t   size;
    timestamp_t ts;
} packet_t;

/** Encoder API **/
typedef struct
{
    /* Encoder parameters */
    unsigned int gop_length;
    unsigned int num_enc_delay; /* the number of encoder delay due to lookahead and so on. */
    unsigned int num_dec_delay; /* the number of decoder delay due to picture reordering */
    unsigned int frame_rate_num;
    unsigned int frame_rate_den;

    /* Output timestamp simulator */
    const timestamp_t *out_ts_sim;

    unsigned int num_inputs;
    unsigned int num_outputs;
} encoder_t;

enum
{
    ENCODER_NO_MORE_OUTPUT_PACKETS = -1,
    ENCODER_WANTS_INPUT_PACKETS    = 0,
    ENCODER_OUTPUT_PACKET          = 1
};

static encoder_t encoders[NUM_ENCODERS] =
    {
        { .gop_length = 10, .num_enc_delay = 0, .num_dec_delay = 0, .frame_rate_num =    25, .frame_rate_den =    1 },
        { .gop_length = 10, .num_enc_delay = 2, .num_dec_delay = 1, .frame_rate_num = 30000, .frame_rate_den = 1001 },
        { .gop_length = 10, .num_enc_delay = 2, .num_dec_delay = 2, .frame_rate_num = 24000, .frame_rate_den = 1001 },
    };

static const timestamp_t ts_array[NUM_ENCODERS][NUM_INPUT_RAW_PACKETS] =
    {
        /* decode: I[0]P[1]P[2]P[3]P[4]P[5]P[6]P[7]P[8]P[9]I[10]P[11]P[12]P[13]P[14]
         * output: I[0]P[1]P[2]P[3]P[4]P[5]P[6]P[7]P[8]P[9]I[10]P[11]P[12]P[13]P[14] */
        {
            { 0,  0}, { 1,  1}, { 2,  2}, { 3,  3}, { 4,  4},
            { 5,  5}, { 6,  6}, { 7,  7}, { 8,  8}, { 9,  9},
            {10, 10}, {11, 11}, {12, 12}, {13, 13}, {14, 14},
        },

        /* decode: I[2]B[0]B[1]P[5]B[3]B[4]P[8]B[6]B[7]P[9]I[12]B[10]B[11]P[14]B[13]
         * output:     B[0]B[1]I[2]B[3]B[4]P[5]B[6]B[7]P[8]P[ 9]B[10]B[11]I[12]B[13]P[14] */
        {
            { 0,  3}, { 1,  1}, { 2,  2}, { 3,  6}, { 4,  4},
            { 5,  5}, { 6,  9}, { 7,  7}, { 8,  8}, { 9, 10},
            {10, 13}, {11, 11}, {12, 12}, {13, 15}, {14, 14},
        },

        /* decode: I[2]B[1]B[0]P[5]B[4]B[3]P[8]B[7]B[6]P[9]I[12]B[11]B[10]P[14]B[13]
         * output:         B[0]B[1]I[2]B[3]B[4]P[5]B[6]B[7]P[ 8]P[ 9]B[10]B[11]I[12]B[13]P[14] */
        {
            { 0,  4}, { 1,  3}, { 2,  2}, { 3,  7}, { 4,  6},
            { 5,  5}, { 6, 10}, { 7,  9}, { 8,  8}, { 9, 11},
            {10, 14}, {11, 13}, {12, 12}, {13, 16}, {14, 15},
        }
    };

static void encoder_close( encoder_t *enc )
{
    free( enc );
}

static encoder_t *encoder_open( unsigned int encoder_idx )
{
    encoder_t *enc = malloc( sizeof(encoder_t) );
    if( !enc )
        return NULL;
    *enc = encoders[encoder_idx];

    enc->out_ts_sim = ts_array[encoder_idx];

    return enc;
}

static int encoder_send_raw_packet_into_encoder( encoder_t *enc, packet_t *in )
{
    if( in && in->data && in->size > 0 )
        ++ enc->num_inputs;
    return 0;
}

static int encoder_receive_encoded_packet_from_encoder( encoder_t *enc, packet_t *out )
{
    if( enc->num_outputs < enc->num_inputs )
    {
        if( enc->num_inputs > enc->num_enc_delay )
        {
            out->ts = enc->out_ts_sim[ enc->num_outputs ];

            out->ts.dts *= enc->frame_rate_den;
            out->ts.pts *= enc->frame_rate_den;
            ++ enc->num_outputs;
            return ENCODER_OUTPUT_PACKET;
        }
        else
            return ENCODER_WANTS_INPUT_PACKETS;
    }
    else
        return ENCODER_NO_MORE_OUTPUT_PACKETS;
}


/** Wrapped encoder API **/
typedef struct
{
    encoder_t        *enc;
    ptspassthrough_t *ppt;
} wrapped_encoder_t;

enum
{
    WRAPPED_ENCODER_NO_MORE_OUTPUT_PACKETS = -1,
    WRAPPED_ENCODER_WANTS_INPUT_PACKETS    = 0,
    WRAPPED_ENCODER_OUTPUT_PACKET          = 1
};

static void wrapped_encoder_log( void *handle, ptspassthrough_log_level_t level, const char *format, va_list args )
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
        fprintf( dst, "[Encoder][%s] ", prefix );
    else
        fprintf( dst, "[Encoder] " );
    vfprintf( dst, format, args );
}

static void wrapped_encoder_close( wrapped_encoder_t *wenc )
{
    if( !wenc )
        return;

    ptspassthrough_destroy( &wenc->ppt );
    encoder_close( wenc->enc );
    free( wenc );
}

static wrapped_encoder_t *wrapped_encoder_open( unsigned int encoder_idx )
{
    wrapped_encoder_t *wenc = malloc( sizeof(wrapped_encoder_t) );
    if( !wenc )
        return NULL;
    memset( wenc, 0, sizeof(*wenc) );

    wenc->enc = encoder_open( encoder_idx );
    if( !wenc->enc )
        goto fail;

    if( ptspassthrough_create( &wenc->ppt ) < 0 )
        goto fail;

    if( ptspassthrough_register_logging_callback( wenc->ppt, wrapped_encoder_log, stderr ) < 0 )
        goto fail;

    if( ptspassthrough_set_log_level( wenc->ppt, PTSPASSTHROUGH_LOG_DEBUG ) < 0 )
        goto fail;

    if( ptspassthrough_setup( wenc->ppt, wenc->enc->gop_length /* + wenc->enc->lookahead */,
                                         wenc->enc->num_dec_delay,
                                         wenc->enc->frame_rate_den ) < 0 )
        goto fail;

    return wenc;

fail:
    wrapped_encoder_close( wenc );
    return NULL;
}

static int wrapped_encoder_send_raw_packet_into_encoder( wrapped_encoder_t *wenc, packet_t *in )
{
    if( in->data && in->size > 0 )
        (void)ptspassthrough_put_pts_on_buffer( wenc->ppt, &in->ts.dts, &in->ts.pts );

    return encoder_send_raw_packet_into_encoder( wenc->enc, in );
}

static int wrapped_encoder_receive_encoded_packet_from_encoder( wrapped_encoder_t *wenc, packet_t *out )
{
    int ret = encoder_receive_encoded_packet_from_encoder( wenc->enc, out );

    if( ret == ENCODER_OUTPUT_PACKET )
        (void)ptspassthrough_restore_pts_from_buffer( wenc->ppt, out->ts.pts, &out->ts.dts, &out->ts.pts );

    switch( ret )
    {
        case ENCODER_NO_MORE_OUTPUT_PACKETS : ret = WRAPPED_ENCODER_NO_MORE_OUTPUT_PACKETS; break;
        case ENCODER_WANTS_INPUT_PACKETS    : ret = WRAPPED_ENCODER_WANTS_INPUT_PACKETS;    break;
        case ENCODER_OUTPUT_PACKET          : ret = WRAPPED_ENCODER_OUTPUT_PACKET;          break;
    }

    return ret;
}


/** Example implematation **/

static timestamp_t gen_timestamp( unsigned int picture_num )
{
    return (timestamp_t){ .dts = picture_num * 100 + 70,
                          .pts = picture_num * 100 + 70 };
}

static packet_t get_input_packet( unsigned int picture_num )
{
    static uint8_t data[] = { 0x00 };
    static size_t  size   = sizeof(data);
    timestamp_t ts = gen_timestamp( picture_num );
    if( picture_num < NUM_INPUT_RAW_PACKETS )
        return (packet_t){ .data = data, .size = size, .ts = ts };
    else
        return (packet_t){ .data = NULL, .size =    0, .ts = ts };
}

static int encode( wrapped_encoder_t *wenc, packet_t *out, packet_t *in )
{
    int ret;

    if( (ret = wrapped_encoder_send_raw_packet_into_encoder( wenc, in )) < 0 )
        return ret;

    if( (ret = wrapped_encoder_receive_encoded_packet_from_encoder( wenc, out )) < 0 )
        return ret;

    return ret;
}

int main()
{
    int ret;

    for( unsigned int picture_num = 0; picture_num < NUM_INPUT_RAW_PACKETS; picture_num++ )
    {
        packet_t in = get_input_packet( picture_num );

        fprintf( stderr, "Input: DTS=%"PRId64", PTS=%"PRId64"\n", in.ts.dts, in.ts.pts );
    }

    for( unsigned int encoder_idx = 0; encoder_idx < NUM_ENCODERS; encoder_idx++ )
    {
        fprintf( stderr, "\nStart encoder %u\n", encoder_idx );

        wrapped_encoder_t *wenc = wrapped_encoder_open( encoder_idx );
        if( !wenc )
        {
            fprintf( stderr, "Failed to open encoder %u\n", encoder_idx );
            continue;
        }

        for( unsigned int picture_num = 0; ; picture_num++ )
        {
            packet_t in;    /* Raw packet */
            packet_t out;   /* Encoded packet */

            in = get_input_packet( picture_num );

            ret = encode( wenc, &out, &in );

            if( ret == WRAPPED_ENCODER_NO_MORE_OUTPUT_PACKETS )
                break;
        }
        fprintf( stderr, "End encoder %u\n", encoder_idx );

        wrapped_encoder_close( wenc );
    }

    return 0;
}
