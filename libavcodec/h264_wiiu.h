/**
 * Decoder using Wii U's h264.rpl
 * Created 2020 by GaryOderNichts
 * Modified 2026 by Roastedd for CoffeeFlix: buffer size and layout, packet copy, decoding of
 * every picture and reordering, stream size changes, decoding into the caller's frames.
 */

#ifndef H264_WIIU_H
#define H264_WIIU_H

#include <wut.h>
#include <h264/decode.h>

#include "avcodec.h"
#include "wiiu_frame_targets.h"

// Pictures the reorder queue holds back, at most (H.264 streams rarely need more than 2).
#define WIIU_MAX_REORDER 4
// Pictures one H264DECExecute call can hand to the output callback.
#define WIIU_MAX_OUTPUT 4

typedef struct
{
    void* framebuffer;
    int width;
    int height;
    int pitch;
    int crop_top;
    int crop_bottom;
    int crop_left;
    int crop_right;
    double timestamp;
} WIIUOutput;

typedef struct
{
    // Must stay the first member: its address is the context's, whether h264.rpl passes the
    // user memory pointer itself or the value it points to back to the output callback.
    void* self;

    // decoder memory
    void* decoder;

    // the picture the decoder writes, sized for the largest one it can decode
    void* framebuffer;

    // aligned copy of the packet the decoder reads
    uint8_t* bitstream;
    int bitstream_size;

    // pictures reported by the output callback during H264DECExecute
    WIIUOutput output[WIIU_MAX_OUTPUT];
    int output_count;

    // timestamps of the last packets handed to the decoder
    int64_t sent_pts[8];
    int sent_index;

    // What the decoder has done so far, to adapt and to explain problems in the log.
    int callback_seen;
    int executes;
    int pictures;
    int ref_ok;
    int nonref_ok;
    int nonref_errors;
    int skip_nonref;  // left out from now on: the decoder rejects them
    int reported;

    // Pictures come out in decoding order; they are put back into presentation order here.
    AVFrame* queue[WIIU_MAX_REORDER + 1];
    int queue_count;
    int delay;
    int64_t last_pts;

    // Layout of the last picture (the size only changes at an IDR picture). Pictures after it
    // are decoded straight into the caller's frames when their buffers have that layout.
    int seq_pitch;
    int seq_height;  // padded
    int seq_x, seq_y, seq_width, seq_height_visible;
    int direct_reported;
    // The last sequence parameter set (IDR pictures with the same one keep the size).
    uint8_t sps[256];
    int sps_size;
    // Submitted targets stay owned through delayed output and errors.
    WiiUFrameTargets targets;
    int targets_reported;
    int reset_failed;

    // Timing, logged every few seconds.
    int64_t stats_since;
    int stats_pictures;
    int stats_direct;
    int stats_executes;
    int64_t stats_execute_us;
    int64_t stats_execute_max_us;
    int64_t stats_copy_us;
    uint64_t stats_cpu_ns;
    uint64_t stats_wakes;
} WIIUContext;

#endif /* H264_WIIU_H */
