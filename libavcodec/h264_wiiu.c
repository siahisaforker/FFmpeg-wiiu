/**
 * Decoder using Wii U's h264.rpl
 * Created 2020 by GaryOderNichts
 * Modified 2026 by Roastedd for CoffeeFlix: buffer size and layout, packet copy, decoding of
 * every picture and reordering, stream size changes, decoding into the caller's frames.
 *
 * Buffer layout, bitstream copy and status handling follow Moonlight for Wii U; decoding-order
 * output (one picture per H264DECExecute with OUTPUT_PER_FRAME set) follows Cemu's h264.rpl.
 */

#include "h264_wiiu.h"

#include "libavutil/common.h"
#include "libavutil/imgutils.h"
#include "libavutil/time.h"
#include "internal.h"

#include <coreinit/cache.h>
#include <coreinit/thread.h>
#include <malloc.h>
#include <string.h>

// memory requirement for the maximum supported level (level 42)
#define H264_MEM_REQUIREMENT (0x2200000 + 0x3ff + 0x480000)
#define H264_MEM_ALIGNMENT 0x400

// The decoder writes NV12 with each row padded to 256 bytes and the height padded to 16 lines,
// the chroma plane right after the padded luma plane.
#define H264_FRAME_PITCH(w) (((w) + 0xff) & ~0xff)
#define H264_FRAME_HEIGHT(h) (((h) + 0xf) & ~0xf)
#define H264_FRAME_SIZE(pitch, h) (((pitch) * H264_FRAME_HEIGHT(h) * 3) >> 1)
// Holds any picture a level 4.2 decoder accepts, so a stream that changes size (an ad in a
// live stream) can't make the decoder write past the buffer.
#define H264_MAX_FRAME_SIZE H264_FRAME_SIZE(H264_FRAME_PITCH(2800), 1408)

static void frame_callback(H264DecodeOutput *output)
{
    WIIUContext* ctx = output ? output->userMemory : NULL;
    int i;

    if (!ctx || ctx->self != ctx)
        return;

    for (i = 0; i < output->frameCount && ctx->output_count < WIIU_MAX_OUTPUT; i++)
    {
        H264DecodeResult* result = output->decodeResults[i];
        WIIUOutput* out;

        if (!result || !result->framebuffer)
            continue;

        ctx->callback_seen = 1;
        out = &ctx->output[ctx->output_count++];
        out->framebuffer = result->framebuffer;
        out->width = result->width;
        out->height = result->height;
        out->pitch = result->nextLine;
        out->crop_top = out->crop_bottom = out->crop_left = out->crop_right = 0;
        if (result->cropEnableFlag)
        {
            out->crop_top = result->cropTop;
            out->crop_bottom = result->cropBottom;
            out->crop_left = result->cropLeft;
            out->crop_right = result->cropRight;
        }
        out->timestamp = result->timestamp;
    }
}

static void h264_wiiu_release(WIIUContext* ctx, int opened)
{
    int i;

    if (ctx->decoder && opened)
    {
        H264DECFlush(ctx->decoder);
        H264DECEnd(ctx->decoder);
        H264DECClose(ctx->decoder);
    }
    wiiu_targets_clear(&ctx->targets);
    free(ctx->decoder);
    ctx->decoder = NULL;

    free(ctx->framebuffer);
    ctx->framebuffer = NULL;
    free(ctx->bitstream);
    ctx->bitstream = NULL;
    ctx->bitstream_size = 0;

    for (i = 0; i < ctx->queue_count; i++)
        av_frame_free(&ctx->queue[i]);
    ctx->queue_count = 0;
}

// Memory the decoder can use (H264DECCheckMemSegmentation; some places in the heap fail it).
// A block that fails is kept while asking again, so the heap hands out another place.
static void *alloc_decoder_memory(AVCodecContext *avctx, uint32_t size)
{
    void *held[4];
    void *p;
    int n = 0, res = 0;

    while ((p = memalign(H264_MEM_ALIGNMENT, size)) && (res = H264DECCheckMemSegmentation(p, size)) != 0)
    {
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: memory at %p unusable (0x%07X), trying elsewhere\n", p, res);
        if (n == (int) FF_ARRAY_ELEMS(held))
        {
            free(p);
            p = NULL;
            break;
        }
        held[n++] = p;
    }
    while (n > 0)
        free(held[--n]);
    return p;
}

static int h264_wiiu_decode_init(AVCodecContext *avctx)
{
    WIIUContext* ctx = avctx->priv_data;
    int res, i;

    ctx->self = ctx;
    ctx->last_pts = AV_NOPTS_VALUE;
    for (i = 0; i < (int) FF_ARRAY_ELEMS(ctx->sent_pts); i++)
        ctx->sent_pts[i] = AV_NOPTS_VALUE;
    // Two pictures cover typical encoder settings (B-frames used as references), so the delay
    // rarely has to grow mid-stream, which costs a picture.
    ctx->delay = av_clip(FFMAX(avctx->has_b_frames, 2), 0, WIIU_MAX_REORDER);

    ctx->decoder = alloc_decoder_memory(avctx, H264_MEM_REQUIREMENT);
    ctx->framebuffer = alloc_decoder_memory(avctx, H264_MAX_FRAME_SIZE);
    if (!ctx->decoder || !ctx->framebuffer)
    {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: Not enough memory the decoder can use\n");
        h264_wiiu_release(ctx, 0);
        return AVERROR(ENOMEM);
    }
    // Only the decoder writes the frame buffer: flushed once here, the CPU cache never holds
    // changes of its own that could later land on top of a decoded picture.
    DCFlushRange(ctx->framebuffer, H264_MAX_FRAME_SIZE);

    res = H264DECInitParam(H264_MEM_REQUIREMENT, ctx->decoder);
    if (res == 0)
        res = H264DECSetParam_FPTR_OUTPUT(ctx->decoder, frame_callback);
    if (res == 0)
        res = H264DECSetParam_USER_MEMORY(ctx->decoder, &ctx->self);
    if (res == 0)
        res = H264DECSetParam_OUTPUT_PER_FRAME(ctx->decoder, 1);
    if (res != 0)
    {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: Error initializing decoder 0x%07X\n", res);
        h264_wiiu_release(ctx, 0);
        return AVERROR_EXTERNAL;
    }

    res = H264DECOpen(ctx->decoder);
    if (res != 0)
    {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: Error opening decoder 0x%07X\n", res);
        h264_wiiu_release(ctx, 0);
        return AVERROR_EXTERNAL;
    }

    res = H264DECBegin(ctx->decoder);
    if (res != 0)
    {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: Error preparing decoder 0x%07X\n", res);
        H264DECClose(ctx->decoder);
        h264_wiiu_release(ctx, 0);
        return AVERROR_EXTERNAL;
    }

    // Wii U decoder only outputs NV12
    avctx->pix_fmt = AV_PIX_FMT_NV12;

    return 0;
}

static int h264_wiiu_decode_close(AVCodecContext *avctx)
{
    h264_wiiu_release(avctx->priv_data, 1);
    return 0;
}

static void h264_wiiu_flush(AVCodecContext *avctx)
{
    WIIUContext* ctx = avctx->priv_data;
    int i;

    // Ending the hardware session drains outstanding output before its targets
    // are released. Begin starts afresh at the next seek/keyframe.
    ctx->output_count = 0;
    H264DECFlush(ctx->decoder);
    if (H264DECEnd(ctx->decoder) != 0) {
        ctx->reset_failed = 1;
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: could not drain decoder for seek\n");
        return; // keep targets alive until Close
    }
    wiiu_targets_clear(&ctx->targets);
    if (H264DECBegin(ctx->decoder) != 0) {
        ctx->reset_failed = 1;
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: could not restart decoder after seek\n");
    }
    ctx->output_count = 0;
    ctx->seq_pitch = 0;
    for (i = 0; i < (int) FF_ARRAY_ELEMS(ctx->sent_pts); ++i)
        ctx->sent_pts[i] = AV_NOPTS_VALUE;
    for (i = 0; i < ctx->queue_count; i++)
        av_frame_free(&ctx->queue[i]);
    ctx->queue_count = 0;
    ctx->last_pts = AV_NOPTS_VALUE;
}

// Seconds between two timestamps of the stream.
static double h264_wiiu_seconds(AVCodecContext *avctx, int64_t a, int64_t b)
{
    AVRational tb = avctx->pkt_timebase;
    if (tb.num <= 0 || tb.den <= 0)
        tb = (AVRational){ 1, 90000 };
    return (a - b) * av_q2d(tb);
}

// Whether the packet can change the picture size, which only a sequence parameter set sets: it
// holds one other than the last one (kept, and no picture goes straight into a caller's frame
// until one of the new size came out), or an IDR picture without one (it could switch to one
// sent before). The NAL units are read up to the first slice; a packet without one counts.
static int h264_wiiu_new_sequence(WIIUContext* ctx, const uint8_t* p, int size)
{
    const uint8_t* end = p + size;
    const uint8_t* sps = NULL;
    int sps_size = -1, idr = -1;

    for (; end - p > 3; p++)
    {
        int type;
        if (p[0] != 0 || p[1] != 0 || p[2] != 1)
            continue;
        if (sps && sps_size < 0)
            sps_size = (int) (p - sps);
        type = p[3] & 0x1f;
        if (type == 1 || type == 5)
        {
            idr = type == 5;
            break;
        }
        if (type == 7)
            sps = p + 3;
        p += 2;
    }
    if (idr < 0 || (sps && sps_size < 0))
        return 1;
    if (!sps)
        return idr;
    while (sps_size > 0 && sps[sps_size - 1] == 0)  // the start code's leading zero
        sps_size--;
    if (sps_size == ctx->sps_size && !memcmp(sps, ctx->sps, sps_size))
        return 0;
    ctx->sps_size = sps_size <= (int) sizeof(ctx->sps) ? sps_size : 0;
    memcpy(ctx->sps, sps, ctx->sps_size);
    ctx->seq_pitch = 0;
    return 1;
}

// A frame from the caller's allocator for the decoder to write into, when its buffer is laid out
// the way the decoder writes the current pictures (pitch, padded height, alignment), else null.
// The default allocator's frames never are.
static AVFrame* h264_wiiu_direct_frame(AVCodecContext *avctx, WIIUContext* ctx)
{
    AVFrame* frame;
    int size;

    if (avctx->get_buffer2 == avcodec_default_get_buffer2 || ctx->seq_pitch <= 0 || ctx->seq_x || ctx->seq_y ||
        ctx->seq_width != avctx->width || ctx->seq_height_visible != avctx->height)
        return NULL;
    size = H264_FRAME_SIZE(ctx->seq_pitch, ctx->seq_height);
    frame = av_frame_alloc();
    if (!frame)
        return NULL;
    if (ff_get_buffer(avctx, frame, 0) < 0 || !frame->buf[0] || frame->buf[0]->data != frame->data[0] ||
        frame->buf[0]->size < size || ((uintptr_t) frame->data[0] & (H264_MEM_ALIGNMENT - 1)) ||
        frame->linesize[0] != ctx->seq_pitch || frame->linesize[1] != ctx->seq_pitch ||
        frame->data[1] != frame->data[0] + ctx->seq_pitch * ctx->seq_height ||
        H264DECCheckMemSegmentation(frame->data[0], size) != 0)
    {
        if (!ctx->direct_reported)
        {
            ctx->direct_reported = 1;
            av_log(avctx, AV_LOG_WARNING, "h264_wiiu: copying pictures, the frames don't have the decoder's layout\n");
        }
        av_frame_free(&frame);
    }
    return frame;
}

// Puts a decoded picture into the reorder queue: the caller's frame it was decoded into when that
// has the picture's layout (the frame is taken from *direct), else a copy out of the decoder's
// buffer.
static int h264_wiiu_queue_picture(AVCodecContext *avctx, const WIIUOutput* out, int64_t pts)
{
    WIIUContext* ctx = avctx->priv_data;
    uint8_t* src[4] = { NULL };
    int src_linesize[4] = { 0 };
    int x = 0, y = 0, width, height, pitch, size, ret;
    AVFrame** owner = wiiu_target_slot(&ctx->targets, out->framebuffer);
    AVFrame* target = owner ? *owner : NULL;
    AVFrame* frame;
    int64_t start;

    if (out->width <= 0 || out->height <= 0 || out->width > 4096 || out->height > 4096)
        return AVERROR_INVALIDDATA;
    pitch = out->pitch >= out->width && out->pitch <= 4096 ? out->pitch : H264_FRAME_PITCH(out->width);
    size = H264_FRAME_SIZE(pitch, out->height);
    if (out->framebuffer == ctx->framebuffer && size > H264_MAX_FRAME_SIZE)
        return AVERROR_INVALIDDATA;
    if (!target && out->framebuffer != ctx->framebuffer) {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: output names an unowned buffer\n");
        return AVERROR_INVALIDDATA;
    }
    if (target && size > target->buf[0]->size)
    {
        av_log(avctx, AV_LOG_ERROR, "h264_wiiu: a %dx%d picture overflowed the frame it was decoded into\n",
               out->width, out->height);
        return AVERROR_INVALIDDATA;
    }

    // Visible part: the container's size when it describes this picture (it knows the exact
    // crop), else the decoder's crop (the stream changed size).
    width = out->width;
    height = out->height;
    if (out->crop_left >= 0 && out->crop_right >= 0 && out->crop_top >= 0 && out->crop_bottom >= 0 &&
        out->crop_left + out->crop_right < width && out->crop_top + out->crop_bottom < height)
    {
        x = out->crop_left;
        y = out->crop_top;
        width -= out->crop_left + out->crop_right;
        height -= out->crop_top + out->crop_bottom;
    }
    if (avctx->width > 0 && avctx->height > 0 && avctx->width <= out->width && avctx->height <= out->height &&
        out->width - avctx->width < 16 && out->height - avctx->height < 16)
    {
        width = avctx->width;
        height = avctx->height;
        x = FFMIN(x, out->width - width);
        y = FFMIN(y, out->height - height);
    }
    else if (width != avctx->width || height != avctx->height)
    {
        av_log(avctx, AV_LOG_INFO, "h264_wiiu: Picture size %dx%d\n", width, height);
        if ((ret = ff_set_dimensions(avctx, width, height)) < 0)
            return ret;
    }
    x &= ~1;
    y &= ~1;

    ctx->seq_pitch = pitch;
    ctx->seq_height = H264_FRAME_HEIGHT(out->height);
    ctx->seq_x = x;
    ctx->seq_y = y;
    ctx->seq_width = width;
    ctx->seq_height_visible = height;

    if (target && !x && !y && target->width == width && target->height == height && target->linesize[0] == pitch &&
        target->linesize[1] == pitch && target->data[1] == target->data[0] + pitch * H264_FRAME_HEIGHT(out->height))
    {
        // Decoded straight into the caller's frame: nothing to copy, and the CPU never reads it.
        frame = target;
        *owner = NULL;
        ctx->stats_direct++;
    }
    else
    {
        start = av_gettime_relative();

        // The picture was written by the decoder, not through the CPU cache.
        DCInvalidateRange(out->framebuffer, size);

        frame = av_frame_alloc();
        if (!frame)
            return AVERROR(ENOMEM);
        if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        {
            av_frame_free(&frame);
            return ret;
        }

        src[0] = (uint8_t*) out->framebuffer + y * pitch + x;
        src[1] = (uint8_t*) out->framebuffer + pitch * H264_FRAME_HEIGHT(out->height) + (y / 2) * pitch + x;
        src_linesize[0] = src_linesize[1] = pitch;
        av_image_copy(frame->data, frame->linesize, (const uint8_t **) src, src_linesize, AV_PIX_FMT_NV12, width, height);

        // The caller's frames can go to the GPU as they are, like the ones decoded into: out of
        // the CPU cache with them.
        if (avctx->get_buffer2 != avcodec_default_get_buffer2)
        {
            DCFlushRange(frame->data[0], frame->linesize[0] * height);
            DCFlushRange(frame->data[1], frame->linesize[1] * ((height + 1) / 2));
        }

        ctx->stats_copy_us += av_gettime_relative() - start;
    }
    // A cropped/layout-mismatched target was copied; its output has now arrived.
    if (target && owner && *owner)
        av_frame_free(owner);
    ctx->stats_pictures++;

    frame->pts = pts;

    if (ctx->pictures++ == 0)
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: first picture %dx%d, pitch %d, crop %d %d %d %d, %s\n", out->width,
               out->height, pitch, out->crop_left, out->crop_right, out->crop_top, out->crop_bottom,
               out->framebuffer == ctx->framebuffer ? "in our buffer" : "in another buffer");

    // A picture that is due before one already returned came too late to be put in order: the
    // stream reorders more than expected. Hold back one more picture from now on.
    if (pts != AV_NOPTS_VALUE && ctx->last_pts != AV_NOPTS_VALUE && pts <= ctx->last_pts)
    {
        if (h264_wiiu_seconds(avctx, ctx->last_pts, pts) < 1.0)
        {
            if (ctx->delay < WIIU_MAX_REORDER)
                ctx->delay++;
            av_frame_free(&frame);
            return 0;
        }
        ctx->last_pts = AV_NOPTS_VALUE;  // timestamps jumped back (a new stream section)
    }

    if (ctx->queue_count > WIIU_MAX_REORDER)
    {
        av_frame_free(&frame);
        return 0;
    }
    ctx->queue[ctx->queue_count++] = frame;
    return 0;
}

// Timestamp of a decoded picture: the one given to the decoder with its packet. Decoding order
// makes that the current packet; anything else the decoder reports must be a recent packet's.
static int64_t h264_wiiu_pts(WIIUContext* ctx, const WIIUOutput* out, int64_t current)
{
    int64_t pts;
    int i;

    if (current == AV_NOPTS_VALUE || !(out->timestamp > -9e18 && out->timestamp < 9e18))
        return current;
    pts = (int64_t) out->timestamp;
    for (i = 0; i < (int) FF_ARRAY_ELEMS(ctx->sent_pts); i++)
        if (ctx->sent_pts[i] == pts)
            return pts;
    return current;
}

// Hands the packet given to the decoder over for decoding into the frame from *direct (the
// decoder's own buffer without one) and queues the pictures that come out.
static int h264_wiiu_execute(AVCodecContext *avctx, AVPacket *avpkt, int skippable, void* target)
{
    WIIUContext* ctx = avctx->priv_data;
    OSThread* thread = OSGetCurrentThread();
    uint64_t cpu = thread->coreTimeConsumedNs, wakes = thread->wakeCount;
    int64_t start = av_gettime_relative(), elapsed;
    int res, i;

    ctx->output_count = 0;
    res = H264DECExecute(ctx->decoder, target);
    elapsed = av_gettime_relative() - start;
    ctx->stats_executes++;
    ctx->stats_execute_us += elapsed;
    ctx->stats_execute_max_us = FFMAX(ctx->stats_execute_max_us, elapsed);
    ctx->stats_cpu_ns += thread->coreTimeConsumedNs - cpu;
    ctx->stats_wakes += thread->wakeCount - wakes;

    ctx->executes++;
    if (!ctx->reported && ctx->executes >= 30 && ctx->pictures == 0)
    {
        ctx->reported = 1;
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: no picture after %d packets (status 0x%X, callback %s)\n",
               ctx->executes, res, ctx->callback_seen ? "seen" : "never called");
    }
    if ((res & ~0xff) != 0)
    {
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: Error decoding %s picture: 0x%07X\n",
               skippable ? "an unreferenced" : "a reference", res);
        if (skippable && ++ctx->nonref_errors >= 8 && ctx->nonref_ok == 0 && ctx->ref_ok >= 3 && !ctx->skip_nonref)
        {
            ctx->skip_nonref = 1;
            av_log(avctx, AV_LOG_WARNING, "h264_wiiu: leaving out unreferenced pictures, the decoder rejects them\n");
        }
        return AVERROR_INVALIDDATA;
    }
    if (skippable)
        ctx->nonref_ok++;
    else
        ctx->ref_ok++;

    if (ctx->output_count == 0)
    {
        // A decoder that never calls back after several pictures leaves each one where it was
        // told to write it. (One that does call back can simply have nothing to show yet:
        // reordering.)
        WIIUOutput out = { 0 };
        if (ctx->callback_seen || ctx->executes < 8 || res != 0xE4 || avctx->width <= 0 || avctx->height <= 0)
            return 0;
        out.framebuffer = target;
        out.width = avctx->width;
        out.height = avctx->height;
        out.pitch = H264_FRAME_PITCH(avctx->width);
        return h264_wiiu_queue_picture(avctx, &out, avpkt->pts);
    }

    for (i = 0; i < ctx->output_count; i++)
    {
        const WIIUOutput* out = &ctx->output[i];
        res = h264_wiiu_queue_picture(avctx, out, h264_wiiu_pts(ctx, out, avpkt->pts));
        if (res < 0)
            return res;
    }
    return 0;
}

// Logs every 10 seconds where the decoding time went.
static void h264_wiiu_stats(AVCodecContext *avctx, WIIUContext* ctx)
{
    int64_t now = av_gettime_relative();
    int copies = ctx->stats_pictures - ctx->stats_direct;

    if (!ctx->stats_since)
        ctx->stats_since = now;
    if (now - ctx->stats_since < 10000000 || ctx->stats_executes == 0)
        return;
    av_log(avctx, AV_LOG_WARNING, "h264_wiiu: %d pictures in %.1f s, %d without a copy: decoding %.1f ms each "
           "(longest %.1f, CPU %.1f ms, %.1f waits), copying %.1f ms each\n",
           ctx->stats_pictures, (now - ctx->stats_since) / 1e6, ctx->stats_direct,
           ctx->stats_execute_us / 1e3 / ctx->stats_executes, ctx->stats_execute_max_us / 1e3,
           ctx->stats_cpu_ns / 1e6 / ctx->stats_executes, (double) ctx->stats_wakes / ctx->stats_executes,
           copies > 0 ? ctx->stats_copy_us / 1e3 / copies : 0.0);
    ctx->stats_since = now;
    ctx->stats_pictures = ctx->stats_direct = ctx->stats_executes = 0;
    ctx->stats_execute_us = ctx->stats_execute_max_us = ctx->stats_copy_us = 0;
    ctx->stats_cpu_ns = ctx->stats_wakes = 0;
}

static int h264_wiiu_decode_packet(AVCodecContext *avctx, AVPacket *avpkt)
{
    WIIUContext* ctx = avctx->priv_data;
    int size = avpkt->size;
    int res;
    BOOL skippable = FALSE;
    AVFrame* direct = NULL;
    AVFrame** slot;
    void* target;

    if (ctx->reset_failed)
        return AVERROR_EXTERNAL;

    // Pictures nothing else refers to (most B-frames) are left out when playback falls behind,
    // and for good if the decoder turned out to reject them.
    if (H264DECCheckSkipableFrame(avpkt->data, size, &skippable) != 0)
        skippable = FALSE;
    if (skippable && (avctx->skip_frame >= AVDISCARD_NONREF || ctx->skip_nonref))
        return 0;

    // The decoder reads the bitstream from memory on its own: give it an aligned copy.
    if (size + AV_INPUT_BUFFER_PADDING_SIZE > ctx->bitstream_size)
    {
        int alloc = FFALIGN(size + AV_INPUT_BUFFER_PADDING_SIZE, 64 * 1024);
        free(ctx->bitstream);
        ctx->bitstream = memalign(H264_MEM_ALIGNMENT, alloc);
        ctx->bitstream_size = ctx->bitstream ? alloc : 0;
        if (!ctx->bitstream)
            return AVERROR(ENOMEM);
    }
    memcpy(ctx->bitstream, avpkt->data, size);
    memset(ctx->bitstream + size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
    DCFlushRange(ctx->bitstream, size + AV_INPUT_BUFFER_PADDING_SIZE);

    ctx->sent_pts[ctx->sent_index] = avpkt->pts;
    ctx->sent_index = (ctx->sent_index + 1) % (int) FF_ARRAY_ELEMS(ctx->sent_pts);
    res = H264DECSetBitstream(ctx->decoder, ctx->bitstream, size, (double) avpkt->pts);
    if (res != 0)
    {
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: Error setting bitstream 0x%07X\n", res);
        return AVERROR_INVALIDDATA;
    }

    // A picture of the current size goes straight into a frame of the caller's; one that can
    // change the size into the decoder's own buffer, which holds any size.
    slot = wiiu_target_slot(&ctx->targets, NULL);
    if (!h264_wiiu_new_sequence(ctx, avpkt->data, size) && slot)
        direct = h264_wiiu_direct_frame(avctx, ctx);
    if (direct)
        *slot = direct; // retained even when Execute fails or brings no picture
    else if (!slot && !ctx->targets_reported) {
        ctx->targets_reported = 1;
        av_log(avctx, AV_LOG_WARNING, "h264_wiiu: pending targets full; using persistent buffer\n");
    }
    target = direct ? direct->data[0] : ctx->framebuffer;
    res = h264_wiiu_execute(avctx, avpkt, skippable, target);
    h264_wiiu_stats(avctx, ctx);
    return res;
}

// Index of the queued picture to return next: the earliest one.
static int h264_wiiu_next(WIIUContext* ctx)
{
    int i, best = 0;

    if (ctx->queue[0]->pts == AV_NOPTS_VALUE)
        return 0;
    for (i = 1; i < ctx->queue_count; i++)
    {
        int64_t pts = ctx->queue[i]->pts;
        if (pts != AV_NOPTS_VALUE && pts < ctx->queue[best]->pts)
            best = i;
    }
    return best;
}

static int h264_wiiu_decode_frame(AVCodecContext *avctx, void *data, int *got_frame, AVPacket *avpkt)
{
    WIIUContext* ctx = avctx->priv_data;
    AVFrame *avframe = data;
    int draining = avpkt->size <= 0;
    int ret = 0, i;

    if (!draining)
        ret = h264_wiiu_decode_packet(avctx, avpkt);

    if (ctx->queue_count > (draining ? 0 : ctx->delay))
    {
        i = h264_wiiu_next(ctx);
        av_frame_move_ref(avframe, ctx->queue[i]);
        av_frame_free(&ctx->queue[i]);
        memmove(&ctx->queue[i], &ctx->queue[i + 1], (ctx->queue_count - i - 1) * sizeof(ctx->queue[0]));
        ctx->queue_count--;
        if (avframe->pts != AV_NOPTS_VALUE)
            ctx->last_pts = avframe->pts;
        *got_frame = 1;
    }

    if (ret < 0 && !*got_frame)
        return ret;
    return draining ? 0 : avpkt->size;
}

AVCodec ff_h264_wiiu_decoder =
{
    .name           = "h264_wiiu",
    .long_name      = NULL_IF_CONFIG_SMALL("H.264 Decoder using h264.rpl"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_H264,
    .priv_data_size = sizeof(WIIUContext),
    .init           = h264_wiiu_decode_init,
    .close          = h264_wiiu_decode_close,
    .decode         = h264_wiiu_decode_frame,
    .flush          = h264_wiiu_flush,
    .capabilities   = AV_CODEC_CAP_DELAY | AV_CODEC_CAP_DR1,
    .bsfs           = "h264_mp4toannexb",
};
