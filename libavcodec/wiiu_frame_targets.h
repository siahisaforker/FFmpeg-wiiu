// Output targets remain owned until their decoder callback, or decoder drain.
#ifndef WIIU_FRAME_TARGETS_H
#define WIIU_FRAME_TARGETS_H
#include "libavutil/frame.h"
#define WIIU_MAX_TARGETS 8
typedef struct WiiUFrameTargets {
    AVFrame* frames[WIIU_MAX_TARGETS];
} WiiUFrameTargets;
static inline AVFrame** wiiu_target_slot(WiiUFrameTargets* targets, const void* data) {
    int i;
    for (i = 0; i < WIIU_MAX_TARGETS; ++i) {
        AVFrame* f = targets->frames[i];
        if (data ? f && f->data[0] == data : !f)
            return &targets->frames[i];
    }
    return NULL;
}
// Call only after H264DECEnd/Close: no decoder writes may remain in flight.
static inline void wiiu_targets_clear(WiiUFrameTargets* targets) {
    int i;
    for (i = 0; i < WIIU_MAX_TARGETS; ++i)
        av_frame_free(&targets->frames[i]);
}
#endif
