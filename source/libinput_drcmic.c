/* The GamePad's microphone. See input_drcmic.h.
 *
 * mic.rpl keeps a ring of samples in memory it is given: MICGetStatus says
 * where the unread ones start and how many there are, MICSetDataConsumed
 * hands them back. */

#ifdef HAVE_MIC

#include <string.h>

#include <coreinit/cache.h>
#include <coreinit/memdefaultheap.h>
#include <mic/mic.h>

#include "input_drcmic.h"

#define MIC_SAMPLES 0x2800      /* the least the library takes */
#define MIC_STATE_SAMPLE_RATE 0 /* as decaf names the states */
#define DEFAULT_RATE 32000

static MICHandle handle = -1;
static MICWorkMemory work;
static bool opened, running;
static uint32_t rate = DEFAULT_RATE, drops;
static int (*dm_logger)(const char *fmt, ...);

#define dm_log(...) do { if (dm_logger) dm_logger(__VA_ARGS__); } while (0)

void DM_SetLog(int (*print)(const char *fmt, ...)) { dm_logger = print; }

int32_t DM_Open(int gamepad)
{
    MICError err = MIC_ERROR_OK;
    uint32_t value = 0;

    if (opened)
        return DM_ESTATE;
    if (gamepad < 0 || gamepad > 1)
        return DM_EINVAL;
    work.sampleMaxCount = MIC_SAMPLES;
    work.sampleBuffer = MEMAllocFromDefaultHeapEx(MIC_SAMPLES * sizeof(int16_t), 0x40);
    if (!work.sampleBuffer)
        return DM_ENOMEM;
    handle = MICInit(gamepad ? MIC_INSTANCE_1 : MIC_INSTANCE_0, 0, &work, &err);
    if (handle < 0 || err != MIC_ERROR_OK) {
        dm_log("GamePad mic: init failed (%d)\n", (int)err);
        MEMFreeToDefaultHeap(work.sampleBuffer);
        work.sampleBuffer = NULL;
        handle = -1;
        return DM_ENODEV;
    }
    rate = DEFAULT_RATE;
    if (MICGetState(handle, MIC_STATE_SAMPLE_RATE, &value) == MIC_ERROR_OK && value >= 8000 && value <= 96000)
        rate = value;
    opened = true;
    drops = 0;
    dm_log("GamePad mic: %u Hz\n", (unsigned)rate);
    return DM_OK;
}

int32_t DM_Start(void)
{
    MICError err;

    if (!opened)
        return DM_ESTATE;
    if (running)
        return DM_OK;
    err = MICOpen(handle);
    if (err != MIC_ERROR_OK) {
        dm_log("GamePad mic: open failed (%d)\n", (int)err);
        return DM_ENODEV;
    }
    running = true;
    return DM_OK;
}

int32_t DM_Stop(void)
{
    if (!running)
        return DM_OK;
    MICClose(handle);
    running = false;
    return DM_OK;
}

int32_t DM_Close(void)
{
    if (!opened)
        return DM_ESTATE;
    DM_Stop();
    MICUninit(handle);
    MEMFreeToDefaultHeap(work.sampleBuffer);
    work.sampleBuffer = NULL;
    handle = -1;
    opened = false;
    return DM_OK;
}

bool DM_IsOpen(void) { return opened; }
uint32_t DM_Rate(void) { return rate; }
uint32_t DM_Dropped(void) { return drops; }

uint32_t DM_Available(void)
{
    MICStatus st;

    if (!running || MICGetStatus(handle, &st) != MIC_ERROR_OK || st.availableData < 0)
        return 0;
    return (uint32_t)st.availableData;
}

uint32_t DM_Read(int16_t *dst, uint32_t max_samples)
{
    MICStatus st;
    const int16_t *ring = work.sampleBuffer;
    uint32_t n, pos;

    if (!running || !dst || MICGetStatus(handle, &st) != MIC_ERROR_OK || st.availableData <= 0)
        return 0;
    n = (uint32_t)st.availableData;
    if (n > max_samples) {
        /* What does not fit is let go, so the system's ring keeps moving. */
        drops += n - max_samples;
        n = max_samples;
    }
    pos = (uint32_t)st.bufferPos % MIC_SAMPLES;
    DCInvalidateRange(work.sampleBuffer, MIC_SAMPLES * sizeof(int16_t));
    for (uint32_t i = 0; i < n; i++)
        dst[i] = ring[(pos + i) % MIC_SAMPLES];
    MICSetDataConsumed(handle, st.availableData);
    return n;
}

#endif /* HAVE_MIC */
