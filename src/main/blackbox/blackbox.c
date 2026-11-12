/*
 * This file is part of St4rpony (derived from Betaflight).
 *
 * St4rpony is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * St4rpony is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_BLACKBOX

#include "blackbox.h"
#include "blackbox_fielddefs.h"
#include "blackbox_io.h"
#include "blackbox_sbl.h"

#include "build/build_config.h"
#include "build/version.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/sdcard.h"
#include "drivers/time.h"

#include "fc/board_info.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/ladrc.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "io/beeper.h"
#include "io/serial.h"

#include "pg/dyn_notch.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/pilot.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#if !defined(DEFAULT_BLACKBOX_DEVICE)
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_NONE
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 4);

PG_RESET_TEMPLATE(blackboxConfig_t, blackboxConfig,
    .fields_disabled_mask = 0, // 0 = Full .SBL stream, nonzero = Compact .SBL stream
    .sample_rate = BLACKBOX_RATE_QUARTER,
    .device = DEFAULT_BLACKBOX_DEVICE,
    .mode = BLACKBOX_MODE_NORMAL,
    .high_resolution = false
);

STATIC_ASSERT((sizeof(blackboxConfig()->fields_disabled_mask) * 8) >= FLIGHT_LOG_FIELD_SELECT_COUNT, too_many_flight_log_fields_selections);
STATIC_ASSERT(sizeof(sblChunkHdr_t) == 8, sbl_chunk_hdr_size);
STATIC_ASSERT(sizeof(sblSessionHeader_t) == 152, sbl_session_hdr_size);
STATIC_ASSERT(sizeof(sblFrameFullHead_t) == 74, sbl_full_frame_head_size);
STATIC_ASSERT(sizeof(sblFrameCompactHead_t) == 50, sbl_compact_frame_head_size);
STATIC_ASSERT(sizeof(sblEventPayload_t) == 16, sbl_event_size);
STATIC_ASSERT(sizeof(sblSessionFooter_t) == 16, sbl_footer_size);

#define BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS 600
#define SBL_RING_BYTES                   4096
#define SBL_TX_BUF_BYTES                 512
#define SBL_EVENT_QUEUE_CAP              8

typedef enum {
    BLACKBOX_STATE_DISABLED = 0,
    BLACKBOX_STATE_STOPPED,
    BLACKBOX_STATE_PREPARE_LOG_FILE,
    BLACKBOX_STATE_SEND_HEADER,
    BLACKBOX_STATE_CACHE_FLUSH,
    BLACKBOX_STATE_PAUSED,
    BLACKBOX_STATE_RUNNING,
    BLACKBOX_STATE_SHUTTING_DOWN,
    BLACKBOX_STATE_START_ERASE,
    BLACKBOX_STATE_ERASING,
    BLACKBOX_STATE_ERASED
} blackboxState_e;

// From rc_controls.c
extern boxBitmask_t rcModeActivationMask;

static blackboxState_e blackboxState = BLACKBOX_STATE_DISABLED;

static uint32_t blackboxLastArmingBeep = 0;
static uint32_t blackboxLastFlightModeFlags = 0;

static uint32_t sblStateStartTimeMs = 0;

STATIC_UNIT_TESTED int16_t blackboxIInterval = 0;
STATIC_UNIT_TESTED int8_t blackboxPInterval = 0;
STATIC_UNIT_TESTED int32_t blackboxSInterval = 0;
STATIC_UNIT_TESTED int32_t blackboxSlowFrameIterationTimer = 0;

static uint32_t blackboxIteration = 0;
static uint16_t blackboxLoopIndex = 0;
static uint16_t blackboxPFrameIndex = 0;
static uint16_t blackboxIFrameIndex = 0;
static bool blackboxLoggedAnyFrames = false;
static bool blackboxModeActivationConditionPresent = false;
static bool startedLoggingInTestMode = false;

// Normal SRAM buffers (explicitly NOT FASTRAM/CCMRAM so SPI DMA can read directly on STM32F405)
static uint8_t sblRing[SBL_RING_BYTES];
static uint16_t sblRingHead = 0;
static uint16_t sblRingTail = 0;
static uint16_t sblRingCount = 0;
static uint16_t sblRingCapacity = 0;

static uint8_t sblTxBuf[SBL_TX_BUF_BYTES];
static uint16_t sblTxPos = 0;
static uint16_t sblTxLen = 0;

static sblEventPayload_t sblEventQueue[SBL_EVENT_QUEUE_CAP];
static uint8_t sblEventHead = 0;
static uint8_t sblEventTail = 0;
static uint8_t sblEventCount = 0;

static uint16_t sblChunkSeq = 0;
static uint8_t sblMotorCount = 4;
static uint8_t sblFrameSize = 0;
static bool sblCompactMode = false;
static bool sblFooterQueued = false;
static uint32_t sblTotalFrames = 0;
static uint32_t sblDroppedFrames = 0;
static uint32_t sblLastTimeUs = 0;

static inline int16_t sblClampI16(float value)
{
    return (int16_t)lrintf(constrainf(value, -32767.0f, 32767.0f));
}

static inline uint16_t sblClampU16(float value)
{
    return (uint16_t)lrintf(constrainf(value, 0.0f, 65535.0f));
}

static void sblCopyFixedStr(char *dst, size_t dstSize, const char *src)
{
    if (!dst || dstSize == 0) {
        return;
    }
    memset(dst, 0, dstSize);
    if (!src) {
        return;
    }
    for (size_t i = 0; i < dstSize && src[i] != '\0'; i++) {
        dst[i] = src[i];
    }
}

#ifndef UNIT_TEST
static size_t sblAppendFixedStr(char *dst, size_t dstSize, size_t pos, const char *src)
{
    if (!dst || !src || pos >= dstSize) {
        return pos;
    }
    for (size_t i = 0; src[i] != '\0' && pos < dstSize; i++) {
        dst[pos++] = src[i];
    }
    return pos;
}
#endif

static void sblResetStreamBuffers(void)
{
    sblRingHead = 0;
    sblRingTail = 0;
    sblRingCount = 0;
    sblTxPos = 0;
    sblTxLen = 0;
    sblEventHead = 0;
    sblEventTail = 0;
    sblEventCount = 0;
    sblChunkSeq = 0;
    sblFooterQueued = false;
    sblTotalFrames = 0;
    sblDroppedFrames = 0;
    sblLastTimeUs = 0;

    sblCompactMode = (blackboxConfig()->fields_disabled_mask != 0);
#ifndef UNIT_TEST
    sblMotorCount = (uint8_t)MIN((int)getMotorCount(), (int)SBL_MAX_MOTORS);
#else
    sblMotorCount = 4;
#endif
    sblFrameSize = (uint8_t)(sblCompactMode ? SBL_COMPACT_FRAME_SIZE(sblMotorCount) : SBL_FULL_FRAME_SIZE(sblMotorCount));
    sblRingCapacity = (sblFrameSize > 0) ? (uint16_t)(sizeof(sblRing) / sblFrameSize) : 0;
}

static uint8_t sblChunkFlags(void)
{
    return (uint8_t)((sblCompactMode ? SBL_FLAG_COMPACT : 0U) |
        (((uint8_t)sblMotorCount << SBL_FLAG_MOTOR_COUNT_SHIFT) & SBL_FLAG_MOTOR_COUNT_MASK));
}

static bool sblStageChunk(uint8_t type, uint8_t flags, const uint8_t *payload, uint16_t payloadLen)
{
    const uint32_t totalLen = sizeof(sblChunkHdr_t) + payloadLen + sizeof(uint16_t);
    if (totalLen > sizeof(sblTxBuf)) {
        return false;
    }

    sblChunkHdr_t hdr;
    hdr.magic = SBL_CHUNK_MAGIC_LE;
    hdr.type = type;
    hdr.flags = flags;
    hdr.payloadLen = payloadLen;
    hdr.seq = sblChunkSeq++;

    memcpy(sblTxBuf, &hdr, sizeof(hdr));
    if (payloadLen > 0 && payload) {
        memcpy(sblTxBuf + sizeof(hdr), payload, payloadLen);
    }

    const uint16_t crc = sblCrc16Ccitt(sblTxBuf + 2, sizeof(hdr) - 2 + payloadLen);
    sblTxBuf[sizeof(hdr) + payloadLen] = (uint8_t)(crc & 0xFFU);
    sblTxBuf[sizeof(hdr) + payloadLen + 1] = (uint8_t)((crc >> 8) & 0xFFU);

    sblTxPos = 0;
    sblTxLen = (uint16_t)totalLen;
    return true;
}

static bool sblDrainTxBuf(void)
{
    while (sblTxPos < sblTxLen) {
        const uint32_t written = blackboxDeviceWrite(sblTxBuf + sblTxPos, (uint32_t)(sblTxLen - sblTxPos));
        if (written == 0) {
            return false;
        }
        sblTxPos = (uint16_t)(sblTxPos + written);
    }
    return true;
}

// Buf must be at least FORMATTED_DATE_TIME_BUFSIZE
STATIC_UNIT_TESTED char *blackboxGetStartDateTime(char *buf)
{
#ifdef USE_RTC_TIME
    dateTime_t dt;
    rtcGetDateTime(&dt);
    dateTimeFormatLocal(buf, &dt);
#else
    memcpy(buf, "0000-01-01T00:00:00.000", 24);
#endif
    return buf;
}

static void sblPopulateSessionHeader(sblSessionHeader_t *hdr)
{
    memset(hdr, 0, sizeof(*hdr));
    hdr->version = SBL_FORMAT_VERSION;
    hdr->compact = sblCompactMode ? 1U : 0U;
    hdr->sampleDenom = (blackboxPInterval > 0) ? (uint8_t)blackboxPInterval : (uint8_t)MAX(1, (int)blackboxIInterval);
    hdr->motorCount = sblMotorCount;

#ifndef UNIT_TEST
    hdr->pidLooptimeUs = targetPidLooptime;
    hdr->gyroSampleHz = (gyro.sampleLooptime > 0) ? (uint16_t)MIN(65535U, 1000000U / (uint32_t)gyro.sampleLooptime) : 0U;
    hdr->vbatRefMv = (uint16_t)MIN(65535U, (uint32_t)getBatteryVoltageLatest() * 10U);
    if (currentBatteryProfile) {
        hdr->vbatMinCellMv = (uint16_t)(currentBatteryProfile->vbatmincellvoltage * 10U);
        hdr->vbatWarnCellMv = (uint16_t)(currentBatteryProfile->vbatwarningcellvoltage * 10U);
        hdr->vbatMaxCellMv = (uint16_t)(currentBatteryProfile->vbatmaxcellvoltage * 10U);
    }
    hdr->motorOutputLow = (int16_t)lrintf(getMotorOutputLow());
    hdr->motorOutputHigh = (int16_t)lrintf(getMotorOutputHigh());
    hdr->mixerType = mixerConfig()->mixer_type;
    hdr->fixedWing = isFixedWing() ? 1U : 0U;

    const ladrcProfile_t *ladrc = ladrcProfiles(systemConfig()->pidProfileIndex);
    if (ladrc) {
        hdr->antiWindup = ladrc->anti_windup;
        hdr->vbatB0Comp = ladrc->vbat_b0_comp;
        hdr->airspeedSched = ladrc->airspeed_sched;
        hdr->airspeedRefCms = ladrc->airspeed_ref_cms;
        hdr->distLimitPermille = ladrc->disturbance_limit;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            hdr->axis[axis].order = ladrc->axis[axis].order;
            hdr->axis[axis].wcDeci = ladrc->axis[axis].wc_deci;
            hdr->axis[axis].woDeci = ladrc->axis[axis].wo_deci;
            hdr->axis[axis].b0Deci = ladrc->axis[axis].b0_deci;
            hdr->axis[axis].tdHz = ladrc->axis[axis].td_hz;
        }
    }

    hdr->gyroLpf1StaticHz = gyroConfig()->gyro_lpf1_static_hz;
#ifdef USE_DYN_LPF
    hdr->gyroLpf1DynMinHz = gyroConfig()->gyro_lpf1_dyn_min_hz;
    hdr->gyroLpf1DynMaxHz = gyroConfig()->gyro_lpf1_dyn_max_hz;
#endif
    hdr->gyroLpf2StaticHz = gyroConfig()->gyro_lpf2_static_hz;
#ifdef USE_DYN_NOTCH_FILTER
    hdr->dynNotchCount = dynNotchConfig()->dyn_notch_count;
    hdr->dynNotchMinHz = dynNotchConfig()->dyn_notch_min_hz;
    hdr->dynNotchMaxHz = dynNotchConfig()->dyn_notch_max_hz;
#endif

    sblCopyFixedStr(hdr->craftName, sizeof(hdr->craftName), pilotConfig()->craftName);

    size_t pos = 0;
#ifdef USE_BOARD_INFO
    pos = sblAppendFixedStr(hdr->boardInfo, sizeof(hdr->boardInfo), pos, getManufacturerId());
    pos = sblAppendFixedStr(hdr->boardInfo, sizeof(hdr->boardInfo), pos, " ");
    sblAppendFixedStr(hdr->boardInfo, sizeof(hdr->boardInfo), pos, getBoardName());
#else
    sblCopyFixedStr(hdr->boardInfo, sizeof(hdr->boardInfo), targetName);
#endif

    pos = 0;
    pos = sblAppendFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), pos, FC_FIRMWARE_NAME);
    pos = sblAppendFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), pos, " ");
    pos = sblAppendFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), pos, FC_VERSION_STRING);
    pos = sblAppendFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), pos, " ");
    sblAppendFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), pos, targetName);

    char dtBuf[FORMATTED_DATE_TIME_BUFSIZE];
    sblCopyFixedStr(hdr->startDatetime, sizeof(hdr->startDatetime), blackboxGetStartDateTime(dtBuf));
#else
    hdr->pidLooptimeUs = targetPidLooptime;
    hdr->gyroSampleHz = (targetPidLooptime > 0) ? (uint16_t)(1000000U / targetPidLooptime) : 1000U;
    hdr->motorOutputLow = (int16_t)lrintf(getMotorOutputLow());
    hdr->motorOutputHigh = (int16_t)lrintf(getMotorOutputHigh());
    sblCopyFixedStr(hdr->craftName, sizeof(hdr->craftName), "St4rponyTest");
    sblCopyFixedStr(hdr->boardInfo, sizeof(hdr->boardInfo), "SPBE TEST");
    sblCopyFixedStr(hdr->fwRevision, sizeof(hdr->fwRevision), "St4rpony SBL1");
    sblCopyFixedStr(hdr->startDatetime, sizeof(hdr->startDatetime), "2026-11-11T12:00:00");
#endif
}

static bool sblCaptureFrame(timeUs_t currentTimeUs)
{
    if (sblRingCapacity == 0 || sblFrameSize == 0) {
        return false;
    }
    if (sblRingCount >= sblRingCapacity) {
        sblDroppedFrames++;
        return false;
    }

    uint8_t *slot = sblRing + ((size_t)sblRingHead * sblFrameSize);

#ifndef UNIT_TEST
    const float mixAuth = mixerGetAuthorityScale();
    const bool stabilized = pidIsStabilisationEnabled();
    const int16_t throttleVal = (int16_t)lrintf(constrainf(mixerGetThrottle() * 1000.0f, 0.0f, 1000.0f));
    const uint16_t vbatMv = (uint16_t)MIN(65535U, (uint32_t)getBatteryVoltageLatest() * 10U);
    const uint16_t mixAuthPermille = sblClampU16(mixAuth * 1000.0f);

    uint8_t resetFlags = 0;
    float repB0Scale = 1.0f;

    if (sblCompactMode) {
        sblFrameCompactHead_t f;
        f.timeUs = currentTimeUs;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const ladrcRuntimeAxis_t *st = ladrcGetAxisState(axis);
            f.gyroFiltDds[axis] = sblClampI16(gyro.gyroADCf[axis] * 10.0f);
            f.setpointDds[axis] = sblClampI16(pidGetPreviousSetpoint(axis) * 10.0f);
            if (st) {
                f.tdRateDds[axis] = sblClampI16(st->rLastNorm * 10000.0f);
                f.z1Dds[axis] = sblClampI16(st->z1Last * 10000.0f);
                f.z3DistPermille[axis] = sblClampI16(st->distTermSum);
                f.uCmdPermille[axis] = sblClampI16(st->uCmdNorm * 1000.0f);
                resetFlags |= (uint8_t)((st->resetMode & SBL_RESET_AXIS_MASK) << (axis * 2));
                if (axis == FD_ROLL) {
                    repB0Scale = st->b0Eff / MAX(0.05f, st->b0Nom);
                }
            } else {
                f.tdRateDds[axis] = 0;
                f.z1Dds[axis] = 0;
                f.z3DistPermille[axis] = 0;
                f.uCmdPermille[axis] = 0;
            }
        }
        if (mixAuth < 0.995f) {
            resetFlags |= SBL_RESET_SATURATED_BIT;
        }
        if (stabilized) {
            resetFlags |= SBL_RESET_STABILISED_BIT;
        }
        f.b0ScalePermille = sblClampU16(repB0Scale * 1000.0f);
        f.mixAuthPermille = mixAuthPermille;
        f.vbatMv = vbatMv;
        f.rcThrottle = throttleVal;
        f.resetFlags = resetFlags;
        f.reserved = 0;

        memcpy(slot, &f, sizeof(f));
        slot += sizeof(f);
    } else {
        sblFrameFullHead_t f;
        f.timeUs = currentTimeUs;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            const ladrcRuntimeAxis_t *st = ladrcGetAxisState(axis);
            f.gyroRawDds[axis] = sblClampI16(gyro.gyroADC[axis] * 10.0f);
            f.gyroFiltDds[axis] = sblClampI16(gyro.gyroADCf[axis] * 10.0f);
            f.setpointDds[axis] = sblClampI16(pidGetPreviousSetpoint(axis) * 10.0f);
            if (st) {
                f.tdRateDds[axis] = sblClampI16(st->rLastNorm * 10000.0f);
                f.tdRateDotCkrps[axis] = sblClampI16(st->rDotLast * 100.0f);
                f.z1Dds[axis] = sblClampI16(st->z1Last * 10000.0f);
                f.z2Ckrps[axis] = sblClampI16(st->z2Last * 100.0f);
                f.z3DistPermille[axis] = sblClampI16(st->distTermSum);
                f.pdTermPermille[axis] = sblClampI16(st->pTermSum + st->dTermSum);
                f.uCmdPermille[axis] = sblClampI16(st->uCmdNorm * 1000.0f);
                resetFlags |= (uint8_t)((st->resetMode & SBL_RESET_AXIS_MASK) << (axis * 2));
                if (axis == FD_ROLL) {
                    repB0Scale = st->b0Eff / MAX(0.05f, st->b0Nom);
                }
            } else {
                f.tdRateDds[axis] = 0;
                f.tdRateDotCkrps[axis] = 0;
                f.z1Dds[axis] = 0;
                f.z2Ckrps[axis] = 0;
                f.z3DistPermille[axis] = 0;
                f.pdTermPermille[axis] = 0;
                f.uCmdPermille[axis] = 0;
            }
        }
        if (mixAuth < 0.995f) {
            resetFlags |= SBL_RESET_SATURATED_BIT;
        }
        if (stabilized) {
            resetFlags |= SBL_RESET_STABILISED_BIT;
        }
        f.b0ScalePermille = sblClampU16(repB0Scale * 1000.0f);
        f.mixAuthPermille = mixAuthPermille;
        f.vbatMv = vbatMv;
        f.rcThrottle = throttleVal;
        f.resetFlags = resetFlags;
        f.reserved = 0;

        memcpy(slot, &f, sizeof(f));
        slot += sizeof(f);
    }

    for (uint8_t m = 0; m < sblMotorCount; m++) {
        const int16_t motVal = sblClampI16(motor[m]);
        memcpy(slot + (size_t)m * sizeof(int16_t), &motVal, sizeof(int16_t));
    }
#else
    memset(slot, 0, sblFrameSize);
    memcpy(slot, &currentTimeUs, sizeof(currentTimeUs));
#endif

    sblRingHead = (uint16_t)((sblRingHead + 1U) % sblRingCapacity);
    sblRingCount++;
    sblTotalFrames++;
    sblLastTimeUs = currentTimeUs;
    blackboxLoggedAnyFrames = true;
    return true;
}

static void sblPumpOutput(bool forceBatchImmediate)
{
    if (!sblDrainTxBuf()) {
        blackboxDeviceFlush();
        return;
    }

    if (sblEventCount > 0) {
        const sblEventPayload_t ev = sblEventQueue[sblEventTail];
        sblEventTail = (uint8_t)((sblEventTail + 1U) % SBL_EVENT_QUEUE_CAP);
        sblEventCount--;
        sblStageChunk(SBL_CHUNK_EVENT, sblChunkFlags(), (const uint8_t *)&ev, sizeof(ev));
        if (!sblDrainTxBuf()) {
            blackboxDeviceFlush();
        }
        return;
    }

    if (sblRingCount > 0 && sblFrameSize > 0) {
        const uint16_t maxPayloadBytes = (uint16_t)(sizeof(sblTxBuf) - sizeof(sblChunkHdr_t) - sizeof(uint16_t));
        const uint16_t maxChunkFrames = MIN((uint16_t)(maxPayloadBytes / sblFrameSize), (uint16_t)(sblCompactMode ? 6U : 4U));
        const uint16_t minBatchFrames = forceBatchImmediate ? 1U : MIN((uint16_t)2U, maxChunkFrames);

        if (sblRingCount >= minBatchFrames) {
            const uint16_t count = MIN(sblRingCount, maxChunkFrames);
            sblChunkHdr_t hdr;
            hdr.magic = SBL_CHUNK_MAGIC_LE;
            hdr.type = SBL_CHUNK_LADRC_BATCH;
            hdr.flags = sblChunkFlags();
            hdr.payloadLen = (uint16_t)(count * sblFrameSize);
            hdr.seq = sblChunkSeq++;

            memcpy(sblTxBuf, &hdr, sizeof(hdr));
            uint8_t *dst = sblTxBuf + sizeof(hdr);
            for (uint16_t i = 0; i < count; i++) {
                const uint8_t *src = sblRing + ((size_t)sblRingTail * sblFrameSize);
                memcpy(dst, src, sblFrameSize);
                dst += sblFrameSize;
                sblRingTail = (uint16_t)((sblRingTail + 1U) % sblRingCapacity);
                sblRingCount--;
            }

            const uint16_t crc = sblCrc16Ccitt(sblTxBuf + 2, sizeof(hdr) - 2 + hdr.payloadLen);
            sblTxBuf[sizeof(hdr) + hdr.payloadLen] = (uint8_t)(crc & 0xFFU);
            sblTxBuf[sizeof(hdr) + hdr.payloadLen + 1] = (uint8_t)((crc >> 8) & 0xFFU);

            sblTxPos = 0;
            sblTxLen = (uint16_t)(sizeof(hdr) + hdr.payloadLen + sizeof(uint16_t));
            if (!sblDrainTxBuf()) {
                blackboxDeviceFlush();
            }
        }
    }
}

bool blackboxMayEditConfig(void)
{
    return blackboxState <= BLACKBOX_STATE_STOPPED;
}

static void blackboxSetState(blackboxState_e newState)
{
    switch (newState) {
    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        blackboxLoggedAnyFrames = false;
        sblResetStreamBuffers();
        break;
    case BLACKBOX_STATE_SEND_HEADER: {
        sblSessionHeader_t hdr;
        sblPopulateSessionHeader(&hdr);
        sblStageChunk(SBL_CHUNK_SESSION_HEADER, sblChunkFlags(), (const uint8_t *)&hdr, sizeof(hdr));
        sblStateStartTimeMs = millis();
        break;
    }
    case BLACKBOX_STATE_SHUTTING_DOWN:
        sblStateStartTimeMs = millis();
        break;
    default:
        break;
    }
    blackboxState = newState;
}

void blackboxValidateConfig(void)
{
    switch (blackboxConfig()->device) {
#ifdef USE_FLASHFS
    case BLACKBOX_DEVICE_FLASH:
#endif
#ifdef USE_SDCARD
    case BLACKBOX_DEVICE_SDCARD:
#endif
#ifdef USE_BLACKBOX_VIRTUAL
    case BLACKBOX_DEVICE_VIRTUAL:
#endif
    case BLACKBOX_DEVICE_SERIAL:
        break;
    default:
        blackboxConfigMutable()->device = BLACKBOX_DEVICE_NONE;
        break;
    }
}

static void blackboxResetIterationTimers(void)
{
    blackboxIteration = 0;
    blackboxLoopIndex = 0;
    blackboxIFrameIndex = 0;
    blackboxPFrameIndex = 0;
    blackboxSlowFrameIterationTimer = 0;
}

static void blackboxStart(void)
{
    blackboxValidateConfig();

    if (!blackboxDeviceOpen()) {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
        return;
    }

    blackboxModeActivationConditionPresent = isModeActivationConditionPresent(BOXBLACKBOX);
    blackboxResetIterationTimers();
    blackboxLastArmingBeep = getArmingBeepTimeMicros();
    memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));

    blackboxSetState(BLACKBOX_STATE_PREPARE_LOG_FILE);
}

void blackboxFinish(void)
{
    switch (blackboxState) {
    case BLACKBOX_STATE_DISABLED:
    case BLACKBOX_STATE_STOPPED:
    case BLACKBOX_STATE_SHUTTING_DOWN:
        break;
    case BLACKBOX_STATE_RUNNING:
    case BLACKBOX_STATE_PAUSED:
        blackboxLogEvent(FLIGHT_LOG_EVENT_LOG_END, NULL);
        FALLTHROUGH;
    default:
        blackboxSetState(BLACKBOX_STATE_SHUTTING_DOWN);
        break;
    }
}

void blackboxLogEvent(FlightLogEvent event, flightLogEventData_t *data)
{
    if (!(blackboxState == BLACKBOX_STATE_RUNNING || blackboxState == BLACKBOX_STATE_PAUSED || blackboxState == BLACKBOX_STATE_SHUTTING_DOWN)) {
        return;
    }
    if (sblEventCount >= SBL_EVENT_QUEUE_CAP) {
        return;
    }

    sblEventPayload_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.timeUs = sblLastTimeUs ? sblLastTimeUs : micros();
    ev.eventType = (uint8_t)event;

    if (data) {
        switch (event) {
        case FLIGHT_LOG_EVENT_SYNC_BEEP:
            ev.value0 = data->syncBeep.time;
            break;
        case FLIGHT_LOG_EVENT_FLIGHTMODE:
            ev.value0 = data->flightMode.flags;
            ev.value1 = data->flightMode.lastFlags;
            break;
        case FLIGHT_LOG_EVENT_DISARM:
            ev.value0 = data->disarm.reason;
            break;
        case FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT:
            ev.paramByte = (uint8_t)(data->inflightAdjustment.adjustmentFunction |
                (data->inflightAdjustment.floatFlag ? FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT_FUNCTION_FLOAT_VALUE_FLAG : 0U));
            if (data->inflightAdjustment.floatFlag) {
                memcpy(&ev.value0, &data->inflightAdjustment.newFloatValue, sizeof(ev.value0));
            } else {
                ev.value0 = (uint32_t)data->inflightAdjustment.newValue;
            }
            break;
        case FLIGHT_LOG_EVENT_LOGGING_RESUME:
            ev.value0 = data->loggingResume.logIteration;
            ev.value1 = data->loggingResume.currentTime;
            break;
        default:
            break;
        }
    }

    sblEventQueue[sblEventHead] = ev;
    sblEventHead = (uint8_t)((sblEventHead + 1U) % SBL_EVENT_QUEUE_CAP);
    sblEventCount++;
}

static void blackboxCheckAndLogArmingBeep(void)
{
    if (getArmingBeepTimeMicros() != blackboxLastArmingBeep) {
        blackboxLastArmingBeep = getArmingBeepTimeMicros();
        flightLogEventData_t eventData;
        eventData.syncBeep.time = blackboxLastArmingBeep;
        blackboxLogEvent(FLIGHT_LOG_EVENT_SYNC_BEEP, &eventData);
    }
}

static void blackboxCheckAndLogFlightMode(void)
{
    if (memcmp(&rcModeActivationMask, &blackboxLastFlightModeFlags, sizeof(blackboxLastFlightModeFlags))) {
        flightLogEventData_t eventData;
        eventData.flightMode.lastFlags = blackboxLastFlightModeFlags;
        memcpy(&blackboxLastFlightModeFlags, &rcModeActivationMask, sizeof(blackboxLastFlightModeFlags));
        memcpy(&eventData.flightMode.flags, &rcModeActivationMask, sizeof(eventData.flightMode.flags));
        blackboxLogEvent(FLIGHT_LOG_EVENT_FLIGHTMODE, &eventData);
    }
}

STATIC_UNIT_TESTED bool blackboxShouldLogPFrame(void)
{
    return blackboxPFrameIndex == 0 && blackboxPInterval != 0;
}

STATIC_UNIT_TESTED bool blackboxShouldLogIFrame(void)
{
    return blackboxLoopIndex == 0;
}

#ifdef UNIT_TEST
STATIC_UNIT_TESTED bool writeSlowFrameIfNeeded(void)
{
    const bool shouldWrite = blackboxSlowFrameIterationTimer >= blackboxSInterval;
    if (shouldWrite) {
        blackboxSlowFrameIterationTimer = 0;
    }
    return shouldWrite;
}
#endif

STATIC_UNIT_TESTED void blackboxAdvanceIterationTimers(void)
{
    ++blackboxSlowFrameIterationTimer;
    ++blackboxIteration;

    if (++blackboxLoopIndex >= blackboxIInterval) {
        blackboxLoopIndex = 0;
        blackboxIFrameIndex++;
        blackboxPFrameIndex = 0;
    } else if (++blackboxPFrameIndex >= blackboxPInterval) {
        blackboxPFrameIndex = 0;
    }
}

STATIC_UNIT_TESTED void blackboxLogIteration(timeUs_t currentTimeUs)
{
    blackboxCheckAndLogArmingBeep();
    blackboxCheckAndLogFlightMode();

    if (blackboxShouldLogPFrame() || blackboxShouldLogIFrame()) {
        sblCaptureFrame(currentTimeUs);
    }
    sblPumpOutput(false);
}

static bool inMotorTestMode(void)
{
    static uint32_t resetTime = 0;

    if (!ARMING_FLAG(ARMED) && areMotorsRunning()) {
        resetTime = millis() + 5000;
        return true;
    }
    return millis() < resetTime;
}

static void startInTestMode(void)
{
    if (!startedLoggingInTestMode) {
        if (blackboxConfig()->device == BLACKBOX_DEVICE_SERIAL) {
            serialPort_t *sharedBlackboxAndMspPort = findSharedSerialPort(FUNCTION_BLACKBOX, FUNCTION_MSP);
            if (sharedBlackboxAndMspPort) {
                return;
            }
        }
        blackboxStart();
        startedLoggingInTestMode = true;
    }
}

static void stopInTestMode(void)
{
    if (startedLoggingInTestMode) {
        blackboxFinish();
        startedLoggingInTestMode = false;
    }
}

static void blackboxUpdateArmed(timeUs_t currentTimeUs)
{
    switch (blackboxState) {
    case BLACKBOX_STATE_STOPPED:
        if (ARMING_FLAG(ARMED)) {
            blackboxOpen();
            blackboxStart();
        }
#ifdef USE_FLASHFS
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_START_ERASE);
        }
#endif
        break;

    case BLACKBOX_STATE_PREPARE_LOG_FILE:
        if (blackboxDeviceBeginLog()) {
            blackboxSetState(BLACKBOX_STATE_SEND_HEADER);
        }
        break;

    case BLACKBOX_STATE_SEND_HEADER:
        if (millis() > sblStateStartTimeMs + 25) {
            if (sblDrainTxBuf()) {
                blackboxSetState(BLACKBOX_STATE_CACHE_FLUSH);
            } else {
                blackboxDeviceFlush();
            }
        }
        break;

    case BLACKBOX_STATE_CACHE_FLUSH:
        if (blackboxDeviceFlushForceComplete()) {
            blackboxSetState(BLACKBOX_STATE_RUNNING);
        }
        break;

    case BLACKBOX_STATE_PAUSED:
        if (IS_RC_MODE_ACTIVE(BOXBLACKBOX) && (blackboxShouldLogPFrame() || blackboxShouldLogIFrame())) {
            flightLogEventData_t resumeData;
            resumeData.loggingResume.logIteration = blackboxIteration;
            resumeData.loggingResume.currentTime = currentTimeUs;
            blackboxLogEvent(FLIGHT_LOG_EVENT_LOGGING_RESUME, &resumeData);
            blackboxSetState(BLACKBOX_STATE_RUNNING);
            blackboxLogIteration(currentTimeUs);
        } else {
            sblPumpOutput(true);
        }
        blackboxAdvanceIterationTimers();
        break;

    case BLACKBOX_STATE_RUNNING:
        if (blackboxModeActivationConditionPresent && !IS_RC_MODE_ACTIVE(BOXBLACKBOX) && !startedLoggingInTestMode) {
            blackboxSetState(BLACKBOX_STATE_PAUSED);
        } else {
            blackboxLogIteration(currentTimeUs);
        }
        blackboxAdvanceIterationTimers();
        break;

    case BLACKBOX_STATE_SHUTTING_DOWN: {
        const bool timedOut = millis() > sblStateStartTimeMs + BLACKBOX_SHUTDOWN_TIMEOUT_MILLIS;

        if (!timedOut) {
            sblPumpOutput(true);
            if (!sblFooterQueued && sblTxPos >= sblTxLen && sblRingCount == 0 && sblEventCount == 0) {
                sblSessionFooter_t ftr;
                ftr.totalFrames = sblTotalFrames;
                ftr.droppedFrames = sblDroppedFrames;
                ftr.endTimeUs = sblLastTimeUs ? sblLastTimeUs : currentTimeUs;
#ifdef USE_SDCARD
                ftr.sdWriteErrors = sdcard_getWriteErrorCount();
#else
                ftr.sdWriteErrors = 0;
#endif
                sblStageChunk(SBL_CHUNK_SESSION_FOOTER, sblChunkFlags(), (const uint8_t *)&ftr, sizeof(ftr));
                sblFooterQueued = true;
                sblDrainTxBuf();
            }
        }

        if ((timedOut || (sblFooterQueued && sblTxPos >= sblTxLen)) &&
            blackboxDeviceEndLog(blackboxLoggedAnyFrames) &&
            (timedOut || blackboxDeviceFlushForceComplete())) {
            blackboxDeviceClose();
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        } else {
            blackboxDeviceFlush();
        }
        break;
    }

#ifdef USE_FLASHFS
    case BLACKBOX_STATE_START_ERASE:
        blackboxEraseAll();
        blackboxSetState(BLACKBOX_STATE_ERASING);
        break;
    case BLACKBOX_STATE_ERASING:
        if (isBlackboxErased()) {
            beeper(BEEPER_BLACKBOX_ERASE);
            blackboxSetState(BLACKBOX_STATE_ERASED);
        }
        break;
    case BLACKBOX_STATE_ERASED:
        if (!IS_RC_MODE_ACTIVE(BOXBLACKBOXERASE)) {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
        }
        break;
#endif

    default:
        break;
    }

    if (isBlackboxDeviceFull()) {
#ifdef USE_FLASHFS
        if (blackboxState != BLACKBOX_STATE_ERASING
            && blackboxState != BLACKBOX_STATE_START_ERASE
            && blackboxState != BLACKBOX_STATE_ERASED)
#endif
        {
            blackboxSetState(BLACKBOX_STATE_STOPPED);
            if (startedLoggingInTestMode) {
                startedLoggingInTestMode = false;
            }
        }
    } else {
        switch (blackboxConfig()->mode) {
        case BLACKBOX_MODE_MOTOR_TEST:
            if (inMotorTestMode()) {
                if (blackboxState == BLACKBOX_STATE_STOPPED) {
                    startInTestMode();
                }
            } else if (blackboxState != BLACKBOX_STATE_STOPPED) {
                stopInTestMode();
            }
            break;
        case BLACKBOX_MODE_ALWAYS_ON:
            if (blackboxState == BLACKBOX_STATE_STOPPED) {
                startInTestMode();
            }
            break;
        case BLACKBOX_MODE_NORMAL:
        default:
            break;
        }
    }
}

void blackboxUpdate(timeUs_t currentTimeUs)
{
    blackboxUpdateArmed(currentTimeUs);
}

int blackboxCalculatePDenom(int rateNum, int rateDenom)
{
    return blackboxIInterval * rateNum / rateDenom;
}

uint8_t blackboxGetRateDenom(void)
{
    return blackboxPInterval;
}

uint16_t blackboxGetPRatio(void)
{
    return blackboxPInterval ? (uint16_t)(blackboxIInterval / blackboxPInterval) : 1;
}

uint8_t blackboxCalculateSampleRate(uint16_t pRatio)
{
    return llog2(32000 / (targetPidLooptime * pRatio));
}

void blackboxInit(void)
{
    blackboxResetIterationTimers();
    sblResetStreamBuffers();

    blackboxIInterval = (uint16_t)(32 * 1000 / targetPidLooptime);
    blackboxPInterval = 1 << blackboxConfig()->sample_rate;
    if (blackboxPInterval > blackboxIInterval) {
        blackboxPInterval = 0;
    }

    if (blackboxConfig()->device) {
        blackboxSetState(BLACKBOX_STATE_STOPPED);
    } else {
        blackboxSetState(BLACKBOX_STATE_DISABLED);
    }
    blackboxSInterval = blackboxIInterval * 256;
}
#endif
