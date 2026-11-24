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

#pragma once

#include <stdint.h>
#include <stddef.h>

#define SBL_CHUNK_MAGIC_0           'S'
#define SBL_CHUNK_MAGIC_1           'L'
#define SBL_CHUNK_MAGIC_LE          0x4C53U
#define SBL_FORMAT_VERSION          1U

#define SBL_FLAG_COMPACT            (1U << 0)
#define SBL_FLAG_MOTOR_COUNT_SHIFT  1U
#define SBL_FLAG_MOTOR_COUNT_MASK   (0x0FU << SBL_FLAG_MOTOR_COUNT_SHIFT)

#define SBL_RESET_AXIS_MASK         0x03U
#define SBL_RESET_SATURATED_BIT     (1U << 6)
#define SBL_RESET_STABILISED_BIT    (1U << 7)

#define SBL_HDR_FLAG_MANUAL_OVERRIDE (1U << 0)
#define SBL_HDR_FLAG_DSHOT_FEEDBACK  (1U << 1)
#define SBL_HDR_FLAG_HAS_INDI_META   (1U << 7)

#define SBL_MAX_MOTORS              8U

typedef enum {
    SBL_CHUNK_SESSION_HEADER = 1,
    SBL_CHUNK_LADRC_BATCH    = 2,
    SBL_CHUNK_EVENT          = 3,
    SBL_CHUNK_SESSION_FOOTER = 4,
} sblChunkType_e;

typedef struct __attribute__((packed)) sblChunkHdr_s {
    uint16_t magic;      // SBL_CHUNK_MAGIC_LE ('S', 'L')
    uint8_t  type;       // sblChunkType_e
    uint8_t  flags;      // bit0=compact, bits1..4=motorCount
    uint16_t payloadLen; // payload bytes before trailing CRC16
    uint16_t seq;        // monotonic chunk sequence number
} sblChunkHdr_t;

typedef struct __attribute__((packed)) sblLadrcAxisCfg_s {
    uint8_t  order;    // 1 or 2
    uint8_t  reserved;
    uint16_t wcDeci;   // 0.1 rad/s
    uint16_t woDeci;   // 0.1 rad/s
    uint16_t b0Deci;   // 0.1 norm units
    uint16_t tdHz;     // Hz (0 = disabled)
} sblLadrcAxisCfg_t;

typedef struct __attribute__((packed)) sblSessionHeader_s {
    uint8_t  version;
    uint8_t  compact;
    uint8_t  sampleDenom;
    uint8_t  motorCount;
    uint32_t pidLooptimeUs;
    uint16_t gyroSampleHz;
    uint16_t vbatRefMv;
    uint16_t vbatMinCellMv;
    uint16_t vbatWarnCellMv;
    uint16_t vbatMaxCellMv;
    int16_t  motorOutputLow;
    int16_t  motorOutputHigh;
    uint8_t  mixerType;
    uint8_t  fixedWing;
    uint8_t  antiWindup;
    uint8_t  vbatB0Comp;
    uint8_t  airspeedSched;
    uint8_t  reserved0;
    uint16_t airspeedRefCms;
    uint16_t distLimitPermille;
    uint16_t gyroLpf1StaticHz;
    uint16_t gyroLpf1DynMinHz;
    uint16_t gyroLpf1DynMaxHz;
    uint16_t gyroLpf2StaticHz;
    uint8_t  dynNotchCount;
    uint8_t  reserved1;
    uint16_t dynNotchMinHz;
    uint16_t dynNotchMaxHz;
    sblLadrcAxisCfg_t axis[3];
    char     craftName[16];
    char     boardInfo[16];
    char     fwRevision[24];
    char     startDatetime[20];
    uint16_t propSizeDeciIn;
    uint16_t auwCentiKg;
    uint16_t agilityPct;
    uint16_t authorityTrimPct;
} sblSessionHeader_t;

typedef struct __attribute__((packed)) sblFrameFullHead_s {
    uint32_t timeUs;
    int16_t  gyroRawDds[3];     // pre-filter gyro in 0.1 deg/s
    int16_t  gyroFiltDds[3];    // post-filter gyro in 0.1 deg/s
    int16_t  setpointDds[3];    // setpoint in 0.1 deg/s
    int16_t  tdRateDds[3];      // TD filtered rate r in 0.1 deg/s
    int16_t  tdRateDotCkrps[3]; // TD feedforward rDot in 0.01 kdeg/s^2
    int16_t  z1Dds[3];          // LESO state z1 in 0.1 deg/s
    int16_t  z2Ckrps[3];        // LESO state z2 in 0.01 kdeg/s^2
    int16_t  z3DistPermille[3]; // distTermSum in permille of sumLimit
    int16_t  pdTermPermille[3]; // pTermSum + dTermSum in permille
    int16_t  uCmdPermille[3];   // uCmdNorm * 1000
    uint16_t b0ScalePermille;   // b0Eff / b0Nom * 1000
    uint16_t mixAuthPermille;   // mixer authority * 1000
    uint16_t vbatMv;            // battery voltage in mV
    int16_t  rcThrottle;        // throttle 0..1000
    uint8_t  resetFlags;        // bits 0..1/2..3/4..5 axis resetMode, bit6 sat, bit7 stab
    uint8_t  reserved;
} sblFrameFullHead_t;

typedef struct __attribute__((packed)) sblFrameCompactHead_s {
    uint32_t timeUs;
    int16_t  gyroFiltDds[3];    // post-filter gyro in 0.1 deg/s
    int16_t  setpointDds[3];    // setpoint in 0.1 deg/s
    int16_t  tdRateDds[3];      // TD filtered rate r in 0.1 deg/s
    int16_t  z1Dds[3];          // LESO state z1 in 0.1 deg/s
    int16_t  z3DistPermille[3]; // distTermSum in permille of sumLimit
    int16_t  uCmdPermille[3];   // uCmdNorm * 1000
    uint16_t b0ScalePermille;   // b0Eff / b0Nom * 1000
    uint16_t mixAuthPermille;   // mixer authority * 1000
    uint16_t vbatMv;            // battery voltage in mV
    int16_t  rcThrottle;        // throttle 0..1000
    uint8_t  resetFlags;        // bits 0..1/2..3/4..5 axis resetMode, bit6 sat, bit7 stab
    uint8_t  reserved;
} sblFrameCompactHead_t;

typedef struct __attribute__((packed)) sblEventPayload_s {
    uint32_t timeUs;
    uint8_t  eventType;
    uint8_t  paramByte;
    uint16_t reserved;
    uint32_t value0;
    uint32_t value1;
} sblEventPayload_t;

typedef struct __attribute__((packed)) sblSessionFooter_s {
    uint32_t totalFrames;
    uint32_t droppedFrames;
    uint32_t endTimeUs;
    uint32_t sdWriteErrors;
} sblSessionFooter_t;

#define SBL_FULL_FRAME_SIZE(motors)    (sizeof(sblFrameFullHead_t) + (size_t)(motors) * sizeof(int16_t))
#define SBL_COMPACT_FRAME_SIZE(motors) (sizeof(sblFrameCompactHead_t) + (size_t)(motors) * sizeof(int16_t))
#define SBL_MAX_FRAME_SIZE             SBL_FULL_FRAME_SIZE(SBL_MAX_MOTORS)

static inline uint16_t sblCrc16Ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
