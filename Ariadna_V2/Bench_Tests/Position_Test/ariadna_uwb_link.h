// =====================================================================
//  Ariadna V2 — UWB tag I2C link
//
//  The DWM1000 tag no longer sits on the Position MCU's SPI bus. The
//  TriSense module drives SCK and MOSI itself whenever its IMU is not
//  selected, so no second device can share those wires. The tag now lives
//  on its own board (the anchor PCB, flashed with ../Uwb_Tag), which runs
//  the two-way ranging and answers the Position MCU as an I2C SLAVE on the
//  sensor bus (PSDA/PSCL, D4/D5, through the I2CP uSup connector).
//
//  This header is the whole contract between the two. It is plain C with
//  no Arduino dependency, so ../../test/uwb_link_test.cpp checks the
//  encoder and decoder on the host, and the same bytes are what fly.
//  Each sketch carries its own copy of it; `cd ../../test && make sync`
//  checks the copies, `make sync-fix` refreshes them.
//
//  ---------------------------------------------------------------------
//  THE TRANSACTION
//  ---------------------------------------------------------------------
//  One plain I2C read of exactly ARIADNA_UWB_LINK_FRAME_SIZE bytes from
//  ARIADNA_UWB_I2C_ADDR. No register pointer, no write phase: the tag
//  always answers with its current frame. EXACTLY that many bytes, every
//  time — the slave queues the whole frame into its TX FIFO, and a master
//  that stops early leaves bytes behind that the RP2350's I2C block only
//  flushes at the next read.
//
//  The frame carries the most recent range to every anchor the tag has
//  heard, not a queue of every range. The Position MCU reads well above
//  the per-anchor ranging rate and tells a new range from a repeat by the
//  per-slot `count`, so nothing is lost in practice; and if a read is
//  missed, the newest range is the one worth having anyway.
//
//  ---------------------------------------------------------------------
//  TIME
//  ---------------------------------------------------------------------
//  The two boards do not share a clock and nothing here tries to make
//  them. Each range carries its AGE in milliseconds, computed by the tag
//  at the moment it answers the read. The reader subtracts that from its
//  own clock at the moment the read completes. The error is the length of
//  the transfer itself, under two milliseconds — small next to the
//  ranging round-robin that ariadna_uwb_derotate() already corrects.
//
//  ---------------------------------------------------------------------
//  LAYOUT (little-endian, packed, ARIADNA_UWB_LINK_FRAME_SIZE bytes)
//  ---------------------------------------------------------------------
//    0  u8   magic        ARIADNA_UWB_LINK_MAGIC
//    1  u8   version      ARIADNA_UWB_LINK_VERSION
//    2  u8   flags        ARIADNA_UWB_LF_*
//    3  u8   n_valid      slots holding a range
//    4  u16  seq          frames served since boot (wraps)
//    6  u16  total        ranges accepted since boot (wraps)
//    8  slot[ARIADNA_UWB_LINK_SLOTS], each 8 bytes:
//         u16 addr        anchor short address (0 = empty slot)
//         u16 range_cm    centimetres; 0xFFFF = no range
//         u16 age_ms      age at the moment of the read, saturating
//         i8  rxp_dbm     received power, dBm, saturating
//         u8  count       ranges from this anchor (wraps)
//   72  u16  crc          ariadna-wide CRC-16 (0x1021, init 0xFFFF)
//                         over bytes 0..71
//
//  Centimetres are plenty: the DW1000 resolves a few millimetres and the
//  estimator weights a range at 25 cm or worse.
// =====================================================================
#ifndef ARIADNA_UWB_LINK_H
#define ARIADNA_UWB_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 7-bit address of the tag on the sensor bus. Clear of everything else
 * that lives there: AK09918C 0x0C, TF Luna 0x10, BMP580 0x46/0x47, and
 * the TriSense IMU at 0x68/0x69 (it joins the I2C bus whenever its CS is
 * HIGH).
 */
#define ARIADNA_UWB_I2C_ADDR        0x2A

#define ARIADNA_UWB_LINK_MAGIC      0xA7
#define ARIADNA_UWB_LINK_VERSION    1
#define ARIADNA_UWB_LINK_SLOTS      8
#define ARIADNA_UWB_LINK_HDR_SIZE   8
#define ARIADNA_UWB_LINK_SLOT_SIZE  8
#define ARIADNA_UWB_LINK_FRAME_SIZE (ARIADNA_UWB_LINK_HDR_SIZE + \
                                     ARIADNA_UWB_LINK_SLOTS * ARIADNA_UWB_LINK_SLOT_SIZE + 2)

#define ARIADNA_UWB_RANGE_NONE      0xFFFFu

/** The DW1000 answered with the right device ID and ranging is running. */
#define ARIADNA_UWB_LF_RADIO_UP     (1u << 0)
/** At least one anchor ranged in the last second. */
#define ARIADNA_UWB_LF_RANGING      (1u << 1)

/** Decoder results. */
#define ARIADNA_UWB_LINK_OK          0
#define ARIADNA_UWB_LINK_E_SHORT    -1
#define ARIADNA_UWB_LINK_E_MAGIC    -2
#define ARIADNA_UWB_LINK_E_VERSION  -3
#define ARIADNA_UWB_LINK_E_CRC      -4

/** One slot, decoded. `valid` is 0 for an empty slot. */
typedef struct {
    uint16_t addr;
    uint16_t age_ms;
    float    range_m;
    int8_t   rxp_dbm;
    uint8_t  count;
    uint8_t  valid;
    uint8_t  _pad;
} ariadna_uwb_link_slot_t;

/** One frame, decoded. */
typedef struct {
    uint8_t  flags;
    uint8_t  n_valid;
    uint16_t seq;
    uint16_t total;
    uint16_t _pad;
    ariadna_uwb_link_slot_t slot[ARIADNA_UWB_LINK_SLOTS];
} ariadna_uwb_link_frame_t;

/** Same polynomial and seed as ariadna_crc16() in ariadna_link.h. */
static inline uint16_t ariadna_uwb_link_crc(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline void ariadna_uwb_link_put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t ariadna_uwb_link_get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/** Metres to the wire's centimetres. Anything that is not a usable range
 *  — NaN, negative, beyond 655 m — becomes "no range" rather than a
 *  clamped number that would look real. */
static inline uint16_t ariadna_uwb_link_cm(float range_m)
{
    if (!(range_m >= 0.0f)) return (uint16_t)ARIADNA_UWB_RANGE_NONE;
    const float cm = range_m * 100.0f + 0.5f;
    if (!(cm < 65535.0f)) return (uint16_t)ARIADNA_UWB_RANGE_NONE;
    return (uint16_t)cm;
}

/**
 * Encode a frame into `out`, which must hold ARIADNA_UWB_LINK_FRAME_SIZE
 * bytes. A slot is written as empty unless its `valid` is set, its
 * address is non-zero and its range is usable; n_valid is recomputed
 * from what was actually written, so it can never disagree with it.
 */
static inline void ariadna_uwb_link_encode(const ariadna_uwb_link_frame_t *f,
                                           uint8_t *out)
{
    uint8_t n_valid = 0;
    uint8_t *p = out + ARIADNA_UWB_LINK_HDR_SIZE;
    for (uint8_t i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++, p += ARIADNA_UWB_LINK_SLOT_SIZE) {
        const ariadna_uwb_link_slot_t *s = &f->slot[i];
        uint16_t cm = s->valid ? ariadna_uwb_link_cm(s->range_m)
                               : (uint16_t)ARIADNA_UWB_RANGE_NONE;
        const uint16_t addr = (cm == ARIADNA_UWB_RANGE_NONE) ? 0 : s->addr;
        if (addr == 0) cm = (uint16_t)ARIADNA_UWB_RANGE_NONE;
        ariadna_uwb_link_put16(p + 0, addr);
        ariadna_uwb_link_put16(p + 2, cm);
        ariadna_uwb_link_put16(p + 4, addr ? s->age_ms : 0);
        p[6] = addr ? (uint8_t)s->rxp_dbm : 0;
        p[7] = addr ? s->count : 0;
        if (addr) n_valid++;
    }
    out[0] = ARIADNA_UWB_LINK_MAGIC;
    out[1] = ARIADNA_UWB_LINK_VERSION;
    out[2] = f->flags;
    out[3] = n_valid;
    ariadna_uwb_link_put16(out + 4, f->seq);
    ariadna_uwb_link_put16(out + 6, f->total);
    const size_t body = ARIADNA_UWB_LINK_FRAME_SIZE - 2;
    ariadna_uwb_link_put16(out + body, ariadna_uwb_link_crc(out, body));
}

/**
 * Decode `n` received bytes. Returns ARIADNA_UWB_LINK_OK and fills `f`,
 * or a negative ARIADNA_UWB_LINK_E_* and leaves `f` untouched — a frame
 * that fails any check is never half-applied.
 *
 * The CRC is checked before anything else is believed, so a bus that
 * reads all 0xFF (nobody answering, or a stuck line) or a frame torn by a
 * reset is rejected as a whole.
 */
static inline int ariadna_uwb_link_decode(const uint8_t *buf, size_t n,
                                          ariadna_uwb_link_frame_t *f)
{
    if (n < ARIADNA_UWB_LINK_FRAME_SIZE) return ARIADNA_UWB_LINK_E_SHORT;
    const size_t body = ARIADNA_UWB_LINK_FRAME_SIZE - 2;
    if (ariadna_uwb_link_crc(buf, body) != ariadna_uwb_link_get16(buf + body))
        return ARIADNA_UWB_LINK_E_CRC;
    if (buf[0] != ARIADNA_UWB_LINK_MAGIC)   return ARIADNA_UWB_LINK_E_MAGIC;
    if (buf[1] != ARIADNA_UWB_LINK_VERSION) return ARIADNA_UWB_LINK_E_VERSION;

    f->flags = buf[2];
    f->seq   = ariadna_uwb_link_get16(buf + 4);
    f->total = ariadna_uwb_link_get16(buf + 6);
    f->_pad  = 0;
    uint8_t n_valid = 0;
    const uint8_t *p = buf + ARIADNA_UWB_LINK_HDR_SIZE;
    for (uint8_t i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++, p += ARIADNA_UWB_LINK_SLOT_SIZE) {
        ariadna_uwb_link_slot_t *s = &f->slot[i];
        const uint16_t addr = ariadna_uwb_link_get16(p + 0);
        const uint16_t cm   = ariadna_uwb_link_get16(p + 2);
        s->addr    = addr;
        s->age_ms  = ariadna_uwb_link_get16(p + 4);
        s->rxp_dbm = (int8_t)p[6];
        s->count   = p[7];
        s->_pad    = 0;
        s->valid   = (addr != 0 && cm != ARIADNA_UWB_RANGE_NONE) ? 1 : 0;
        s->range_m = s->valid ? (float)cm * 0.01f : 0.0f;
        if (s->valid) n_valid++;
    }
    f->n_valid = n_valid;
    return ARIADNA_UWB_LINK_OK;
}

#ifdef __cplusplus
}
#endif

#endif // ARIADNA_UWB_LINK_H
