// =====================================================================
//  Static Fire Stand <-> Ariadna V2: I2C link protocol
//
//  Header only, NO Arduino dependency, so the very same file is
//    * compiled into the stand firmware (this folder),
//    * compiled into the Ariadna example Examples/Static_Fire_Link,
//    * and checked on a workstation (tests/link_proto_test.cpp).
//
//  Canonical copy: Static_Fire_Stand/firmware/StaticFire_Stand/sf_link_proto.h
//  The Arduino builder only sees the sketch folder, so Ariadna carries a
//  byte-for-byte copy. Bump SFL_PROTO_VERSION when the wire format changes;
//  both sides refuse to talk across versions.
//
//  Roles
//    Ariadna V2 Master  : I2C MASTER (connector I2C1/I2C2, GP8/GP9, 10k PU)
//    Stand (Pico 2 W)   : I2C SLAVE at SFL_I2C_ADDR (I2C0, GP4/GP5)
//
//  The stand cannot push anything on an I2C bus, so everything it wants to
//  say is a level in a register that Ariadna polls, and everything Ariadna
//  wants to say is a level in a heartbeat it writes 20x a second. Levels,
//  not events: a lost frame is repeated by the next one, nothing has to be
//  retransmitted and nothing can be missed.
//
//  Registers (the master writes one register byte, then reads / writes):
//
//    0x00 STATUS     read 16 B   live state, thrust, time to / since T0
//    0x01 INFO       read 16 B   firmware, burn id, igniter on-time, timing
//    0x10 HEARTBEAT  write 8 B   Ariadna's readiness and its two ack levels
//    0x20 PAGE       write 5 B   select a 64 B chunk of the last burn, then
//                    read 66 B   [code][64 data][crc8]
//
//  All integers little endian. Every frame carries a CRC-8 (poly 0x07).
// =====================================================================
#ifndef SF_LINK_PROTO_H
#define SF_LINK_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define SFL_PROTO_VERSION    1u
#define SFL_I2C_ADDR         0x2Cu     // 0x2A UWB tag, 0x2B Rampa, 0x40 PCA9685

#define SFL_REG_STATUS       0x00u
#define SFL_REG_INFO         0x01u
#define SFL_REG_HEARTBEAT    0x10u
#define SFL_REG_PAGE         0x20u

#define SFL_MAGIC_STAND      0x5Fu     // first byte of every stand -> Ariadna frame
#define SFL_MAGIC_ARIADNA    0xA7u     // first byte of the heartbeat

#define SFL_STATUS_SIZE      16u
#define SFL_INFO_SIZE        16u
#define SFL_HB_SIZE          8u
#define SFL_PAGE_REQ_SIZE    5u
#define SFL_PAGE_BYTES       256u      // one flash page of the stand's log
#define SFL_CHUNK            64u       // bytes per I2C read of a page
#define SFL_CHUNKS_PER_PAGE  (SFL_PAGE_BYTES / SFL_CHUNK)
#define SFL_CHUNK_REPLY      (SFL_CHUNK + 2u)

// Timing, ms. The stand calls Ariadna lost after SFL_HB_TIMEOUT_MS without a
// good heartbeat; Ariadna calls the stand lost after SFL_STATUS_TIMEOUT_MS
// without a good STATUS. Either one in the last seconds of a countdown
// stops it, and during the burn Ariadna drives the flaps fully open.
#define SFL_HB_PERIOD_MS       50u
#define SFL_HB_TIMEOUT_MS     300u
#define SFL_STATUS_TIMEOUT_MS 200u

// "no T0 yet" marker for STATUS.t_rel_us
#define SFL_T_NONE           ((int32_t)0x80000000)

// Stand states (the stand's StandState enum, repeated here so Ariadna does
// not need the firmware to know what they mean).
enum {
    SFL_ST_BOOT      = 0,
    SFL_ST_IDLE      = 1,
    SFL_ST_COUNTDOWN = 2,
    SFL_ST_RECORDING = 3,
    SFL_ST_FAULT     = 4,
    SFL_ST_DUMPING   = 5
};

// ---- STATUS.flags (stand -> Ariadna) ---------------------------------
#define SFL_F_READY        0x01u  // idle, load cell + storage fine, RBF key out
#define SFL_F_RBF_OUT      0x02u  // the stand's RBF key is removed (armed)
#define SFL_F_IGNITE       0x04u  // T0 reached: Ariadna must fire NOW (latched)
#define SFL_F_DATA_READY   0x08u  // a finished burn waits to be copied
#define SFL_F_LINK_OK      0x10u  // the stand hears Ariadna's heartbeat
#define SFL_F_ARIADNA_RDY  0x20u  // ... and Ariadna says it is ready
#define SFL_F_HX_OK        0x40u  // load cell answering
#define SFL_F_LOG_OK       0x80u  // flash log usable

// ---- HEARTBEAT.flags (Ariadna -> stand) -------------------------------
#define SFL_A_READY        0x01u  // everything Ariadna needs for a run is fine
#define SFL_A_ARMED        0x02u  // pyro armed (Navigation MCU, pyro 1)
#define SFL_A_FIRED        0x04u  // pyro 1 has been commanded for this run
#define SFL_A_CONT         0x08u  // igniter continuity on pyro 1
#define SFL_A_BUSY         0x10u  // copying data / writing flash
#define SFL_A_FAULT        0x20u  // something Ariadna wants a human to look at

// ---- PAGE reply codes ---------------------------------------------------
enum {
    SFL_PG_OK     = 0,
    SFL_PG_BUSY   = 1,   // stand is not idle: flash is in use
    SFL_PG_NODATA = 2,   // no finished burn
    SFL_PG_RANGE  = 3,   // page or chunk out of range
    SFL_PG_BADREQ = 4    // request failed its CRC
};

// =====================================================================
//  helpers
// =====================================================================
static inline uint8_t sfl_crc8(const uint8_t *p, size_t n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (uint8_t k = 0; k < 8; k++)
            c = (c & 0x80u) ? (uint8_t)((c << 1) ^ 0x07u) : (uint8_t)(c << 1);
    }
    return c;
}

static inline void sfl_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void sfl_put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint16_t sfl_get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t sfl_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// =====================================================================
//  STATUS   0 magic | 1 state | 2 flags | 3 run_seq | 4..7 thrust_mN
//           8..11 t_rel_us | 12..13 pages | 14 abort | 15 crc8
//
//  t_rel_us is the time since T0 in microseconds, NEGATIVE during the
//  countdown (minus the time left), SFL_T_NONE while idle. It is taken by
//  the stand at the moment the master starts the read, which is what lets
//  Ariadna put its own samples on the stand's timeline to well under a
//  millisecond without any shared clock.
//
//  run_seq changes at every countdown start and never equals the ack
//  Ariadna last sent, so a stale ack cannot swallow a new run.
// =====================================================================
typedef struct {
    uint8_t  state;
    uint8_t  flags;
    uint8_t  run_seq;
    uint8_t  abort_reason;
    int32_t  thrust_mN;
    int32_t  t_rel_us;
    uint16_t pages;           // pages of the finished burn (header..footer)
} sfl_status_t;

static inline void sfl_status_encode(const sfl_status_t *s, uint8_t out[SFL_STATUS_SIZE])
{
    out[0] = SFL_MAGIC_STAND;
    out[1] = s->state;
    out[2] = s->flags;
    out[3] = s->run_seq;
    sfl_put32(out + 4,  (uint32_t)s->thrust_mN);
    sfl_put32(out + 8,  (uint32_t)s->t_rel_us);
    sfl_put16(out + 12, s->pages);
    out[14] = s->abort_reason;
    out[15] = sfl_crc8(out, SFL_STATUS_SIZE - 1);
}

/** 1 if the frame is a valid STATUS, 0 otherwise (s untouched). */
static inline int sfl_status_decode(const uint8_t in[SFL_STATUS_SIZE], sfl_status_t *s)
{
    if (in[0] != SFL_MAGIC_STAND) return 0;
    if (sfl_crc8(in, SFL_STATUS_SIZE - 1) != in[15]) return 0;
    s->state        = in[1];
    s->flags        = in[2];
    s->run_seq      = in[3];
    s->thrust_mN    = (int32_t)sfl_get32(in + 4);
    s->t_rel_us     = (int32_t)sfl_get32(in + 8);
    s->pages        = sfl_get16(in + 12);
    s->abort_reason = in[14];
    return 1;
}

// =====================================================================
//  INFO     0 magic | 1 proto | 2 fw_major | 3 fw_minor | 4..7 burn_id
//           8..9 ignition_ms | 10..11 preroll_ms | 12..13 recording_ms
//           14 reserved | 15 crc8
//
//  burn_id is the id of the burn the PAGE register serves (the last one
//  closed), 0 if there is none. ignition_ms is what the stand is set to,
//  and what Ariadna uses as the pyro pulse (the Navigation MCU's own
//  limits still apply).
// =====================================================================
typedef struct {
    uint8_t  proto;
    uint8_t  fw_major, fw_minor;
    uint32_t burn_id;
    uint16_t ignition_ms;
    uint16_t preroll_ms;
    uint16_t recording_ms;
} sfl_info_t;

static inline void sfl_info_encode(const sfl_info_t *s, uint8_t out[SFL_INFO_SIZE])
{
    memset(out, 0, SFL_INFO_SIZE);
    out[0] = SFL_MAGIC_STAND;
    out[1] = s->proto;
    out[2] = s->fw_major;
    out[3] = s->fw_minor;
    sfl_put32(out + 4, s->burn_id);
    sfl_put16(out + 8,  s->ignition_ms);
    sfl_put16(out + 10, s->preroll_ms);
    sfl_put16(out + 12, s->recording_ms);
    out[15] = sfl_crc8(out, SFL_INFO_SIZE - 1);
}

static inline int sfl_info_decode(const uint8_t in[SFL_INFO_SIZE], sfl_info_t *s)
{
    if (in[0] != SFL_MAGIC_STAND) return 0;
    if (sfl_crc8(in, SFL_INFO_SIZE - 1) != in[15]) return 0;
    s->proto        = in[1];
    s->fw_major     = in[2];
    s->fw_minor     = in[3];
    s->burn_id      = sfl_get32(in + 4);
    s->ignition_ms  = sfl_get16(in + 8);
    s->preroll_ms   = sfl_get16(in + 10);
    s->recording_ms = sfl_get16(in + 12);
    return 1;
}

// =====================================================================
//  HEARTBEAT  0 magic | 1 flags | 2 ack_seq | 3 data_seq | 4..6 reserved
//             7 crc8
//
//  ack_seq   run_seq whose IGNITE Ariadna has acted on (fired for).
//  data_seq  run_seq whose data Ariadna has finished copying.
// =====================================================================
typedef struct {
    uint8_t flags;
    uint8_t ack_seq;
    uint8_t data_seq;
} sfl_hb_t;

static inline void sfl_hb_encode(const sfl_hb_t *h, uint8_t out[SFL_HB_SIZE])
{
    memset(out, 0, SFL_HB_SIZE);
    out[0] = SFL_MAGIC_ARIADNA;
    out[1] = h->flags;
    out[2] = h->ack_seq;
    out[3] = h->data_seq;
    out[7] = sfl_crc8(out, SFL_HB_SIZE - 1);
}

static inline int sfl_hb_decode(const uint8_t in[SFL_HB_SIZE], sfl_hb_t *h)
{
    if (in[0] != SFL_MAGIC_ARIADNA) return 0;
    if (sfl_crc8(in, SFL_HB_SIZE - 1) != in[7]) return 0;
    h->flags    = in[1];
    h->ack_seq  = in[2];
    h->data_seq = in[3];
    return 1;
}

// =====================================================================
//  PAGE     request  0 reg(0x20) | 1 page_lo | 2 page_hi | 3 chunk | 4 crc8
//                    (crc8 over bytes 1..3)
//           reply    0 code | 1..64 data | 65 crc8 (over bytes 0..64)
//
//  Page 0 is the burn header (SFH1), the last page the footer (SFF1), in
//  between the data pages (SFP1): the stand's own on-flash format, byte
//  for byte (docs/DATA_FORMAT.md), so the copy on Ariadna decodes with the
//  same Python tools as a dump taken from the stand.
// =====================================================================
static inline void sfl_page_req_encode(uint16_t page, uint8_t chunk, uint8_t out[SFL_PAGE_REQ_SIZE])
{
    out[0] = (uint8_t)SFL_REG_PAGE;
    out[1] = (uint8_t)page;
    out[2] = (uint8_t)(page >> 8);
    out[3] = chunk;
    out[4] = sfl_crc8(out + 1, 3);
}

/** The bytes AFTER the register byte, as the slave receives them. */
static inline int sfl_page_req_decode(const uint8_t *args /*4*/, uint16_t *page, uint8_t *chunk)
{
    if (sfl_crc8(args, 3) != args[3]) return 0;
    *page  = sfl_get16(args);
    *chunk = args[2];
    return 1;
}

static inline void sfl_chunk_reply_encode(uint8_t code, const uint8_t *data /*64 or NULL*/,
                                          uint8_t out[SFL_CHUNK_REPLY])
{
    out[0] = code;
    if (data) memcpy(out + 1, data, SFL_CHUNK); else memset(out + 1, 0xFF, SFL_CHUNK);
    out[SFL_CHUNK_REPLY - 1] = sfl_crc8(out, SFL_CHUNK_REPLY - 1);
}

static inline int sfl_chunk_reply_decode(const uint8_t in[SFL_CHUNK_REPLY], uint8_t *code,
                                         uint8_t data[SFL_CHUNK])
{
    if (sfl_crc8(in, SFL_CHUNK_REPLY - 1) != in[SFL_CHUNK_REPLY - 1]) return 0;
    *code = in[0];
    memcpy(data, in + 1, SFL_CHUNK);
    return 1;
}

// =====================================================================
//  Stand-side bookkeeping shared by firmware and test
// =====================================================================

/** What the stand last heard from Ariadna. Written from the I2C interrupt
 *  (core 1), read by the main loop (core 0): hb_ms is stored LAST and read
 *  FIRST, and every field is one aligned word, so no lock is needed. */
typedef struct {
    volatile uint32_t packed;     // flags | ack_seq << 8 | data_seq << 16
    volatile uint32_t hb_ms;      // millis() of the last good heartbeat, 0 = never
} sfl_rx_t;

static inline void sfl_rx_store(sfl_rx_t *r, const sfl_hb_t *h, uint32_t now_ms)
{
    r->packed = (uint32_t)h->flags | ((uint32_t)h->ack_seq << 8) | ((uint32_t)h->data_seq << 16);
    r->hb_ms  = now_ms ? now_ms : 1u;       // 0 is "never"
}

static inline int sfl_rx_alive(const sfl_rx_t *r, uint32_t now_ms)
{
    const uint32_t t = r->hb_ms;
    return t != 0 && (int32_t)(now_ms - t) < (int32_t)SFL_HB_TIMEOUT_MS;
}

static inline sfl_hb_t sfl_rx_get(const sfl_rx_t *r)
{
    const uint32_t p = r->packed;
    sfl_hb_t h;
    h.flags = (uint8_t)p; h.ack_seq = (uint8_t)(p >> 8); h.data_seq = (uint8_t)(p >> 16);
    return h;
}

/** Fresh and says READY. */
static inline int sfl_rx_ready(const sfl_rx_t *r, uint32_t now_ms)
{
    return sfl_rx_alive(r, now_ms) && (sfl_rx_get(r).flags & SFL_A_READY);
}

/** Next run_seq: never 0, and never a value Ariadna currently holds as an
 *  ack, so neither a stale ignition ack nor a stale "data copied" can be
 *  mistaken for an answer to the new run (a stand that reboots restarts
 *  its counter, an Ariadna that stays up keeps its levels). */
static inline uint8_t sfl_next_run_seq(uint8_t prev, uint8_t ack, uint8_t data)
{
    uint8_t n = (uint8_t)(prev + 1u);
    while (n == 0 || n == ack || n == data) n++;
    return n;
}

#endif // SF_LINK_PROTO_H
