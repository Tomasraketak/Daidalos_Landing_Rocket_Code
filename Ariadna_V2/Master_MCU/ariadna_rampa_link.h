// =====================================================================
//  Ariadna V2 — Rocket <-> Rampa (Launch Pad) Umbilical I2C Protocol
//
//  Physical connection: 3-pin magnetic pogo connector (GND, SDA, SCL)
//    Ariadna V2 Master MCU  : I2C1 or I2C2 uSup -> GP8 (SDA), GP9 (SCL)
//                             (shared I2C0 bus with PCA9685 @ 0x40)
//    Rampa PCB1 (Pico 2 U7) : J3 pin 1 = GND
//                             J3 pin 3 -> R22 (100R) -> GP1 (I2C0 SCL)
//                             J3 pin 4 -> R21 (1k)   -> GP0 (I2C0 SDA)
//                             (Rampa internal sensors live on I2C1 GP6/GP7)
//
//  Role: Rocket (Master MCU) = I2C Master, Rampa = I2C Slave @ 0x2B.
//  While umbilical I2C heartbeats arrive (< 2 s), Rampa keeps its LoRa
//  radio hardware-powered OFF (TCA9534 P4 LORA_EN = LOW -> TPS22918 U12).
//
//  Safe hot-disconnect before liftoff (T - ARIADNA_UMB_DETACH_MS):
//    1. T - 1000 ms: Rocket opens mandatory 300 ms final LoRa RX window
//       for last-chance ABORT from Ground Station.
//    2. T - 100 ms: Rocket sends RAMPA_CMD_COMMIT (exact remaining ms) and
//       reads back RAMPA_STATE_COMMITTED.
//    3. Immediately upon COMMIT ACK:
//         - Rampa calls Wire.end() and puts GP0/GP1 to Hi-Z INPUT.
//         - Rocket stops addressing 0x2B (keeps I2C0 alive for PCA9685).
//    4. T - 0 ms: Rampa verifies physical key + local abort button, fires
//       igniter, rocket lifts off and magnetic pogo separates cleanly.
//
//  Plain C, no Arduino dependency: host-tested by ../../test/rampa_link_test.cpp.
// =====================================================================
#ifndef ARIADNA_RAMPA_LINK_H
#define ARIADNA_RAMPA_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 7-bit I2C slave address of Rampa on Master MCU's I2C0 bus (clear of PCA9685 0x40/0x70). */
#define ARIADNA_RAMPA_I2C_ADDR      0x2Bu

#define ARIADNA_RAMPA_CMD_MAGIC     0xB5u
#define ARIADNA_RAMPA_STAT_MAGIC    0xB6u
#define ARIADNA_RAMPA_VERSION       1u

/** Polling rate on ground / during countdown. */
#define ARIADNA_RAMPA_POLL_HZ       20u
/** Heartbeat timeout on Rampa before treating umbilical as disconnected. */
#define ARIADNA_RAMPA_TIMEOUT_MS    1500u
/** Time before T-0 at which Rocket commits launch and both sides detach I2C. */
#define ARIADNA_UMB_DETACH_MS       100u
/** Minimum allowed countdown remaining time to accept a new COUNTDOWN command. */
#define ARIADNA_COUNTDOWN_MIN_MS    5000u
/** Maximum allowed countdown duration (1 hour). */
#define ARIADNA_COUNTDOWN_MAX_MS    3600000u

/** Decode return codes. */
#define ARIADNA_RAMPA_OK            0
#define ARIADNA_RAMPA_E_SHORT      -1
#define ARIADNA_RAMPA_E_MAGIC      -2
#define ARIADNA_RAMPA_E_VERSION    -3
#define ARIADNA_RAMPA_E_CRC        -4

/** Commands sent from Rocket (Master MCU) -> Rampa over I2C write. */
typedef enum {
    RAMPA_CMD_PING      = 0,  // idle heartbeat / sync / clear old ABORT when ready
    RAMPA_CMD_PREPARE   = 1,  // launch soon: run continuity/power checks, buzz alert
    RAMPA_CMD_COUNTDOWN = 2,  // active countdown, t_rem_ms counts down to 0
    RAMPA_CMD_COMMIT    = 3,  // final T-100 ms lock-in; both sides detach I2C after ACK
    RAMPA_CMD_ABORT     = 4,  // immediate disarm & cancel countdown
    RAMPA_CMD_DETACH    = 5   // software disconnect umbilical without firing
} ariadna_rampa_cmd_id_t;

/** Rampa mission/safety states reported back to Rocket. */
typedef enum {
    RAMPA_STATE_OFFLINE   = 0,  // (used by Master MCU when Rampa is not responding)
    RAMPA_STATE_IDLE      = 1,  // connected, safe, PYRO_ARM OFF
    RAMPA_STATE_READY     = 2,  // PREPARE acknowledged, all interlocks healthy
    RAMPA_STATE_COUNTDOWN = 3,  // synchronized countdown running
    RAMPA_STATE_COMMITTED = 4,  // T-100 ms committed, I2C detaching, firing at T-0
    RAMPA_STATE_FIRED     = 5,  // igniter fired / firing pulse active
    RAMPA_STATE_ABORT     = 6,  // aborted by GS, Rocket, local button, or fault
    RAMPA_STATE_FAULT     = 7   // hardware/interlock fault (see fault_mask)
} ariadna_rampa_state_t;

// --- ariadna_rampa_cmd_t.rocket_flags --------------------------------
#define ARIADNA_RK_POST_OK      (1u << 0)
#define ARIADNA_RK_NAV_VALID    (1u << 1)
#define ARIADNA_RK_ARMED        (1u << 2)
#define ARIADNA_RK_RBF_OUT      (1u << 3)
#define ARIADNA_RK_POS_OK       (1u << 4)
#define ARIADNA_RK_NAV_OK       (1u << 5)
#define ARIADNA_RK_NO_ABORT     (1u << 6)
#define ARIADNA_RK_GPS_UTC      (1u << 7)

// --- ariadna_rampa_status_t.flags ------------------------------------
#define ARIADNA_RMP_KEY_ARMED   (1u << 0)  // physical key switch J4 closed (RBF_SENSE HIGH)
#define ARIADNA_RMP_CONT1       (1u << 1)  // pyro channel 1 continuity OK
#define ARIADNA_RMP_CONT2       (1u << 2)  // pyro channel 2 continuity OK
#define ARIADNA_RMP_CONT3       (1u << 3)  // pyro channel 3 continuity OK
#define ARIADNA_RMP_PYRO_ARMED  (1u << 4)  // PYRO_ARM high-side MOSFET Q20 enabled
#define ARIADNA_RMP_LORA_OFF    (1u << 5)  // Rampa Ra-02 radio powered OFF via U12
#define ARIADNA_RMP_DETACHING   (1u << 6)  // Rampa about to call Wire.end() after this read
#define ARIADNA_RMP_READY       (1u << 7)  // all Rampa Go/No-Go interlocks pass

// --- ariadna_rampa_status_t.fault_mask -------------------------------
#define ARIADNA_RMPF_LOW_VBAT   (1u << 0)  // battery below safe threshold
#define ARIADNA_RMPF_NO_KEY     (1u << 1)  // physical arming key J4 open
#define ARIADNA_RMPF_NO_CONT    (1u << 2)  // selected igniter channel has no continuity
#define ARIADNA_RMPF_TIMEOUT    (1u << 3)  // Rocket I2C heartbeat timed out during countdown
#define ARIADNA_RMPF_ROCKET     (1u << 4)  // Rocket reported not ready / fault
#define ARIADNA_RMPF_HW         (1u << 5)  // internal I2C IC (TCA9534 / ADS1115 / INA226) fault
#define ARIADNA_RMPF_BTN_ABORT  (1u << 6)  // local physical abort button SW1 pressed
#define ARIADNA_RMPF_CLOCK_SKEW (1u << 7)  // excessive timing jump / invalid countdown

#define ARIADNA_RAMPA_CMD_SIZE   16u
#define ARIADNA_RAMPA_STAT_SIZE  20u

#pragma pack(push, 1)
/** Rocket (Master MCU) -> Rampa I2C write payload (16 bytes). */
typedef struct {
    uint8_t  magic;         //  0  ARIADNA_RAMPA_CMD_MAGIC (0xB5)
    uint8_t  version;       //  1  ARIADNA_RAMPA_VERSION (1)
    uint8_t  cmd;           //  2  ariadna_rampa_cmd_id_t
    uint8_t  seq;           //  3  monotonic sequence number
    int32_t  t_rem_ms;      //  4  ms remaining until T-0 (-1 if no countdown)
    uint32_t utc_ms;        //  8  UTC ms-of-day (0xFFFFFFFF if unknown)
    uint8_t  rocket_flags;  // 12  ARIADNA_RK_*
    uint8_t  pyro_ch;       // 13  1..3 (selected igniter channel)
    uint16_t crc;           // 14  CRC-16/CCITT-FALSE over bytes 0..13
} ariadna_rampa_cmd_t;

/** Rampa -> Rocket (Master MCU) I2C read payload (20 bytes). */
typedef struct {
    uint8_t  magic;         //  0  ARIADNA_RAMPA_STAT_MAGIC (0xB6)
    uint8_t  version;       //  1  ARIADNA_RAMPA_VERSION (1)
    uint8_t  state;         //  2  ariadna_rampa_state_t
    uint8_t  last_seq;      //  3  seq of last valid command processed
    uint8_t  flags;         //  4  ARIADNA_RMP_*
    uint8_t  fault_mask;    //  5  ARIADNA_RMPF_*
    uint16_t vbat_mv;       //  6  Rampa battery voltage, mV
    int32_t  t_rem_ms;      //  8  Rampa's synchronized remaining ms to T-0 (-1 if none)
    uint16_t cont_mv[3];    // 12  continuity ADC readings (mV) for channels 1..3
    uint16_t crc;           // 18  CRC-16/CCITT-FALSE over bytes 0..17
} ariadna_rampa_status_t;
#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(ariadna_rampa_cmd_t) == ARIADNA_RAMPA_CMD_SIZE, "rampa_cmd layout changed");
static_assert(sizeof(ariadna_rampa_status_t) == ARIADNA_RAMPA_STAT_SIZE, "rampa_status layout changed");
#else
_Static_assert(sizeof(ariadna_rampa_cmd_t) == ARIADNA_RAMPA_CMD_SIZE, "rampa_cmd layout changed");
_Static_assert(sizeof(ariadna_rampa_status_t) == ARIADNA_RAMPA_STAT_SIZE, "rampa_status layout changed");
#endif

/** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), identical to ariadna_crc16(). */
static inline uint16_t ariadna_rampa_crc(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

/** Finalize magic, version and CRC on a Rocket -> Rampa command frame. */
static inline void ariadna_rampa_cmd_seal(ariadna_rampa_cmd_t *c)
{
    c->magic   = ARIADNA_RAMPA_CMD_MAGIC;
    c->version = ARIADNA_RAMPA_VERSION;
    c->crc     = ariadna_rampa_crc((const uint8_t *)c, sizeof(*c) - 2u);
}

/** Validate and decode a 16-byte Rocket -> Rampa command frame. */
static inline int ariadna_rampa_cmd_decode(const uint8_t *buf, size_t n,
                                           ariadna_rampa_cmd_t *out)
{
    if (n < sizeof(*out)) return ARIADNA_RAMPA_E_SHORT;
    ariadna_rampa_cmd_t tmp;
    memcpy(&tmp, buf, sizeof(tmp));
    if (ariadna_rampa_crc(buf, sizeof(tmp) - 2u) != tmp.crc)
        return ARIADNA_RAMPA_E_CRC;
    if (tmp.magic != ARIADNA_RAMPA_CMD_MAGIC) return ARIADNA_RAMPA_E_MAGIC;
    if (tmp.version != ARIADNA_RAMPA_VERSION) return ARIADNA_RAMPA_E_VERSION;
    *out = tmp;
    return ARIADNA_RAMPA_OK;
}

/** Finalize magic, version and CRC on a Rampa -> Rocket status frame. */
static inline void ariadna_rampa_status_seal(ariadna_rampa_status_t *s)
{
    s->magic   = ARIADNA_RAMPA_STAT_MAGIC;
    s->version = ARIADNA_RAMPA_VERSION;
    s->crc     = ariadna_rampa_crc((const uint8_t *)s, sizeof(*s) - 2u);
}

/** Validate and decode a 20-byte Rampa -> Rocket status frame. */
static inline int ariadna_rampa_status_decode(const uint8_t *buf, size_t n,
                                              ariadna_rampa_status_t *out)
{
    if (n < sizeof(*out)) return ARIADNA_RAMPA_E_SHORT;
    ariadna_rampa_status_t tmp;
    memcpy(&tmp, buf, sizeof(tmp));
    if (ariadna_rampa_crc(buf, sizeof(tmp) - 2u) != tmp.crc)
        return ARIADNA_RAMPA_E_CRC;
    if (tmp.magic != ARIADNA_RAMPA_STAT_MAGIC) return ARIADNA_RAMPA_E_MAGIC;
    if (tmp.version != ARIADNA_RAMPA_VERSION)  return ARIADNA_RAMPA_E_VERSION;
    *out = tmp;
    return ARIADNA_RAMPA_OK;
}

#ifdef __cplusplus
}
#endif

#endif // ARIADNA_RAMPA_LINK_H
