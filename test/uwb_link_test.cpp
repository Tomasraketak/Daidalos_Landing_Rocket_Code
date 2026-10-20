// =====================================================================
//  Ariadna V2 — UWB tag I2C link checks
//
//  ../Ariadna_V2/common/ariadna_uwb_link.h compiles UNCHANGED into this
//  test, into the tag sketch and into Position_MCU.ino.
//
//  What this establishes: a frame survives the round trip; every way a
//  read can go wrong on a real bus — nobody answering (all 0xFF), a stuck
//  line (all 0x00), a short read, one flipped bit, an old firmware on the
//  other end — is rejected as a whole; and a slot that does not hold a
//  usable range can never decode as one.
//
//  Build:  make uwblink
// =====================================================================
#include "../Ariadna_V2/common/ariadna_uwb_link.h"

#include <cstdio>
#include <cstring>
#include <cmath>

static int g_checks = 0;
static int g_fails  = 0;

static void check(bool cond, const char *what)
{
    g_checks++;
    if (!cond) {
        g_fails++;
        printf("  FAIL  %s\n", what);
    }
}

static void sample(ariadna_uwb_link_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    f->flags = ARIADNA_UWB_LF_RADIO_UP | ARIADNA_UWB_LF_RANGING;
    f->seq   = 0xBEEF;
    f->total = 1234;
    const float r[4] = { 12.34f, 0.07f, 299.99f, 41.5f };
    for (int i = 0; i < 4; i++) {
        f->slot[i].addr    = (uint16_t)(i + 1);
        f->slot[i].range_m = r[i];
        f->slot[i].age_ms  = (uint16_t)(10 * i + 3);
        f->slot[i].rxp_dbm = (int8_t)(-70 - i * 5);
        f->slot[i].count   = (uint8_t)(250 + i);   // wraps past 255
        f->slot[i].valid   = 1;
    }
}

int main()
{
    printf("Ariadna V2 UWB tag I2C link\n===========================\n");

    check(ARIADNA_UWB_LINK_FRAME_SIZE == 74, "frame is 74 bytes");
    check(ARIADNA_UWB_LINK_FRAME_SIZE <= 256, "frame fits the arduino-pico Wire buffer");

    // --- round trip ----------------------------------------------------
    printf("round trip\n");
    ariadna_uwb_link_frame_t in, out;
    sample(&in);
    uint8_t buf[ARIADNA_UWB_LINK_FRAME_SIZE];
    ariadna_uwb_link_encode(&in, buf);
    check(ariadna_uwb_link_decode(buf, sizeof(buf), &out) == ARIADNA_UWB_LINK_OK, "decodes");
    check(out.flags == in.flags && out.seq == in.seq && out.total == in.total, "header survives");
    check(out.n_valid == 4, "n_valid counts the four ranges");
    for (int i = 0; i < 4; i++) {
        const ariadna_uwb_link_slot_t &a = in.slot[i], &b = out.slot[i];
        check(b.valid && b.addr == a.addr, "slot address");
        check(fabsf(b.range_m - a.range_m) <= 0.005f, "range within half a centimetre");
        check(b.age_ms == a.age_ms && b.rxp_dbm == a.rxp_dbm && b.count == a.count,
              "age, power and count");
    }
    for (int i = 4; i < ARIADNA_UWB_LINK_SLOTS; i++)
        check(!out.slot[i].valid && out.slot[i].addr == 0, "unused slots decode empty");

    // --- what a real bus does wrong ------------------------------------
    printf("bus faults\n");
    ariadna_uwb_link_frame_t keep;
    memset(&keep, 0x5A, sizeof(keep));
    out = keep;

    uint8_t ff[ARIADNA_UWB_LINK_FRAME_SIZE];
    memset(ff, 0xFF, sizeof(ff));
    check(ariadna_uwb_link_decode(ff, sizeof(ff), &out) != ARIADNA_UWB_LINK_OK,
          "all 0xFF (no tag on the bus) is rejected");
    uint8_t zz[ARIADNA_UWB_LINK_FRAME_SIZE] = {};
    check(ariadna_uwb_link_decode(zz, sizeof(zz), &out) != ARIADNA_UWB_LINK_OK,
          "all 0x00 (SDA stuck low) is rejected");
    check(ariadna_uwb_link_decode(buf, sizeof(buf) - 1, &out) == ARIADNA_UWB_LINK_E_SHORT,
          "a short read is rejected");
    check(memcmp(&out, &keep, sizeof(out)) == 0, "a rejected frame leaves the output untouched");

    int flips_caught = 0;
    for (size_t byte = 0; byte < sizeof(buf); byte++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t bad[sizeof(buf)];
            memcpy(bad, buf, sizeof(buf));
            bad[byte] ^= (uint8_t)(1u << bit);
            if (ariadna_uwb_link_decode(bad, sizeof(bad), &out) != ARIADNA_UWB_LINK_OK)
                flips_caught++;
        }
    }
    check(flips_caught == (int)sizeof(buf) * 8, "every single-bit error is caught");

    uint8_t v2[sizeof(buf)];
    memcpy(v2, buf, sizeof(buf));
    v2[1] = ARIADNA_UWB_LINK_VERSION + 1;
    ariadna_uwb_link_put16(v2 + sizeof(v2) - 2, ariadna_uwb_link_crc(v2, sizeof(v2) - 2));
    check(ariadna_uwb_link_decode(v2, sizeof(v2), &out) == ARIADNA_UWB_LINK_E_VERSION,
          "a tag running a different protocol version is refused, not misread");

    // --- slots that must never decode as a range -----------------------
    printf("unusable ranges\n");
    sample(&in);
    in.slot[0].range_m = NAN;
    in.slot[1].range_m = -1.0f;
    in.slot[2].range_m = 700.0f;          // beyond 655.34 m
    in.slot[3].addr    = 0;               // range without an anchor
    in.slot[4].addr    = 9;               // anchor without `valid`
    in.slot[4].range_m = 5.0f;
    ariadna_uwb_link_encode(&in, buf);
    check(ariadna_uwb_link_decode(buf, sizeof(buf), &out) == ARIADNA_UWB_LINK_OK, "decodes");
    check(out.n_valid == 0, "none of them is reported as a range");
    for (int i = 0; i < ARIADNA_UWB_LINK_SLOTS; i++)
        check(!out.slot[i].valid && out.slot[i].range_m == 0.0f, "slot decodes empty");

    check(ariadna_uwb_link_cm(0.0f) == 0, "0 m is a range");
    check(ariadna_uwb_link_cm(655.34f) == 65534, "655.34 m is the longest range");
    check(ariadna_uwb_link_cm(655.35f) == ARIADNA_UWB_RANGE_NONE, "and one more is none");

    // --- the link's CRC is the vehicle's CRC ---------------------------
    const uint8_t probe[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    check(ariadna_uwb_link_crc(probe, sizeof(probe)) == 0x29B1,
          "CRC-16/CCITT-FALSE check value, same as ariadna_crc16()");

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
