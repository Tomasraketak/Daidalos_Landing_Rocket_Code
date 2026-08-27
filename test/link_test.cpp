// =====================================================================
//  Host verification of the Ariadna V2 inter-processor link.
//
//  The link codec is shared by four independent programs on three
//  different boards plus a Python ground station. A framing bug there does
//  not announce itself as a compile error — it shows up as a silent packet
//  loss rate on the pad. So it is tested here, on a workstation, exactly
//  the way the guidance module is.
//
//  Build:  cd test && make link
//  Run:    ./link_test
// =====================================================================
#include "../Ariadna_V2/common/ariadna_link.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>

static int g_checks = 0;
static int g_fails  = 0;

static void check(bool ok, const char *what)
{
    g_checks++;
    if (!ok) {
        g_fails++;
        printf("  FAIL  %s\n", what);
    }
}

// Feed a whole buffer through the decoder, collecting every frame that
// passes. Mirrors how the real receivers drain their UART.
struct decoded {
    uint8_t type;
    std::vector<uint8_t> payload;
};

static std::vector<decoded> drain(ariadna_rx_t *rx, const uint8_t *p, size_t n)
{
    std::vector<decoded> out;
    for (size_t i = 0; i < n; i++) {
        const uint8_t *pl = nullptr;
        uint8_t len = 0;
        uint8_t t = ariadna_rx_byte(rx, p[i], &pl, &len);
        if (t) {
            decoded d;
            d.type = t;
            d.payload.assign(pl, pl + len);
            out.push_back(d);
        }
    }
    return out;
}

// ---------------------------------------------------------------------
//  1. Struct layout
//
//  These sizes are also asserted at compile time inside ariadna_link.h.
//  Repeating the offsets here catches the other half of the problem: a
//  field REORDERED without changing the total size, which the size assert
//  cannot see but which silently swaps two telemetry channels.
// ---------------------------------------------------------------------
static void test_layout(void)
{
    printf("layout\n");

    check(sizeof(ariadna_nav_state_t)     == 88, "nav_state is 88 bytes");
    check(sizeof(ariadna_actuator_t)      == 36, "actuator is 36 bytes");
    check(sizeof(ariadna_nav_status_t)    == 48, "nav_status is 48 bytes");
    check(sizeof(ariadna_pos_status_t)    == 80, "pos_status is 80 bytes");
    check(sizeof(ariadna_master_state_t)  == 16, "master_state is 16 bytes");
    check(sizeof(ariadna_cmd_t)           ==  8, "cmd is 8 bytes");
    check(sizeof(ariadna_tlm_t)           == 47, "telemetry is 47 bytes");
    check(sizeof(ariadna_gs_serial_t)     == 49, "gs serial is 49 bytes");

    // Every inter-MCU struct must be a multiple of 4 so that the packed
    // and natural layouts agree and no member is ever misaligned.
    check(sizeof(ariadna_nav_state_t)    % 4 == 0, "nav_state 4-byte multiple");
    check(sizeof(ariadna_actuator_t)     % 4 == 0, "actuator 4-byte multiple");
    check(sizeof(ariadna_nav_status_t)   % 4 == 0, "nav_status 4-byte multiple");
    check(sizeof(ariadna_pos_status_t)   % 4 == 0, "pos_status 4-byte multiple");
    check(sizeof(ariadna_master_state_t) % 4 == 0, "master_state 4-byte multiple");
    check(sizeof(ariadna_cmd_t)          % 4 == 0, "cmd 4-byte multiple");

    // Telemetry field offsets. This is the contract ground_visualizer.py
    // decodes with "<Iii" + "h"*11 + "B"*13 + "bb". If any line here moves,
    // that format string is wrong and the ground station shows garbage.
    check(offsetof(ariadna_tlm_t, t_ms)       ==  0, "tlm.t_ms at 0");
    check(offsetof(ariadna_tlm_t, lat)        ==  4, "tlm.lat at 4");
    check(offsetof(ariadna_tlm_t, lon)        ==  8, "tlm.lon at 8");
    check(offsetof(ariadna_tlm_t, alt_dm)     == 12, "tlm.alt_dm at 12");
    check(offsetof(ariadna_tlm_t, vz_dms)     == 14, "tlm.vz_dms at 14");
    check(offsetof(ariadna_tlm_t, vh_dms)     == 16, "tlm.vh_dms at 16");
    check(offsetof(ariadna_tlm_t, acc_cg)     == 18, "tlm.acc_cg at 18");
    check(offsetof(ariadna_tlm_t, roll_ddeg)  == 20, "tlm.roll at 20");
    check(offsetof(ariadna_tlm_t, pitch_ddeg) == 22, "tlm.pitch at 22");
    check(offsetof(ariadna_tlm_t, yaw_ddeg)   == 24, "tlm.yaw at 24");
    check(offsetof(ariadna_tlm_t, temp_dc)    == 26, "tlm.temp at 26");
    check(offsetof(ariadna_tlm_t, tvc_x_cdeg) == 28, "tlm.tvc_x at 28");
    check(offsetof(ariadna_tlm_t, tvc_y_cdeg) == 30, "tlm.tvc_y at 30");
    check(offsetof(ariadna_tlm_t, thrust_n)   == 32, "tlm.thrust at 32");
    check(offsetof(ariadna_tlm_t, version)    == 34, "tlm.version at 34");
    check(offsetof(ariadna_tlm_t, state)      == 35, "tlm.state at 35");
    check(offsetof(ariadna_tlm_t, nav_phase)  == 36, "tlm.nav_phase at 36");
    check(offsetof(ariadna_tlm_t, clamp_pct)  == 37, "tlm.clamp at 37");
    check(offsetof(ariadna_tlm_t, vbat_cv)    == 38, "tlm.vbat at 38");
    check(offsetof(ariadna_tlm_t, curr_da)    == 39, "tlm.curr at 39");
    check(offsetof(ariadna_tlm_t, hdop_x10)   == 40, "tlm.hdop at 40");
    check(offsetof(ariadna_tlm_t, sats)       == 41, "tlm.sats at 41");
    check(offsetof(ariadna_tlm_t, pyro)       == 42, "tlm.pyro at 42");
    check(offsetof(ariadna_tlm_t, health)     == 43, "tlm.health at 43");
    check(offsetof(ariadna_tlm_t, flags)      == 44, "tlm.flags at 44");
    check(offsetof(ariadna_tlm_t, exp)        == 45, "tlm.exp at 45");
    check(offsetof(ariadna_tlm_t, uwb)        == 46, "tlm.uwb at 46");

    check(offsetof(ariadna_gs_serial_t, rssi) == 47, "gs.rssi at 47");
    check(offsetof(ariadna_gs_serial_t, snr)  == 48, "gs.snr at 48");

    // Nothing may exceed the frame buffer.
    check(sizeof(ariadna_nav_state_t) <= ARIADNA_PAYLOAD_MAX, "nav_state fits a frame");
    check(sizeof(ariadna_gs_serial_t) <= ARIADNA_PAYLOAD_MAX, "gs serial fits a frame");
}

// ---------------------------------------------------------------------
//  2. CRC
//
//  Pinned against the published CRC-16/CCITT-FALSE check value so this can
//  never silently drift away from the routine already flying in
//  Daidalos_Landing_Guidance.ino.
// ---------------------------------------------------------------------
static void test_crc(void)
{
    printf("crc\n");

    const uint8_t vec[] = "123456789";
    check(ariadna_crc16(vec, 9) == 0x29B1, "CCITT-FALSE check value 0x29B1");
    check(ariadna_crc16(nullptr, 0) == 0xFFFF, "empty message is the init value");

    // Single-bit sensitivity: every one-bit flip in a 46-byte payload must
    // change the CRC. CCITT-FALSE guarantees this; the test guards against
    // a broken transcription of the polynomial that still looks plausible.
    uint8_t buf[46];
    for (int i = 0; i < 46; i++) buf[i] = (uint8_t)(i * 7 + 3);
    uint16_t base = ariadna_crc16(buf, 46);
    int misses = 0;
    for (int byte = 0; byte < 46; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            buf[byte] ^= (uint8_t)(1u << bit);
            if (ariadna_crc16(buf, 46) == base) misses++;
            buf[byte] ^= (uint8_t)(1u << bit);
        }
    }
    check(misses == 0, "every single-bit flip changes the CRC");
}

// ---------------------------------------------------------------------
//  3. Round trip
// ---------------------------------------------------------------------
static void test_roundtrip(void)
{
    printf("round trip\n");

    ariadna_nav_state_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.t_ms  = 123456;
    tx.px    = -12.5f; tx.py = 33.25f; tx.pz = 84.75f;
    tx.vx    = 1.5f;  tx.vy = -2.25f;  tx.vz = -31.5f;
    tx.ax    = 0.125f; tx.ay = -0.5f;  tx.az = -9.0f;
    tx.qw    = 0.9990482f; tx.qx = 0.0436194f; tx.qy = 0.0f; tx.qz = 0.0f;
    tx.ux    = 0.0871557f; tx.uy = 0.0f; tx.uz = 0.9961947f;   // 5 deg tilt
    tx.wx    = 0.25f; tx.wy = -0.125f; tx.wz = 0.0f;
    tx.accel_mag = 1.02f;
    tx.flags = ARIADNA_NAV_VALID | ARIADNA_NAV_BARO_OK | ARIADNA_NAV_GPS_FIX;
    tx.quality = 240;

    uint8_t frame[ARIADNA_FRAME_MAX];
    uint16_t n = ariadna_encode(frame, ARIADNA_PKT_NAV_STATE, &tx, sizeof(tx));
    check(n == ARIADNA_HDR_LEN + sizeof(tx) + ARIADNA_CRC_LEN, "frame length");
    check(frame[0] == ARIADNA_SYNC0 && frame[1] == ARIADNA_SYNC1, "sync bytes");
    check(frame[2] == ARIADNA_PKT_NAV_STATE, "type byte");
    check(frame[3] == sizeof(tx), "length byte");

    ariadna_rx_t rx;
    ariadna_rx_init(&rx);
    auto got = drain(&rx, frame, n);

    check(got.size() == 1, "exactly one frame decoded");
    if (got.size() == 1) {
        check(got[0].type == ARIADNA_PKT_NAV_STATE, "decoded type matches");
        ariadna_nav_state_t out;
        memset(&out, 0xAA, sizeof(out));
        check(ariadna_unpack(got[0].payload.data(), (uint8_t)got[0].payload.size(),
                             &out, sizeof(out)) == 1, "unpack accepts right size");
        check(memcmp(&tx, &out, sizeof(tx)) == 0, "payload survives the round trip");
        check(out.pz == 84.75f && out.vz == -31.5f, "floats are bit exact");
        check(out.px == -12.5f && out.az == -9.0f && out.qw == 0.9990482f,
              "position, acceleration and quaternion are bit exact");
        check(out.quality == 240, "trailing byte survives");
    }

    check(rx.rx_ok == 1 && rx.rx_crc == 0 && rx.rx_len == 0, "counters clean");

    // A payload larger than the frame buffer must be refused, not truncated.
    uint8_t big[ARIADNA_PAYLOAD_MAX + 1] = {0};
    check(ariadna_encode(frame, ARIADNA_PKT_NAV_STATE, big,
                         ARIADNA_PAYLOAD_MAX + 1) == 0, "oversize payload refused");

    // Zero-length frames are legal (a bare command / keepalive).
    n = ariadna_encode(frame, ARIADNA_PKT_CMD, nullptr, 0);
    check(n == ARIADNA_HDR_LEN + ARIADNA_CRC_LEN, "zero-length frame length");
    ariadna_rx_init(&rx);
    got = drain(&rx, frame, n);
    check(got.size() == 1 && got[0].payload.empty(), "zero-length frame decodes");
}

// ---------------------------------------------------------------------
//  4. Every packet type
// ---------------------------------------------------------------------
template <typename T>
static void roundtrip_one(uint8_t type, const char *name)
{
    T tx;
    // Fill with a non-trivial pattern so a byte swap or a dropped field
    // cannot pass by coincidence.
    uint8_t *raw = reinterpret_cast<uint8_t *>(&tx);
    for (size_t i = 0; i < sizeof(T); i++) raw[i] = (uint8_t)(i * 31 + 17);

    uint8_t frame[ARIADNA_FRAME_MAX];
    uint16_t n = ariadna_encode(frame, type, &tx, (uint8_t)sizeof(T));

    ariadna_rx_t rx;
    ariadna_rx_init(&rx);
    auto got = drain(&rx, frame, n);

    bool ok = got.size() == 1 && got[0].type == type &&
              got[0].payload.size() == sizeof(T) &&
              memcmp(got[0].payload.data(), &tx, sizeof(T)) == 0;
    check(ok, name);
}

static void test_all_types(void)
{
    printf("all packet types\n");
    roundtrip_one<ariadna_nav_state_t>(ARIADNA_PKT_NAV_STATE,   "NAV_STATE round trip");
    roundtrip_one<ariadna_pos_status_t>(ARIADNA_PKT_POS_STATUS, "POS_STATUS round trip");
    roundtrip_one<ariadna_actuator_t>(ARIADNA_PKT_ACTUATOR,     "ACTUATOR round trip");
    roundtrip_one<ariadna_nav_status_t>(ARIADNA_PKT_NAV_STATUS, "NAV_STATUS round trip");
    roundtrip_one<ariadna_master_state_t>(ARIADNA_PKT_MASTER_STATE, "MASTER_STATE round trip");
    roundtrip_one<ariadna_cmd_t>(ARIADNA_PKT_CMD,               "CMD round trip");
    roundtrip_one<ariadna_gs_serial_t>(ARIADNA_PKT_TELEMETRY,   "GS_SERIAL round trip");
}

// ---------------------------------------------------------------------
//  5. Corruption and resynchronisation
//
//  The behaviour that actually matters in flight: after ANY amount of
//  garbage, the very next good frame must be accepted. A parser that needs
//  a quiet line to recover will never recover on a busy one.
// ---------------------------------------------------------------------
static void test_corruption(void)
{
    printf("corruption and resync\n");

    ariadna_actuator_t act;
    memset(&act, 0, sizeof(act));
    act.t_ms = 999; act.tvc_x_deg = 3.5f; act.clamp = 0.75f;
    act.flags = ARIADNA_ACT_VALID;

    uint8_t good[ARIADNA_FRAME_MAX];
    uint16_t glen = ariadna_encode(good, ARIADNA_PKT_ACTUATOR, &act, sizeof(act));

    // --- a corrupted payload byte must be rejected, not delivered -----
    for (uint16_t i = 0; i < glen - 2; i++) {
        uint8_t bad[ARIADNA_FRAME_MAX];
        memcpy(bad, good, glen);
        bad[i] ^= 0x40;

        ariadna_rx_t rx;
        ariadna_rx_init(&rx);
        auto got = drain(&rx, bad, glen);

        // A flipped bit either fails the sync hunt, fails the length check,
        // or fails the CRC. What must never happen is a frame that decodes
        // to something other than what was sent.
        bool leaked = false;
        for (auto &d : got)
            if (d.type == ARIADNA_PKT_ACTUATOR &&
                (d.payload.size() != sizeof(act) ||
                 memcmp(d.payload.data(), &act, sizeof(act)) != 0))
                leaked = true;
        if (leaked) { check(false, "corrupted frame delivered as valid"); return; }
    }
    check(true, "no corrupted frame is ever delivered");

    // --- resync after arbitrary garbage --------------------------------
    //
    // Each pattern is followed by THREE good frames. The guarantee being
    // tested is "resynchronises within one frame": at least two of the
    // three must be recovered intact.
    //
    // Three, not one, because this is a length-prefixed protocol. Garbage
    // that happens to look like a valid header with a large length makes
    // the parser wait for bytes that are still in flight, and it cannot
    // know the frame is bogus until the CRC over it fails. That is
    // inherent, not a defect — what matters is that recovery then happens
    // immediately, from the bytes already buffered, rather than after the
    // next quiet moment on the line.
    const uint8_t nasty[][6] = {
        { 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5 },   // repeated lead byte
        { 0xA5, 0x1D, 0xFF, 0xFF, 0x00, 0x00 },   // plausible header, bad length
        { 0x00, 0xFF, 0xA5, 0x1D, 0x10, 0x58 },   // header claiming 88 bytes
        { 0xA5, 0x1D, 0x20, 0x24, 0x00, 0x00 },   // right header, frame cut short
    };

    for (int k = 0; k < 4; k++) {
        std::vector<uint8_t> stream(nasty[k], nasty[k] + 6);
        for (int r = 0; r < 3; r++)
            stream.insert(stream.end(), good, good + glen);

        ariadna_rx_t rx;
        ariadna_rx_init(&rx);
        auto got = drain(&rx, stream.data(), stream.size());

        int intact = 0;
        for (auto &d : got)
            if (d.type == ARIADNA_PKT_ACTUATOR &&
                d.payload.size() == sizeof(act) &&
                memcmp(d.payload.data(), &act, sizeof(act)) == 0) intact++;

        char msg[96];
        snprintf(msg, sizeof msg,
                 "resyncs within one frame after garbage pattern %d (%d/3)", k, intact);
        check(intact >= 2, msg);
    }

    // --- randomly corrupted stream --------------------------------------
    //
    // The realistic failure mode is not a crafted pattern, it is noise. A
    // long stream with 1 % of its bytes corrupted must still deliver almost
    // everything, and must NEVER deliver a frame whose contents differ from
    // what was sent. The second property is the one that matters: a wrong
    // servo command accepted as valid is worse than a dropped one.
    {
        const int N_FRAMES = 2000;
        std::vector<uint8_t> stream;
        std::vector<uint32_t> sent;
        uint8_t frame[ARIADNA_FRAME_MAX];

        for (int i = 0; i < N_FRAMES; i++) {
            ariadna_actuator_t a;
            memset(&a, 0, sizeof(a));
            a.t_ms = (uint32_t)i;
            a.tvc_x_deg = (float)i * 0.01f;
            a.clamp = 0.5f;
            uint16_t n = ariadna_encode(frame, ARIADNA_PKT_ACTUATOR, &a, sizeof(a));
            stream.insert(stream.end(), frame, frame + n);
            sent.push_back(a.t_ms);
        }

        // Deterministic corruption: a fixed LCG so a failure is reproducible.
        uint32_t seed = 0xA51D2024u;
        int corrupted = 0;
        for (size_t i = 0; i < stream.size(); i++) {
            seed = seed * 1664525u + 1013904223u;
            if ((seed >> 16) % 100u == 0u) {        // ~1 % of bytes
                stream[i] ^= (uint8_t)(1u << ((seed >> 8) & 7u));
                corrupted++;
            }
        }

        ariadna_rx_t rx;
        ariadna_rx_init(&rx);
        auto got = drain(&rx, stream.data(), stream.size());

        int good_frames = 0, bad_frames = 0;
        for (auto &d : got) {
            if (d.type != ARIADNA_PKT_ACTUATOR || d.payload.size() != sizeof(act)) {
                bad_frames++;
                continue;
            }
            ariadna_actuator_t a;
            memcpy(&a, d.payload.data(), sizeof(a));
            // Reconstruct what this frame should have been and compare in
            // full. Anything that does not match exactly is a leak.
            ariadna_actuator_t want;
            memset(&want, 0, sizeof(want));
            want.t_ms = a.t_ms;
            want.tvc_x_deg = (float)a.t_ms * 0.01f;
            want.clamp = 0.5f;
            if (a.t_ms < (uint32_t)N_FRAMES && memcmp(&a, &want, sizeof(a)) == 0)
                good_frames++;
            else
                bad_frames++;
        }

        printf("    %d frames, %d bytes corrupted -> %d intact, %d leaked\n",
               N_FRAMES, corrupted, good_frames, bad_frames);

        check(bad_frames == 0, "no corrupted frame is ever delivered as valid");
        // Every frame containing a flipped bit is legitimately lost. With
        // ~1 % of bytes hit and 42-byte frames, roughly a third of frames
        // carry damage, so half getting through is a floor, not a target.
        check(good_frames > N_FRAMES / 2, "over half the frames survive 1 % byte errors");
    }

    // --- byte-at-a-time arrival, split across reads ---------------------
    // Real UART reads deliver arbitrary fragments. Feeding one byte per
    // call is already what drain() does, so this checks the other half:
    // that decoder state survives being interleaved with unrelated bytes
    // between frames.
    {
        ariadna_rx_t rx;
        ariadna_rx_init(&rx);
        int decoded_count = 0;
        for (int rep = 0; rep < 20; rep++) {
            uint8_t filler = (uint8_t)(rep * 13);
            if (filler != ARIADNA_SYNC0) {
                const uint8_t *pl; uint8_t l;
                ariadna_rx_byte(&rx, filler, &pl, &l);
            }
            auto got = drain(&rx, good, glen);
            decoded_count += (int)got.size();
        }
        check(decoded_count == 20, "20 frames with filler between them all decode");
    }
}

// ---------------------------------------------------------------------
//  6. Interleaved multi-type stream
//
//  What the Master's UART actually looks like: actuator frames at 200 Hz
//  with status frames folded in at 20 Hz.
// ---------------------------------------------------------------------
static void test_interleaved(void)
{
    printf("interleaved stream\n");

    std::vector<uint8_t> stream;
    uint8_t frame[ARIADNA_FRAME_MAX];

    int n_act = 0, n_status = 0;
    for (int i = 0; i < 100; i++) {
        ariadna_actuator_t a;
        memset(&a, 0, sizeof(a));
        a.t_ms = (uint32_t)i;
        a.tvc_x_deg = (float)i * 0.1f;
        uint16_t n = ariadna_encode(frame, ARIADNA_PKT_ACTUATOR, &a, sizeof(a));
        stream.insert(stream.end(), frame, frame + n);
        n_act++;

        if (i % 10 == 0) {
            ariadna_nav_status_t s;
            memset(&s, 0, sizeof(s));
            s.t_ms = (uint32_t)i;
            s.plan_iters = (uint32_t)i * 7;
            n = ariadna_encode(frame, ARIADNA_PKT_NAV_STATUS, &s, sizeof(s));
            stream.insert(stream.end(), frame, frame + n);
            n_status++;
        }
    }

    ariadna_rx_t rx;
    ariadna_rx_init(&rx);
    auto got = drain(&rx, stream.data(), stream.size());

    int got_act = 0, got_status = 0;
    bool order_ok = true;
    uint32_t last_act = 0;
    for (auto &d : got) {
        if (d.type == ARIADNA_PKT_ACTUATOR) {
            ariadna_actuator_t a;
            ariadna_unpack(d.payload.data(), (uint8_t)d.payload.size(), &a, sizeof(a));
            if (got_act && a.t_ms != last_act + 1) order_ok = false;
            last_act = a.t_ms;
            got_act++;
        } else if (d.type == ARIADNA_PKT_NAV_STATUS) {
            got_status++;
        }
    }

    check(got_act == n_act, "every actuator frame decoded");
    check(got_status == n_status, "every status frame decoded");
    check(order_ok, "frames arrive in order with nothing dropped");
    check(rx.rx_crc == 0 && rx.rx_len == 0, "no spurious rejections");
}

// ---------------------------------------------------------------------
//  7. Length-mismatch rejection
//
//  The version-skew case: a valid frame carrying the wrong-sized payload
//  for its type must be dropped, not read off the end of the buffer.
// ---------------------------------------------------------------------
static void test_length_mismatch(void)
{
    printf("length mismatch\n");

    uint8_t stub[16] = {0};
    uint8_t frame[ARIADNA_FRAME_MAX];
    uint16_t n = ariadna_encode(frame, ARIADNA_PKT_NAV_STATE, stub, 16);

    ariadna_rx_t rx;
    ariadna_rx_init(&rx);
    auto got = drain(&rx, frame, n);

    check(got.size() == 1, "short frame still passes the CRC");
    if (got.size() == 1) {
        ariadna_nav_state_t out;
        int ok = ariadna_unpack(got[0].payload.data(),
                                (uint8_t)got[0].payload.size(),
                                &out, sizeof(out));
        check(ok == 0, "unpack refuses a wrong-sized payload");
    }
}

// ---------------------------------------------------------------------
//  8. Link budget
//
//  Not a correctness test — an arithmetic guard. Add a field to a packet
//  and this is what tells you whether it still fits in the time available.
// ---------------------------------------------------------------------
static void test_link_budget(void)
{
    printf("link budget\n");

    const double byte_time = 10.0 / (double)ARIADNA_MCU_BAUD;   // 8N1

    double nav_load = (ARIADNA_HDR_LEN + sizeof(ariadna_nav_state_t) + ARIADNA_CRC_LEN)
                    * ARIADNA_NAV_STATE_HZ * byte_time;
    double act_load = (ARIADNA_HDR_LEN + sizeof(ariadna_actuator_t) + ARIADNA_CRC_LEN)
                    * ARIADNA_ACTUATOR_HZ * byte_time;
    double sts_load = (ARIADNA_HDR_LEN + sizeof(ariadna_pos_status_t) + ARIADNA_CRC_LEN)
                    * ARIADNA_STATUS_HZ * byte_time;
    double nst_load = (ARIADNA_HDR_LEN + sizeof(ariadna_nav_status_t) + ARIADNA_CRC_LEN)
                    * ARIADNA_STATUS_HZ * byte_time;

    printf("    Position->Master   nav state @%3d Hz + status : %5.1f %%\n",
           ARIADNA_NAV_STATE_HZ, (nav_load + sts_load) * 100.0);
    printf("    Master->Navigation nav state @%3d Hz          : %5.1f %%\n",
           ARIADNA_NAV_STATE_HZ, nav_load * 100.0);
    printf("    Navigation->Master actuator @%3d Hz + status  : %5.1f %%\n",
           ARIADNA_ACTUATOR_HZ, (act_load + nst_load) * 100.0);

    // Half the link is the working limit. Past that, one burst of status
    // traffic starts delaying a control packet.
    check(nav_load + sts_load < 0.5, "Position uplink under 50 % of the link");
    check(nav_load < 0.5,            "Navigation downlink under 50 % of the link");
    check(act_load + nst_load < 0.5, "actuator + status under 50 % of the link");

    // The producer must outrun the consumer, or a control tick occasionally
    // finds no new state. This is the whole reason the estimator runs at
    // 240 Hz against a 200 Hz loop.
    check(ARIADNA_NAV_STATE_HZ > ARIADNA_ACTUATOR_HZ,
          "nav state is produced faster than the control loop consumes it");

    // A floor as well as a ratio. The full state — attitude quaternion,
    // local position, velocity and world-frame acceleration — is required
    // at 100 Hz minimum and specified at 240 Hz. Pinning the floor here
    // means a future field that forces the rate down fails on a
    // workstation instead of quietly halving the navigation bandwidth.
    check(ARIADNA_NAV_STATE_HZ >= 100,
          "nav state rate meets the 100 Hz floor");

    // Headroom, not just fit. A packet that exactly fills the frame leaves
    // nowhere to put the next field, and this one has grown twice.
    check(sizeof(ariadna_nav_state_t) <= ARIADNA_PAYLOAD_MAX - 4,
          "nav state leaves room to grow");

    // ---- LoRa ----------------------------------------------------------
    // ariadna_lora_airtime_us() is integer arithmetic on the target, so it
    // is checked against the datasheet formula in floating point here. A
    // drift between the two would silently mis-state the duty cycle.
    {
        const double t_sym = (double)(1u << ARIADNA_LORA_SF) / ARIADNA_LORA_BW_HZ;
        for (uint16_t pl = 8; pl <= 64; pl += 8) {
            double num = 8.0 * pl - 4.0 * ARIADNA_LORA_SF + 28 + 16;
            double den = 4.0 * ARIADNA_LORA_SF;
            double blocks = ceil(num / den);
            if (blocks < 0) blocks = 0;
            double n_payload = 8 + blocks * (ARIADNA_LORA_CR4 - 4 + 4);
            double ref = ((ARIADNA_LORA_PREAMBLE + 4.25) + n_payload) * t_sym * 1e6;
            double got = (double)ariadna_lora_airtime_us(pl);
            if (fabs(got - ref) > 50.0) {
                char msg[96];
                snprintf(msg, sizeof msg,
                         "airtime(%u) = %.0f us, datasheet says %.0f", pl, got, ref);
                check(false, msg);
            }
        }
        check(true, "integer airtime matches the datasheet formula");
    }

    const uint32_t air_us = ariadna_lora_airtime_us(sizeof(ariadna_tlm_t));
    const double duty = air_us * 1e-6 * ARIADNA_TLM_HZ;

    printf("    LoRa %2u B at SF%d/%luk/4-%d : %5.1f ms on air, %4.1f %% duty at %d Hz\n",
           (unsigned)sizeof(ariadna_tlm_t), ARIADNA_LORA_SF,
           (unsigned long)(ARIADNA_LORA_BW_HZ / 1000), ARIADNA_LORA_CR4,
           air_us / 1000.0, duty * 100.0, ARIADNA_TLM_HZ);

    // The transmitter must be idle at least half the time. Below that the
    // radio has no room to recover from a retry, and the ground station
    // has no gap in which to be doing anything else.
    check(duty < 0.5, "downlink duty cycle leaves the radio idle half the time");

    // And the packet must physically fit in its slot with room to spare.
    check(air_us < (1000000u / ARIADNA_TLM_HZ),
          "one packet fits inside one downlink period");

    if (duty > 0.10)
        printf("    NOTE: %.0f %% duty exceeds the EU 433 MHz ISM 10 %% limit -- see\n"
               "          the regulatory note in ariadna_link.h before flying this\n",
               duty * 100.0);
}

// ---------------------------------------------------------------------
//  9. Transmit ring
//
//  The DMA binding cannot be tested on a workstation, but the bookkeeping
//  can — and the bookkeeping is where the damage is. An off-by-one here
//  does not crash; it drops or duplicates a servo command.
//
//  The fake consumer below does exactly what the DMA channel does: takes
//  the run it is given, copies it out, reports completion. If the ring is
//  correct against this, the only thing left to get wrong on hardware is
//  the channel configuration.
// ---------------------------------------------------------------------
static void test_tx_ring(void)
{
    printf("transmit ring\n");

    ariadna_tx_ring_t r;
    ariadna_tx_init(&r);

    check(ariadna_tx_used(&r) == 0, "starts empty");
    {
        const uint8_t *p = nullptr;
        check(ariadna_tx_next(&r, &p) == 0, "nothing to send when empty");
    }

    // --- byte-exact FIFO order across many wraps -----------------------
    //
    // The ring is 512 bytes and an actuator frame is 42, so this pushes
    // far more than the buffer holds and drains as it goes — every wrap
    // boundary is crossed repeatedly.
    {
        ariadna_tx_init(&r);
        std::vector<uint8_t> expect, got;
        uint8_t frame[ARIADNA_FRAME_MAX];

        for (int i = 0; i < 500; i++) {
            ariadna_actuator_t a;
            memset(&a, 0, sizeof(a));
            a.t_ms = (uint32_t)i;
            a.tvc_x_deg = (float)i;
            uint16_t n = ariadna_encode(frame, ARIADNA_PKT_ACTUATOR, &a, sizeof(a));

            if (ariadna_tx_push(&r, frame, n))
                expect.insert(expect.end(), frame, frame + n);

            // Drain a run most of the time, so the ring stays partly full
            // and the head chases the tail around the buffer.
            if (i % 3 != 0) {
                const uint8_t *p = nullptr;
                uint16_t run = ariadna_tx_next(&r, &p);
                if (run) {
                    got.insert(got.end(), p, p + run);
                    ariadna_tx_complete(&r);
                }
            }
        }
        // Flush whatever is left.
        for (int guard = 0; guard < 64; guard++) {
            const uint8_t *p = nullptr;
            uint16_t run = ariadna_tx_next(&r, &p);
            if (!run) break;
            got.insert(got.end(), p, p + run);
            ariadna_tx_complete(&r);
        }

        check(got == expect, "every queued byte comes out once, in order");
        check(r.sent == expect.size(), "sent counter matches bytes delivered");
        check(ariadna_tx_used(&r) == 0, "ring drains empty");
        printf("    %zu bytes through a %d byte ring, %u frames dropped\n",
               expect.size(), ARIADNA_TX_RING_SZ, (unsigned)r.dropped);
    }

    // --- a full ring refuses whole frames, never partial ----------------
    {
        ariadna_tx_init(&r);
        uint8_t frame[64];
        for (int i = 0; i < 64; i++) frame[i] = (uint8_t)(i + 1);

        int accepted = 0;
        for (int i = 0; i < 20; i++)
            if (ariadna_tx_push(&r, frame, 64)) accepted++;

        check(accepted == ARIADNA_TX_RING_SZ / 64 - 1 ||
              accepted == ARIADNA_TX_RING_SZ / 64,
              "accepts as many whole frames as fit");
        check(r.dropped == (uint32_t)(20 - accepted), "drops are counted");
        check(ariadna_tx_used(&r) == accepted * 64, "no partial frame queued");

        // Everything that was accepted must still be intact.
        std::vector<uint8_t> got;
        for (int guard = 0; guard < 64; guard++) {
            const uint8_t *p = nullptr;
            uint16_t run = ariadna_tx_next(&r, &p);
            if (!run) break;
            got.insert(got.end(), p, p + run);
            ariadna_tx_complete(&r);
        }
        bool intact = got.size() == (size_t)accepted * 64;
        for (size_t i = 0; intact && i < got.size(); i++)
            if (got[i] != (uint8_t)(i % 64 + 1)) intact = false;
        check(intact, "a full ring does not corrupt what it already held");
    }

    // --- the in-flight region is never overwritten -----------------------
    //
    // The failure this guards against: the producer refilling the buffer
    // while DMA is still reading from it, so the bytes on the wire are not
    // the bytes that were queued. tail does not advance until completion,
    // which is what makes that impossible — this proves it.
    {
        ariadna_tx_init(&r);
        uint8_t a_frame[100], b_frame[100];
        memset(a_frame, 0xAA, sizeof(a_frame));
        memset(b_frame, 0xBB, sizeof(b_frame));

        ariadna_tx_push(&r, a_frame, 100);

        const uint8_t *p = nullptr;
        uint16_t run = ariadna_tx_next(&r, &p);
        check(run == 100, "claims the queued run");

        // Simulate the producer running flat out while the transfer is
        // still going: push until it refuses.
        int pushes = 0;
        while (ariadna_tx_push(&r, b_frame, 100)) pushes++;

        bool untouched = true;
        for (int i = 0; i < 100; i++) if (p[i] != 0xAA) untouched = false;
        check(untouched, "in-flight bytes survive a producer filling the ring");
        check(pushes >= 3, "the rest of the ring is still usable meanwhile");

        // And a second claim must be refused while one is outstanding.
        const uint8_t *q = nullptr;
        check(ariadna_tx_next(&r, &q) == 0, "no second claim while in flight");
    }

    // --- oversize frame ---------------------------------------------------
    {
        ariadna_tx_init(&r);
        static uint8_t huge[ARIADNA_TX_RING_SZ + 8];
        memset(huge, 0x5A, sizeof(huge));
        check(ariadna_tx_push(&r, huge, sizeof(huge)) == 0,
              "a frame larger than the ring is refused");
        check(ariadna_tx_used(&r) == 0, "and leaves the ring untouched");
    }

    // --- sizing sanity ------------------------------------------------------
    // The ring must hold at least a few frames of the busiest stream, or a
    // single blocked transfer starts dropping commands.
    {
        const uint16_t nav_frame = ARIADNA_HDR_LEN + sizeof(ariadna_nav_state_t)
                                 + ARIADNA_CRC_LEN;
        const uint16_t act_frame = ARIADNA_HDR_LEN + sizeof(ariadna_actuator_t)
                                 + ARIADNA_CRC_LEN;
        check(ARIADNA_TX_RING_SZ >= 4 * nav_frame,
              "ring holds at least 4 nav_state frames");
        check(ARIADNA_TX_RING_SZ >= 8 * act_frame,
              "ring holds at least 8 actuator frames");
        check((ARIADNA_TX_RING_SZ & (ARIADNA_TX_RING_SZ - 1)) == 0,
              "ring size is a power of two");
    }
}

int main(void)
{
    printf("\nAriadna V2 link codec\n=====================\n");

    test_layout();
    test_crc();
    test_roundtrip();
    test_all_types();
    test_corruption();
    test_interleaved();
    test_length_mismatch();
    test_tx_ring();
    test_link_budget();

    printf("\n%d checks, %d failures\n\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
