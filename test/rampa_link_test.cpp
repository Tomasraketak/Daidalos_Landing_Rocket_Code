// =====================================================================
//  Ariadna V2 — Rampa umbilical I2C & bidirectional LoRa TDMA checks
//
//  Verifies:
//    * ariadna_rampa_link.h (Rocket <-> Rampa I2C cmd/status codec + CRC)
//    * ariadna_lora_sync_t / ariadna_lora_uplink_t codec + CRC
//    * ariadna_lora_next_window_ms() 10 s + mandatory T-1 s final abort window
//
//  Build: make rampa
// =====================================================================
#include "../Ariadna_V2/common/ariadna_link.h"
#include "../Ariadna_V2/common/ariadna_rampa_link.h"

#include <cstdio>
#include <cstring>

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

int main()
{
    printf("Ariadna V2 Rampa umbilical & bidirectional LoRa TDMA\n");
    printf("====================================================\n");

    // ---- 1. Rampa command frame (16 B) --------------------------------
    printf("rampa command codec\n");
    ariadna_rampa_cmd_t cmd_in{}, cmd_out{}, cmd_keep{};
    memset(&cmd_keep, 0x5A, sizeof(cmd_keep));
    cmd_in.cmd          = RAMPA_CMD_COUNTDOWN;
    cmd_in.seq          = 42;
    cmd_in.t_rem_ms     = 59950;
    cmd_in.utc_ms       = 43210987u;
    cmd_in.rocket_flags = ARIADNA_RK_POST_OK | ARIADNA_RK_NAV_VALID | ARIADNA_RK_GPS_UTC;
    cmd_in.pyro_ch      = 1;
    ariadna_rampa_cmd_seal(&cmd_in);

    uint8_t cbuf[ARIADNA_RAMPA_CMD_SIZE];
    memcpy(cbuf, &cmd_in, sizeof(cbuf));
    check(ariadna_rampa_cmd_decode(cbuf, sizeof(cbuf), &cmd_out) == ARIADNA_RAMPA_OK, "cmd decodes");
    check(memcmp(&cmd_in, &cmd_out, sizeof(cmd_in)) == 0, "cmd round-trip exact");

    cmd_out = cmd_keep;
    uint8_t ff[32], zz[32];
    memset(ff, 0xFF, sizeof(ff));
    memset(zz, 0x00, sizeof(zz));
    check(ariadna_rampa_cmd_decode(ff, sizeof(cbuf), &cmd_out) != ARIADNA_RAMPA_OK, "cmd 0xFF rejected");
    check(ariadna_rampa_cmd_decode(zz, sizeof(cbuf), &cmd_out) != ARIADNA_RAMPA_OK, "cmd 0x00 rejected");
    check(ariadna_rampa_cmd_decode(cbuf, sizeof(cbuf) - 1, &cmd_out) == ARIADNA_RAMPA_E_SHORT, "cmd short rejected");
    check(memcmp(&cmd_out, &cmd_keep, sizeof(cmd_out)) == 0, "rejected cmd leaves output untouched");

    int cflips = 0;
    for (size_t b = 0; b < sizeof(cbuf); b++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t bad[sizeof(cbuf)];
            memcpy(bad, cbuf, sizeof(cbuf));
            bad[b] ^= (uint8_t)(1u << bit);
            if (ariadna_rampa_cmd_decode(bad, sizeof(bad), &cmd_out) != ARIADNA_RAMPA_OK)
                cflips++;
        }
    }
    check(cflips == (int)sizeof(cbuf) * 8, "cmd catches every single-bit flip");

    // ---- 2. Rampa status frame (20 B) ---------------------------------
    printf("rampa status codec\n");
    ariadna_rampa_status_t st_in{}, st_out{}, st_keep{};
    memset(&st_keep, 0xA5, sizeof(st_keep));
    st_in.state      = RAMPA_STATE_COUNTDOWN;
    st_in.last_seq   = 42;
    st_in.flags      = ARIADNA_RMP_KEY_ARMED | ARIADNA_RMP_CONT1 | ARIADNA_RMP_LORA_OFF | ARIADNA_RMP_READY;
    st_in.fault_mask = 0;
    st_in.vbat_mv    = 8120;
    st_in.t_rem_ms   = 59948;
    st_in.cont_mv[0] = 1420;
    st_in.cont_mv[1] = 15;
    st_in.cont_mv[2] = 0;
    ariadna_rampa_status_seal(&st_in);

    uint8_t sbuf[ARIADNA_RAMPA_STAT_SIZE];
    memcpy(sbuf, &st_in, sizeof(sbuf));
    check(ariadna_rampa_status_decode(sbuf, sizeof(sbuf), &st_out) == ARIADNA_RAMPA_OK, "status decodes");
    check(memcmp(&st_in, &st_out, sizeof(st_in)) == 0, "status round-trip exact");

    st_out = st_keep;
    check(ariadna_rampa_status_decode(ff, sizeof(sbuf), &st_out) != ARIADNA_RAMPA_OK, "status 0xFF rejected");
    check(ariadna_rampa_status_decode(zz, sizeof(sbuf), &st_out) != ARIADNA_RAMPA_OK, "status 0x00 rejected");
    check(ariadna_rampa_status_decode(sbuf, sizeof(sbuf) - 1, &st_out) == ARIADNA_RAMPA_E_SHORT, "status short rejected");
    check(memcmp(&st_out, &st_keep, sizeof(st_out)) == 0, "rejected status leaves output untouched");

    int sflips = 0;
    for (size_t b = 0; b < sizeof(sbuf); b++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t bad[sizeof(sbuf)];
            memcpy(bad, sbuf, sizeof(sbuf));
            bad[b] ^= (uint8_t)(1u << bit);
            if (ariadna_rampa_status_decode(bad, sizeof(bad), &st_out) != ARIADNA_RAMPA_OK)
                sflips++;
        }
    }
    check(sflips == (int)sizeof(sbuf) * 8, "status catches every single-bit flip");

    // ---- 3. LoRa sync beacon (28 B) & uplink command (20 B) -----------
    printf("bidirectional LoRa sync & uplink codecs\n");
    ariadna_lora_sync_t ls_in{}, ls_out{};
    ls_in.rocket_t_ms     = 123456;
    ls_in.utc_ms          = 36005123;
    ls_in.next_rx_in_ms   = 4877;
    ls_in.rx_dur_ms       = ARIADNA_LORA_WIN_DUR_MS;
    ls_in.t_launch_rem_ms = 55123;
    ls_in.last_cmd_seq    = 7;
    ls_in.last_cmd_rc     = ARIADNA_ULRC_OK;
    ls_in.rampa_state     = RAMPA_STATE_COUNTDOWN;
    ls_in.rampa_flags     = ARIADNA_RMP_KEY_ARMED | ARIADNA_RMP_CONT1;
    ls_in.rampa_fault     = 0;
    ls_in.sync_flags      = ARIADNA_SYNCF_GPS_UTC | ARIADNA_SYNCF_CD_ACTIVE;
    ls_in.rampa_vbat_mv   = 8050;
    ariadna_lora_sync_seal(&ls_in);
    check(ariadna_lora_sync_decode((const uint8_t *)&ls_in, sizeof(ls_in), &ls_out) == 1, "sync decodes");
    check(memcmp(&ls_in, &ls_out, sizeof(ls_in)) == 0, "sync round-trip exact");

    int ls_flips = 0;
    for (size_t b = 0; b < sizeof(ls_in); b++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t bad[sizeof(ls_in)];
            memcpy(bad, &ls_in, sizeof(ls_in));
            bad[b] ^= (uint8_t)(1u << bit);
            if (!ariadna_lora_sync_decode(bad, sizeof(bad), &ls_out)) ls_flips++;
        }
    }
    check(ls_flips == (int)sizeof(ls_in) * 8, "sync catches every single-bit flip");

    ariadna_lora_uplink_t ul_in{}, ul_out{};
    ul_in.seq    = 9;
    ul_in.cmd    = ARIADNA_UL_COUNTDOWN;
    ul_in.utc_ms = 36010055;
    ul_in.arg_ms = 60000;
    ul_in.token  = ARIADNA_UL_TOK_COUNTDOWN;
    ul_in.target = 1;
    ul_in.flags  = ARIADNA_ULF_FINAL_WIN;
    ariadna_lora_uplink_seal(&ul_in);
    check(ariadna_lora_uplink_decode((const uint8_t *)&ul_in, sizeof(ul_in), &ul_out) == 1, "uplink decodes");
    check(memcmp(&ul_in, &ul_out, sizeof(ul_in)) == 0, "uplink round-trip exact");

    int ul_flips = 0;
    for (size_t b = 0; b < sizeof(ul_in); b++) {
        for (int bit = 0; bit < 8; bit++) {
            uint8_t bad[sizeof(ul_in)];
            memcpy(bad, &ul_in, sizeof(ul_in));
            bad[b] ^= (uint8_t)(1u << bit);
            if (!ariadna_lora_uplink_decode(bad, sizeof(bad), &ul_out)) ul_flips++;
        }
    }
    check(ul_flips == (int)sizeof(ul_in) * 8, "uplink catches every single-bit flip");

    // ---- 4. TDMA window scheduling & T-1 s final abort window ---------
    printf("LoRa TDMA window scheduler\n");
    int is_fin = -1;
    uint32_t w = ariadna_lora_next_window_ms(1234, 42345, 1, -1, &is_fin);
    check(w == 7655u && is_fin == 0, "UTC 10 s boundary alignment");

    // Monotonic fallback when GPS UTC is unavailable
    w = ariadna_lora_next_window_ms(12500, ARIADNA_UTC_UNKNOWN, 0, -1, &is_fin);
    check(w == 7500u && is_fin == 0, "monotonic 10 s alignment without GPS");

    // Active countdown: next regular 10 s window is 8000 ms away, but T-1 s
    // final abort window opens at T - 1150 ms = 2850 ms from now!
    w = ariadna_lora_next_window_ms(2000, 2000, 1, 4000, &is_fin);
    check(w == 2850u && is_fin == 1, "mandatory T-1 s final abort window overrides 10 s grid");

    // Regular window near liftoff (T-500 ms) must not overlap detach -> skips past liftoff
    w = ariadna_lora_next_window_ms(9500, 9500, 1, 1000, &is_fin);
    check(w == 10500u && is_fin == 0, "regular window suppressed during final T-1 s..T0 liftoff");

    // Airtime budget: 2 redundant 20 B uplink packets + offsets fit inside 300 ms RX window
    const uint32_t ul_air_ms = (ariadna_lora_airtime_us(sizeof(ariadna_lora_uplink_t)) + 999u) / 1000u;
    const uint32_t sy_air_ms = (ariadna_lora_airtime_us(sizeof(ariadna_lora_sync_t)) + 999u) / 1000u;
    printf("    uplink 20 B airtime: %u ms, sync 28 B airtime: %u ms\n", ul_air_ms, sy_air_ms);
    check(ARIADNA_LORA_UL_OFFSET2_MS + ul_air_ms + 30u < ARIADNA_LORA_WIN_DUR_MS,
          "two redundant uplink packets fit comfortably inside 300 ms RX window");
    check(sy_air_ms < ARIADNA_LORA_TX_GUARD_MS,
          "sync beacon finishes well inside 70 ms TX guard band");

    printf("\n%d checks, %d failures\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
