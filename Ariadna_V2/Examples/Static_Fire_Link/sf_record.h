// =====================================================================
//  Ariadna V2 — Static Fire Link: what is stored on Ariadna's flash chip
//
//  Pure data handling, no Arduino dependency; tested on a workstation
//  (`cd test && make sflink`) and compiled unchanged into the sketch.
//
//  One run = one SLOT of 128 kB (32 sectors) in the top 1 MB of the W25Q128,
//  eight slots, used round-robin. The top of the chip is used on purpose:
//  sectors 0-1 hold the flight parameter table and the flight log grows
//  upwards from sector 2, so neither is touched (a flight log that ever
//  reached the top 1 MB would simply erase old static-fire runs first).
//
//    page 0        AFH1  header: the run, the clamp profile that was used, the
//                        times of its steps, the stand's calibration
//    pages 1..128  the stand's own pages, BYTE FOR BYTE (SFH1 header, SFP1
//                  data, SFF1 footer, docs/DATA_FORMAT.md of Static_Fire_Stand)
//    pages 129..   AFP1  the MERGED table: every thrust sample of the stand
//                        with Ariadna's voltage, current, servo pulse and
//                        clamp closure at that moment (16 B per row)
//
//  The header is written LAST, so a slot whose header checks out is complete;
//  a power cut while writing leaves a slot that is simply not listed.
//
//  Both halves carry their own CRC32 per page, the same zlib CRC the stand
//  uses, so a dump taken from Ariadna decodes with the stand's Python tools.
// =====================================================================
#ifndef SF_RECORD_H
#define SF_RECORD_H

#include <stdint.h>
#include <string.h>
#include "sf_link_proto.h"

#define AFR_PAGE            256u
#define AFR_SLOTS           8u
#define AFR_SLOT_PAGES      512u                       // 128 kB
#define AFR_SLOT_BYTES      (AFR_SLOT_PAGES * AFR_PAGE)
#define AFR_SECTORS_PER_SLOT (AFR_SLOT_BYTES / 4096u)  // 32
#define AFR_REGION_BASE     0x00F00000u                // top 1 MB of the 16 MB chip
#define AFR_RAW_FIRST       1u
#define AFR_RAW_MAX         128u                       // the stand's pages kept (~45 s at 80 SPS)
#define AFR_ROWS_FIRST      (AFR_RAW_FIRST + AFR_RAW_MAX)
#define AFR_ROWS_PER_PAGE   15u
#define AFR_ROWS_MAX_PAGES  (AFR_SLOT_PAGES - AFR_ROWS_FIRST)

#define AFR_MAGIC_HDR   0x31484641u    // "AFH1"
#define AFR_MAGIC_ROWS  0x31504641u    // "AFP1"

// header flags
#define AFR_H_STAND_OK     0x0001u  // stand header + footer present, every page CRC good, finished cleanly
#define AFR_H_STAND_PARTIAL 0x0002u // pages missing / bad / the stand record was truncated
#define AFR_H_NO_OWN       0x0004u  // Ariadna has no samples of its own for this run
#define AFR_H_PROFILE_DONE 0x0008u  // the clamp profile ran to its end
#define AFR_H_STAND_LOST   0x0010u  // the stand link dropped during the burn (clamp forced open)
#define AFR_H_NOT_FIRED    0x0020u  // Ariadna never commanded the pyro for this run

// merged row: bits of `flags` (top byte of `packed`)
//   bits 0..3  the stand's own flags (pyro commanded, continuity, RBF, ADC saturated)
//   bits 4..6  clamp profile phase (sff_phase_t)
//   bit  7     no Ariadna sample at or before this time (the first rows)
#define AFR_ROW_PHASE_SHIFT 4
#define AFR_ROW_NODATA      0x80u

#pragma pack(push, 1)

/** One of Ariadna's own samples, kept in RAM during the run. */
typedef struct {
    int32_t  t_us;         // microseconds since T0, on the stand's clock
    uint16_t vbat_mv;
    int16_t  current_ma;
    uint16_t servo_us;     // the clamp pulse COMMANDED (hobby servos give no feedback)
    uint16_t closed_pm;    // clamp closure, permille: 0 = open, 1000 = shut
    uint8_t  phase;        // sff_phase_t
    uint8_t  flags;        // bit 0 pyro commanded, bit 1 continuity, bit 2 stand link lost
    uint16_t _pad;
} afr_sample_t;

/** One row of the merged table. */
typedef struct {
    int32_t  t_us;         // the stand's timestamp
    uint32_t packed;       // stand: raw 24 bit load cell count | flags << 24 (see above)
    uint16_t vbat_mv;
    int16_t  current_ma;
    uint16_t servo_us;
    uint16_t closed_pm;
} afr_row_t;

typedef union {
    struct {
        uint32_t magic;            //   0
        uint32_t crc32;            //   4  of the page with this field zeroed
        uint32_t run_id;           //   8  Ariadna's own run counter, 1.. across slots
        uint32_t stand_burn_id;    //  12
        uint16_t stand_pages;      //  16  pages of the stand's record that were kept
        uint16_t stand_pages_total;//  18  ... and how many it had
        uint16_t rows_first;       //  20
        uint16_t rows_pages;       //  22
        uint32_t n_rows;           //  24
        uint32_t flags;            //  28  AFR_H_*
        float    cal_counts_per_n; //  32  from the stand's header
        int32_t  tare_offset;      //  36
        int32_t  t_fire_us;        //  40  pyro 1 commanded, SFL_T_NONE if never
        int32_t  t_trigger_us;     //  44  thrust crossed the trigger, SFL_T_NONE if never
        int32_t  t_phase_us[4];    //  48  clamp step 1, 2, 3, full began
        uint16_t ign_ms;           //  64
        uint16_t step_ms;          //  66
        uint16_t trigger_n_x10;    //  68  trigger thrust, 0.1 N
        uint16_t us_closed;        //  70
        uint16_t us_open;          //  72
        uint16_t open_pct_x10[4];  //  74  the four clamp positions, 0.1 % open
        uint16_t n_own;            //  82  Ariadna's own samples that were logged
        uint32_t reserved[4];      //  84
    } f;
    uint8_t raw[AFR_PAGE];
} afr_header_t;

typedef union {
    struct {
        uint32_t magic;            //  0
        uint32_t crc32;            //  4
        uint32_t run_id;           //  8
        uint16_t index;            // 12  page index within the rows area
        uint16_t n_rows;           // 14
        afr_row_t rows[AFR_ROWS_PER_PAGE];   // 16 + 15 * 16 = 256
    } f;
    uint8_t raw[AFR_PAGE];
} afr_rowpage_t;

#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(afr_sample_t) == 16, "afr_sample_t");
static_assert(sizeof(afr_row_t) == 16, "afr_row_t");
static_assert(sizeof(afr_header_t) == AFR_PAGE, "afr_header_t");
static_assert(sizeof(afr_rowpage_t) == AFR_PAGE, "afr_rowpage_t");
static_assert(sizeof(((afr_header_t *)0)->f) <= AFR_PAGE, "header body too big");
#endif

// =====================================================================
//  CRC32, the zlib / IEEE one the stand uses
// =====================================================================
static inline uint32_t afr_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (uint8_t k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

/** CRC of a 256 B page whose crc field sits at `crc_off`. */
static inline uint32_t afr_page_crc(const uint8_t *page, uint32_t crc_off)
{
    uint8_t tmp[AFR_PAGE];
    memcpy(tmp, page, AFR_PAGE);
    memset(tmp + crc_off, 0, 4);
    return afr_crc32(tmp, AFR_PAGE);
}

// =====================================================================
//  The stand's pages
// =====================================================================
#define SFP_MAGIC_HEADER  0x31484653u   // "SFH1"
#define SFP_MAGIC_DATA    0x31504653u   // "SFP1"
#define SFP_MAGIC_FOOTER  0x31464653u   // "SFF1"
#define SFP_SAMPLES_PER_PAGE 29u

typedef enum { AFR_PG_BAD = 0, AFR_PG_HEADER = 1, AFR_PG_DATA = 2, AFR_PG_FOOTER = 3 } afr_pgtype_t;

/** Classify one page of the stand's record and check its CRC. */
static inline int afr_stand_page_type(const uint8_t *p)
{
    const uint32_t magic = sfl_get32(p);
    uint32_t crc_off;
    int type;
    if      (magic == SFP_MAGIC_HEADER) { crc_off = 8;  type = AFR_PG_HEADER; }
    else if (magic == SFP_MAGIC_DATA)   { crc_off = 16; type = AFR_PG_DATA; }
    else if (magic == SFP_MAGIC_FOOTER) { crc_off = 8;  type = AFR_PG_FOOTER; }
    else return AFR_PG_BAD;
    return afr_page_crc(p, crc_off) == sfl_get32(p + crc_off) ? type : (int)AFR_PG_BAD;
}

typedef struct {
    int      has_header, has_footer, clean;
    uint32_t burn_id;
    float    cal_counts_per_n;
    int32_t  tare_offset;
    uint32_t ignition_ms, preroll_ms, recording_ms;
    uint32_t n_samples;
    uint32_t bad_pages;
    int32_t  t_pyro_off_us;
} afr_stand_t;

/** Summary of the stand's header and footer pages. */
static inline void afr_stand_scan(const uint8_t *pages, uint16_t n_pages, afr_stand_t *s)
{
    memset(s, 0, sizeof(*s));
    for (uint16_t i = 0; i < n_pages; i++) {
        const uint8_t *p = pages + (size_t)i * AFR_PAGE;
        const int t = afr_stand_page_type(p);
        if (t == AFR_PG_BAD) { s->bad_pages++; continue; }
        if (t == AFR_PG_HEADER) {
            s->has_header = 1;
            s->burn_id = sfl_get32(p + 4);
            uint32_t cal = sfl_get32(p + 20);
            memcpy(&s->cal_counts_per_n, &cal, 4);
            s->tare_offset = (int32_t)sfl_get32(p + 24);
            s->preroll_ms = sfl_get32(p + 28);
            s->recording_ms = sfl_get32(p + 32);
            s->ignition_ms = sfl_get32(p + 36);
        } else if (t == AFR_PG_FOOTER) {
            s->has_footer = 1;
            s->n_samples = sfl_get32(p + 16);
            s->t_pyro_off_us = (int32_t)sfl_get32(p + 28);
            s->clean = sfl_get32(p + 36) != 0;
        }
    }
}

// =====================================================================
//  Merge: every thrust sample of the stand, with Ariadna's data beside it
// =====================================================================
typedef void (*afr_emit_fn)(void *ctx, const afr_row_t *row);

/**
 * Walks the stand's data pages in order and emits one row per thrust sample,
 * carrying the latest of Ariadna's own samples at or before that time (zero-
 * order hold: the clamp command and the servo pulse are steps, and a held
 * value is the honest way to show one). Rows before Ariadna's first sample
 * get that first sample's values and the NODATA flag.
 *
 * `own` must be sorted by t_us. Returns the number of rows emitted; pages that
 * fail their CRC are skipped, which is the same thing the Python decoder does.
 */
static inline uint32_t afr_merge(const uint8_t *pages, uint16_t n_pages,
                                 const afr_sample_t *own, uint32_t n_own,
                                 afr_emit_fn emit, void *ctx)
{
    uint32_t rows = 0, j = 0;
    for (uint16_t i = 0; i < n_pages; i++) {
        const uint8_t *p = pages + (size_t)i * AFR_PAGE;
        if (afr_stand_page_type(p) != AFR_PG_DATA) continue;
        uint16_t n = (uint16_t)(p[10] | (p[11] << 8));
        if (n > SFP_SAMPLES_PER_PAGE) n = SFP_SAMPLES_PER_PAGE;
        for (uint16_t k = 0; k < n; k++) {
            const uint8_t *s = p + 20 + k * 8;
            afr_row_t row;
            memset(&row, 0, sizeof(row));
            row.t_us = (int32_t)sfl_get32(s);
            const uint32_t packed = sfl_get32(s + 4);
            uint8_t flags = (uint8_t)((packed >> 24) & 0x0Fu);

            if (n_own) {
                while (j + 1 < n_own && own[j + 1].t_us <= row.t_us) j++;
                const afr_sample_t *o = &own[j];
                row.vbat_mv    = o->vbat_mv;
                row.current_ma = o->current_ma;
                row.servo_us   = o->servo_us;
                row.closed_pm  = o->closed_pm;
                flags |= (uint8_t)((o->phase & 7u) << AFR_ROW_PHASE_SHIFT);
                if (row.t_us < own[0].t_us) flags |= AFR_ROW_NODATA;
            } else {
                flags |= AFR_ROW_NODATA;
            }
            row.packed = (packed & 0x00FFFFFFu) | ((uint32_t)flags << 24);
            emit(ctx, &row);
            rows++;
        }
    }
    return rows;
}

// =====================================================================
//  Pages of the slot
// =====================================================================
static inline void afr_rowpage_build(afr_rowpage_t *pg, uint32_t run_id, uint16_t index,
                                     const afr_row_t *rows, uint16_t n)
{
    memset(pg, 0xFF, sizeof(*pg));            // erased flash reads 0xFF; stay close to it
    pg->f.magic  = AFR_MAGIC_ROWS;
    pg->f.crc32  = 0;
    pg->f.run_id = run_id;
    pg->f.index  = index;
    pg->f.n_rows = n;
    if (n) memcpy(pg->f.rows, rows, (size_t)n * sizeof(afr_row_t));
    pg->f.crc32 = afr_crc32(pg->raw, AFR_PAGE);
}

static inline int afr_rowpage_ok(const afr_rowpage_t *pg)
{
    if (pg->f.magic != AFR_MAGIC_ROWS || pg->f.n_rows > AFR_ROWS_PER_PAGE) return 0;
    return afr_page_crc(pg->raw, 4) == pg->f.crc32;
}

static inline void afr_header_seal(afr_header_t *h)
{
    h->f.magic = AFR_MAGIC_HDR;
    h->f.crc32 = 0;
    h->f.crc32 = afr_crc32(h->raw, AFR_PAGE);
}

static inline int afr_header_ok(const afr_header_t *h)
{
    if (h->f.magic != AFR_MAGIC_HDR) return 0;
    return afr_page_crc(h->raw, 4) == h->f.crc32;
}

static inline uint32_t afr_slot_addr(uint32_t slot)
{
    return AFR_REGION_BASE + (slot % AFR_SLOTS) * AFR_SLOT_BYTES;
}

#endif // SF_RECORD_H
