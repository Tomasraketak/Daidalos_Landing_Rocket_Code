// =====================================================================
//  Ariadna V2 — RGB and buzzer annunciators
//
//  One small, non-blocking pattern engine, shared by all three flight
//  boards and by the ground station, so that a colour means the same
//  thing everywhere in the system.
//
//  WHY THIS EXISTS AT ALL
//  ---------------------------------------------------------------------
//  On the pad there is no laptop. There is a vehicle on a rail, a crew
//  behind a line, and a few seconds in which somebody has to decide
//  whether to arm it. A LED that reliably says "the navigation filter has
//  a pad reference, four UWB anchors and a GPS fix" is worth more at that
//  moment than a telemetry stream nobody is looking at, and a buzzer that
//  says "I am about to fire a pyro" is worth more than both.
//
//  Every board therefore drives an indicator:
//
//    Master MCU       SK6812 on GP2 (PIO), buzzer on GP3 (PWM)
//    Position MCU     the XIAO RP2350's own WS2812
//    Navigation MCU   the XIAO RP2350's own WS2812
//
//  ---------------------------------------------------------------------
//  THE ONE RULE: THIS MUST NEVER COST A CONTROL CYCLE
//  ---------------------------------------------------------------------
//  Driving a WS2812 means bit-banging 24 bits at 800 kHz with interrupts
//  off — about 30 microseconds during which nothing else on that core
//  happens. Thirty microseconds is nothing at 20 Hz and it is 0.6 % of a
//  200 Hz control tick, every tick, if you are careless about it.
//
//  Two properties of this engine make it safe:
//
//    * service() is a state machine. It never delays, never spins, and
//      never blocks on a bus.
//    * show() is called ONLY when the computed pixel value differs from
//      the one already on the wire. A solid colour therefore costs one
//      transfer and then nothing at all, and a slow blink costs two
//      transfers a second.
//
//  Call service() at the END of a loop, after the work that matters.
//
//  ---------------------------------------------------------------------
//  BUILDING WITHOUT THE LIBRARY
//  ---------------------------------------------------------------------
//  If Adafruit_NeoPixel is not installed the whole RGB path compiles to
//  nothing and every call is a no-op. That is deliberate: `make syntax`
//  compiles the sketches against a stub Arduino header with no libraries
//  at all, and an annunciator is not worth breaking that over.
//
//  The probe below CANNOT find the library by itself in an Arduino build:
//  the builder only adds a library's include path after it has seen a
//  real #include of it. Every sketch therefore also includes
//  <Adafruit_NeoPixel.h> itself, before this header.
// =====================================================================
#ifndef ARIADNA_RGB_H
#define ARIADNA_RGB_H

#include <stdint.h>

#if defined(__has_include)
#  if __has_include(<Adafruit_NeoPixel.h>)
#    include <Adafruit_NeoPixel.h>
#    define ARIADNA_HAVE_NEOPIXEL 1
#  endif
#endif
#ifndef ARIADNA_HAVE_NEOPIXEL
#  define ARIADNA_HAVE_NEOPIXEL 0
#endif

// ---------------------------------------------------------------------
//  Where the on-board LED is
//
//  The arduino-pico variant for the XIAO RP2350 defines PIN_NEOPIXEL and
//  NEOPIXEL_POWER. Both are used through these macros so that a board
//  variant that names them differently needs one edit here rather than
//  three edits across the sketches. The fallbacks are the XIAO RP2350's
//  documented GPIOs and are only reached if the core does not define
//  them — if the LED stays dark on the bench, this is the first place to
//  look.
// ---------------------------------------------------------------------
#ifndef ARIADNA_RGB_PIN
#  ifdef PIN_NEOPIXEL
#    define ARIADNA_RGB_PIN        PIN_NEOPIXEL
#  else
#    define ARIADNA_RGB_PIN        22
#  endif
#endif
#ifndef ARIADNA_RGB_POWER_PIN
#  ifdef NEOPIXEL_POWER
#    define ARIADNA_RGB_POWER_PIN  NEOPIXEL_POWER
#  else
#    define ARIADNA_RGB_POWER_PIN  23
#  endif
#endif

/** A colour, 8 bits per channel, before brightness scaling. */
typedef struct { uint8_t r, g, b; } ariadna_rgb_c;

static inline ariadna_rgb_c ariadna_rgb_mk(uint8_t r, uint8_t g, uint8_t b)
{
    ariadna_rgb_c c; c.r = r; c.g = g; c.b = b; return c;
}

// The vehicle's palette. Named rather than written out at each use so
// that "armed is red" is a fact stated once, in one place, for three
// boards. A function rather than a compound literal because a compound
// literal is a C construct that C++ only tolerates as an extension, and
// this header is included from .ino files, from the host syntax check
// and, in principle, from C.
#define ARIADNA_C_OFF      ariadna_rgb_mk(  0,   0,   0)
#define ARIADNA_C_RED      ariadna_rgb_mk(255,   0,   0)
#define ARIADNA_C_GREEN    ariadna_rgb_mk(  0, 255,   0)
#define ARIADNA_C_BLUE     ariadna_rgb_mk(  0,   0, 255)
#define ARIADNA_C_AMBER    ariadna_rgb_mk(255, 120,   0)
#define ARIADNA_C_YELLOW   ariadna_rgb_mk(220, 220,   0)
#define ARIADNA_C_MAGENTA  ariadna_rgb_mk(255,   0, 200)
#define ARIADNA_C_CYAN     ariadna_rgb_mk(  0, 220, 220)
#define ARIADNA_C_WHITE    ariadna_rgb_mk(255, 255, 255)

/** How the colour is presented over time. */
typedef enum {
    ARIADNA_PAT_OFF = 0,
    ARIADNA_PAT_SOLID,       // on, continuously
    ARIADNA_PAT_SLOW,        // 1 Hz, half duty  — a healthy, waiting board
    ARIADNA_PAT_FAST,        // 5 Hz, half duty  — attention
    ARIADNA_PAT_PULSE,       // triangular breathe — alive but idle
    ARIADNA_PAT_BLINK_N      // n short flashes, then a pause: a fault CODE
} ariadna_rgb_pattern_t;

/**
 * One addressable pixel with a pattern on it.
 *
 * Deliberately not a class with virtuals: it is instantiated once per
 * board and its whole job is to be cheap.
 */
typedef struct {
    uint8_t  pin;
    uint8_t  power_pin;      // 0xFF = none
    uint8_t  brightness;     // 0..255, applied multiplicatively
    uint8_t  pattern;
    uint8_t  count;          // for BLINK_N
    ariadna_rgb_c colour;
    uint32_t phase_ms;       // millis() at which the current cycle started
    uint32_t last_show_ms;
    uint8_t  last_r, last_g, last_b;
    uint8_t  ready;
#if ARIADNA_HAVE_NEOPIXEL
    Adafruit_NeoPixel *px;
#endif
} ariadna_rgb_t;

#if ARIADNA_HAVE_NEOPIXEL
/**
 * Storage for the driver. One static instance per board; putting it in
 * the header rather than in each sketch keeps the pin choice and the
 * object that uses it in the same place.
 */
static Adafruit_NeoPixel ariadna_rgb_pixel(1, ARIADNA_RGB_PIN, NEO_GRB + NEO_KHZ800);
#endif

/**
 * Bring the LED up.
 *
 * `power_pin` is the XIAO's LED power enable. It is a real pin on the
 * RP2350 boards and driving it is what makes the difference between a
 * working indicator and a mystery.
 */
static inline void ariadna_rgb_begin(ariadna_rgb_t *a, uint8_t brightness,
                                     uint8_t power_pin)
{
    a->pin        = (uint8_t)ARIADNA_RGB_PIN;
    a->power_pin  = power_pin;
    a->brightness = brightness;
    a->pattern    = ARIADNA_PAT_OFF;
    a->count      = 0;
    a->colour     = ARIADNA_C_OFF;
    a->phase_ms   = 0;
    a->last_show_ms = 0;
    a->last_r = a->last_g = a->last_b = 1;   // != 0, so the first OFF is drawn
    a->ready  = 0;

    if (power_pin != 0xFF) {
        pinMode(power_pin, OUTPUT);
        digitalWrite(power_pin, HIGH);
    }
#if ARIADNA_HAVE_NEOPIXEL
    a->px = &ariadna_rgb_pixel;
    a->px->begin();
    a->px->setBrightness(255);      // scaling is done here, in integers
    a->px->clear();
    a->px->show();
    a->ready = 1;
#else
    (void)a->pin;
#endif
}

/** Choose what the LED is saying. Cheap: no I/O happens here. */
static inline void ariadna_rgb_set(ariadna_rgb_t *a, ariadna_rgb_c c,
                                   uint8_t pattern, uint8_t count)
{
    if (a->pattern == pattern && a->count == count &&
        a->colour.r == c.r && a->colour.g == c.g && a->colour.b == c.b)
        return;                      // same message: do not restart the phase
    a->colour  = c;
    a->pattern = pattern;
    a->count   = count;
    a->phase_ms = millis();
}

static inline void ariadna_rgb_brightness(ariadna_rgb_t *a, uint8_t b)
{
    a->brightness = b;
}

/**
 * Advance the pattern and, only if the pixel actually changed, push it.
 *
 * Called from the end of a loop at whatever rate that loop runs. It is
 * safe at 200 Hz because of the change test — see the header comment.
 */
static inline void ariadna_rgb_service(ariadna_rgb_t *a)
{
    const uint32_t now = millis();
    const uint32_t t   = now - a->phase_ms;
    uint8_t scale = 0;               // 0..255 of the chosen colour

    switch (a->pattern) {
    case ARIADNA_PAT_OFF:
        scale = 0;
        break;
    case ARIADNA_PAT_SOLID:
        scale = 255;
        break;
    case ARIADNA_PAT_SLOW:
        scale = ((t % 1000u) < 500u) ? 255 : 0;
        break;
    case ARIADNA_PAT_FAST:
        scale = ((t % 200u) < 100u) ? 255 : 0;
        break;
    case ARIADNA_PAT_PULSE: {
        // Triangular rather than sinusoidal: a sine costs a transcendental
        // call and the eye cannot tell the difference at 2 s a cycle.
        const uint32_t u = t % 2000u;
        scale = (uint8_t)((u < 1000u) ? (u * 255u / 1000u)
                                      : ((2000u - u) * 255u / 1000u));
        break;
    }
    case ARIADNA_PAT_BLINK_N: {
        // n flashes of 120 ms in a 1.6 s frame. Countable from behind the
        // safety line, which is the entire point of encoding a fault as a
        // number of flashes rather than as a shade of orange.
        const uint32_t frame = 1600u;
        const uint32_t u = t % frame;
        const uint32_t n = a->count ? a->count : 1;
        scale = (u < n * 240u && (u % 240u) < 120u) ? 255 : 0;
        break;
    }
    default:
        scale = 0;
        break;
    }

    // Two integer scalings, colour then brightness. No floating point:
    // this runs on the control core.
    const uint16_t s = (uint16_t)scale * (uint16_t)a->brightness;
    const uint8_t r = (uint8_t)(((uint32_t)a->colour.r * s) / 65025u);
    const uint8_t g = (uint8_t)(((uint32_t)a->colour.g * s) / 65025u);
    const uint8_t b = (uint8_t)(((uint32_t)a->colour.b * s) / 65025u);

    if (r == a->last_r && g == a->last_g && b == a->last_b) return;
    a->last_r = r; a->last_g = g; a->last_b = b;
    a->last_show_ms = now;

#if ARIADNA_HAVE_NEOPIXEL
    if (!a->ready) return;
    a->px->setPixelColor(0, a->px->Color(r, g, b));
    a->px->show();
#endif
}

// =====================================================================
//  Buzzer
//
//  Same discipline: a queue of (frequency, duration) steps advanced by a
//  service() call, so nothing in the vehicle ever calls delay() to make a
//  noise. The Master owns the only buzzer, but the engine lives here
//  because the ground station has one too.
// =====================================================================
#define ARIADNA_BEEP_MAX  6

typedef struct { uint16_t hz; uint16_t ms; } ariadna_beep_step_t;

typedef struct {
    uint8_t  pin;
    uint8_t  n;              // steps in the current sequence
    uint8_t  i;              // step being played
    uint8_t  repeat;         // 0 = play once, 1 = loop forever
    uint32_t step_start_ms;
    uint32_t last_start_ms;
    ariadna_beep_step_t step[ARIADNA_BEEP_MAX];
    uint8_t  active;
    uint8_t  enabled;
} ariadna_buzzer_t;

static inline void ariadna_buzzer_begin(ariadna_buzzer_t *z, uint8_t pin)
{
    z->pin = pin;
    z->n = z->i = 0;
    z->repeat = 0;
    z->step_start_ms = 0;
    z->last_start_ms = 0;
    z->active  = 0;
    z->enabled = 1;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

/**
 * Master inhibit. Set false in flight if the buzzer's current draw or its
 * PWM slice is wanted elsewhere; every play call then becomes a no-op
 * rather than needing a guard at each site.
 */
static inline void ariadna_buzzer_enable(ariadna_buzzer_t *z, uint8_t on)
{
    z->enabled = on ? 1 : 0;
    if (!on && z->active) {
        noTone(z->pin);
        z->active = 0;
    }
}

static inline void ariadna_buzzer_stop(ariadna_buzzer_t *z)
{
    if (z->active) noTone(z->pin);
    z->active = 0;
    z->n = z->i = 0;
    z->repeat = 0;
}

/**
 * Start a sequence. `min_gap_ms` refuses to restart a sequence that was
 * begun less than that long ago, which is what lets a caller invoke a
 * beep unconditionally from inside a loop without it stuttering.
 */
static inline void ariadna_buzzer_play(ariadna_buzzer_t *z,
                                       const ariadna_beep_step_t *steps,
                                       uint8_t n, uint8_t repeat,
                                       uint32_t min_gap_ms)
{
    if (!z->enabled || n == 0) return;
    const uint32_t now = millis();
    if (z->active && (uint32_t)(now - z->last_start_ms) < min_gap_ms) return;
    if (n > ARIADNA_BEEP_MAX) n = ARIADNA_BEEP_MAX;
    for (uint8_t k = 0; k < n; k++) z->step[k] = steps[k];
    z->n = n;
    z->i = 0;
    z->repeat = repeat;
    z->active = 1;
    z->step_start_ms = now;
    z->last_start_ms = now;
    if (z->step[0].hz) tone(z->pin, z->step[0].hz, z->step[0].ms);
}

/** Advance the sequence. Non-blocking; call it from the loop. */
static inline void ariadna_buzzer_service(ariadna_buzzer_t *z)
{
    if (!z->active) return;
    const uint32_t now = millis();
    if ((uint32_t)(now - z->step_start_ms) < z->step[z->i].ms) return;

    z->i++;
    if (z->i >= z->n) {
        if (!z->repeat) { ariadna_buzzer_stop(z); return; }
        z->i = 0;
    }
    z->step_start_ms = now;
    if (z->step[z->i].hz) tone(z->pin, z->step[z->i].hz, z->step[z->i].ms);
    else                  noTone(z->pin);
}

#endif // ARIADNA_RGB_H
