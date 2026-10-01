// =====================================================================
//  Ariadna V2 — DMA-fed UART transmit
//
//  Binds the transmit ring in ariadna_link.h to an RP2350 DMA channel, so
//  queuing a frame costs a memcpy and nothing else. The processor never
//  waits for the wire.
//
//  ---------------------------------------------------------------------
//  WHY
//  ---------------------------------------------------------------------
//  The UART's hardware transmit FIFO is 32 bytes and a byte takes 21.7 us
//  at 460800 baud. A 90-byte nav_state frame therefore cannot be handed
//  over in one go — 58 bytes have to be fed as the FIFO drains, which is
//  1.26 ms. Repeated at 100 Hz that is 12.6 % of a core, and on the Master
//  it lands inside the packet-forwarding path, which is the one place in
//  the vehicle where latency was deliberately minimised.
//
//  With DMA the same frame costs a 90-byte copy into the ring — under
//  2 us — and the transfer happens in the background.
//
//  ---------------------------------------------------------------------
//  WHERE DMA IS *NOT* USED, AND WHY
//  ---------------------------------------------------------------------
//  DMA is not free: every channel is another thing to configure wrongly
//  and another buffer whose lifetime has to be reasoned about. It is worth
//  it here because the cost being removed is large and it sits in the
//  control path. Everywhere else in this project it is not:
//
//    flash logging   a 256-byte page at 20 MHz is 33 us of SPI, about 17
//                    times a second — 0.06 % of a core, on core 1, which
//                    is otherwise idle. Nothing to win.
//    LoRa            46 bytes, 4 times a second. The 92 ms the packet
//                    spends on air dwarfs the microseconds of SPI.
//    ADC             three blocking conversions per tick is ~9 us at
//                    200 Hz. The IIR filter already averages ~50 samples,
//                    so free-running DMA would not even improve the noise.
//    UART receive    a receive FIFO does not block the processor: unread
//                    bytes simply wait. The parser costs ~1 % of a core
//                    at full traffic, which is not worth a channel.
//    CRC / parsing   about 1 % of a core in total. Measured, not assumed —
//                    see the budget printed by ../../test/link_test.cpp.
//
//  ---------------------------------------------------------------------
//  STATUS
//  ---------------------------------------------------------------------
//  The ring bookkeeping is host-tested (../../test/link_test.cpp, section
//  9): byte-exact ordering across repeated wraps, whole-frame drops when
//  full, and proof that the in-flight region cannot be overwritten.
//
//  THE DMA CONFIGURATION BELOW IS NOT TESTED. It cannot be, without the
//  chip. Set ARIADNA_USE_DMA_TX to 0 to fall back to blocking writes if
//  anything here misbehaves on the bench — that is a one-line change and
//  the rest of the firmware is unaffected.
//
//  Single producer, single consumer. Each ring must be serviced from ONE
//  core only. In this firmware every ring belongs to the core that fills
//  it, so there is nothing to synchronise.
// =====================================================================
#ifndef ARIADNA_UART_DMA_H
#define ARIADNA_UART_DMA_H

#include "ariadna_link.h"

#include <Arduino.h>
#include "hardware/dma.h"
#include "hardware/uart.h"

#ifndef ARIADNA_USE_DMA_TX
#define ARIADNA_USE_DMA_TX 1
#endif

/**
 * One DMA-fed transmit port.
 *
 * `port` is the Arduino stream, used for begin() and for receive — the
 * receive path is untouched, because a receive FIFO does not block the
 * processor. Only transmit is taken over.
 */
typedef struct {
    ariadna_tx_ring_t ring;
    uart_inst_t      *uart;
    int               ch;        // DMA channel, -1 if unavailable
    bool              ready;
} ariadna_uart_tx_t;

/**
 * Claim a DMA channel and point it at the UART's data register.
 *
 * Call AFTER the port's begin(), which is what configures the UART itself.
 *
 * `inst` is the SDK handle for the same peripheral the Arduino stream
 * uses: Serial1 is uart0 and Serial2 is uart1 on the arduino-pico core.
 * Passing the wrong one sends every frame out of the wrong pins, which on
 * the Master means the Navigation MCU receives the Position MCU's traffic.
 */
static inline bool ariadna_uart_tx_begin(ariadna_uart_tx_t *u, uart_inst_t *inst)
{
    ariadna_tx_init(&u->ring);
    u->uart  = inst;
    u->ch    = -1;
    u->ready = false;

#if ARIADNA_USE_DMA_TX
    // Do not abort on failure: a board that cannot get a channel should
    // still fly, just with blocking writes. `required = false` returns -1
    // instead of panicking.
    u->ch = dma_claim_unused_channel(false);
    if (u->ch < 0) return false;

    dma_channel_config c = dma_channel_get_default_config(u->ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);    // walk the ring
    channel_config_set_write_increment(&c, false);  // one fixed register
    // Pace the transfer against the UART's transmit request, so DMA writes
    // a byte only when the FIFO has room. Without this the channel would
    // dump the whole buffer into a 32-byte FIFO and lose most of it.
    channel_config_set_dreq(&c, uart_get_dreq(inst, true));

    dma_channel_configure(u->ch, &c,
                          &uart_get_hw(inst)->dr,   // destination
                          NULL, 0, false);          // armed by _service()

    u->ready = true;
    return true;
#else
    return false;
#endif
}

/**
 * Advance the transfer. Cheap, and safe to call as often as you like.
 *
 * Retires a finished run and starts the next. Call it once per control
 * tick and after each send; there is no interrupt handler because there is
 * nothing urgent to do — a few hundred microseconds of idle line between
 * runs is invisible next to the frame period.
 */
static inline void ariadna_uart_tx_service(ariadna_uart_tx_t *u)
{
#if ARIADNA_USE_DMA_TX
    if (!u->ready) return;
    if (dma_channel_is_busy(u->ch)) return;

    if (u->ring.inflight) ariadna_tx_complete(&u->ring);

    const uint8_t *p = NULL;
    uint16_t n = ariadna_tx_next(&u->ring, &p);
    if (!n) return;

    // The bytes were written by this core moments ago. The M33 has no data
    // cache, so a compiler barrier is all that is needed to stop the copy
    // being reordered after the trigger.
    __atomic_thread_fence(__ATOMIC_RELEASE);
    dma_channel_transfer_from_buffer_now(u->ch, p, n);
#else
    (void)u;
#endif
}

/**
 * Encode and queue one frame. Never blocks.
 *
 * Returns false if the ring was full, which is counted in
 * u->ring.dropped. A dropped frame is preferable to a stalled control
 * loop: the next one is 5 or 10 ms away, and every packet in this system
 * carries absolute state rather than a delta, so nothing accumulates.
 *
 * `port` is used only when DMA is disabled or unavailable.
 */
template <typename PortT>
static inline bool ariadna_uart_tx_send(ariadna_uart_tx_t *u, PortT &port,
                                        uint8_t type, const void *payload,
                                        uint8_t len)
{
    uint8_t frame[ARIADNA_FRAME_MAX];
    uint16_t n = ariadna_encode(frame, type, payload, len);
    if (!n) return false;

    if (!u->ready) {
        // Fallback: a blocking write, which is what the whole file exists
        // to avoid — but it is correct, and correct beats fast.
        port.write(frame, n);
        return true;
    }

    bool ok = ariadna_tx_push(&u->ring, frame, n) != 0;
    ariadna_uart_tx_service(u);
    return ok;
}

#endif // ARIADNA_UART_DMA_H
