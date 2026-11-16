// =====================================================================
//  Minimal pico-sdk DMA shim — HOST SYNTAX CHECK ONLY.
//
//  This checks that ariadna_uart_dma.h is well-formed C++ and that the
//  ring is driven correctly. It does NOT check the channel configuration,
//  which is the part that can only be verified with a chip in front of
//  you — see the status note in ariadna_uart_dma.h.
// =====================================================================
#ifndef PICO_DMA_STUB_H
#define PICO_DMA_STUB_H

#include <stdint.h>
#include <stddef.h>

enum dma_channel_transfer_size { DMA_SIZE_8 = 0, DMA_SIZE_16 = 1, DMA_SIZE_32 = 2 };

typedef struct { uint32_t ctrl; } dma_channel_config;

inline int  dma_claim_unused_channel(bool) { return 0; }
inline dma_channel_config dma_channel_get_default_config(unsigned) {
    dma_channel_config c = { 0 };
    return c;
}
inline void channel_config_set_transfer_data_size(dma_channel_config *,
                                                  enum dma_channel_transfer_size) {}
inline void channel_config_set_read_increment(dma_channel_config *, bool) {}
inline void channel_config_set_write_increment(dma_channel_config *, bool) {}
inline void channel_config_set_dreq(dma_channel_config *, unsigned) {}
inline void dma_channel_configure(unsigned, const dma_channel_config *,
                                  volatile void *, const volatile void *,
                                  unsigned, bool) {}
inline bool dma_channel_is_busy(unsigned) { return false; }
inline void dma_channel_transfer_from_buffer_now(unsigned, const volatile void *,
                                                 uint32_t) {}

#endif
