/* Trace output on USART2 -> ST-LINK virtual COM port.
 *
 * Transmission is DMA driven from a ring buffer so that tracing never blocks
 * the SSI timing path: trace_printf() only copies into the ring and returns.
 * If the ring is full the message is dropped and counted rather than stalling
 * an interrupt handler.
 */
#ifndef SIMENC_TRACE_H
#define SIMENC_TRACE_H

#include <stdint.h>

void     trace_init(void);
void     trace_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void     trace_write(const char *data, uint32_t len);
uint32_t trace_dropped(void);

/* Blocking drain, for use before a reset or on a fatal path. */
void trace_flush(void);

#endif /* SIMENC_TRACE_H */
