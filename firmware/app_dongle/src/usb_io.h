/* CDC-ACM host pipe: COBS framing both directions, TX frame ring, host
 * command dispatch.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_USB_IO_H
#define APP_USB_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Binds the CDC UART, registers IRQ callbacks, and calls usb_enable(). */
int usb_io_init(void);

bool usb_io_is_configured(void);

/* Encode one USB payload (type + body) into a frame and queue it for the
 * host. Never blocks: if the TX ring cannot take the whole frame, the frame
 * is dropped and counted. Safe from any thread; not for ISR use (the radio
 * path calls it from its processing thread, never from the ESB IRQ). */
void usb_io_send_payload(const void *payload, size_t len);

/* Frames dropped because the TX ring was full (host slow/absent). */
uint32_t usb_io_tx_dropped(void);

#endif
