/* Line-based console on the ST-LINK virtual COM port.
 *
 * Exists so loopback bring-up can change the emulator's behaviour without
 * rebuilding -- in particular selecting a synthetic position source to bypass
 * the analog acquisition.
 */
#ifndef SIMENC_CONSOLE_H
#define SIMENC_CONSOLE_H

void console_init(void);
void console_poll(void);
void console_banner(void);

#endif /* SIMENC_CONSOLE_H */
