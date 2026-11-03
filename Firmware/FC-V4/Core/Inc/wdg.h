/* ============================================================================
 * wdg.h  —  Independent watchdog (IWDG), register-level
 *
 * Normal timeout ~100 ms (10 frames) at the nominal 32 kHz LSI (68-188 ms over
 * the LSI spread). Kicked only by the scheduler after a frame in which every
 * mandatory task checked in (app.c). Frozen while the debugger halts the core.
 * Wdg_Stretch() lengthens the timeout to ~32 s around flash erase (a 128 KB
 * sector erase takes up to 2 s); Wdg_Restore() returns to normal. Once started
 * the IWDG cannot be stopped (by design).
 * ============================================================================ */
#ifndef WDG_H_
#define WDG_H_

void Wdg_Start(void);
void Wdg_Kick(void);
void Wdg_Stretch(void);
void Wdg_Restore(void);

#endif /* WDG_H_ */
