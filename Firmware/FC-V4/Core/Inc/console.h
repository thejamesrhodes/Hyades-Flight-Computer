/* ============================================================================
 * console.h  —  Line-based text command console over USB (usbio)
 *
 * Commands are "<name> [args...]\r" or "\n". Characters are echoed so Tera
 * Term works without local echo; scripts match reply prefixes and ignore it.
 * Replies and logs use Con_Printf(): non-blocking (dropped if the TX ring is
 * full). No float formatting in printf (not linked): use Con_Fx().
 * ============================================================================ */
#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdint.h>
#include <stdbool.h>

#define CON_ARGS_MAX  6u
#define CON_FX_LEN    24u     /* buffer size for one Con_Fx() result */

typedef void (*ConFn)(uint32_t argc, char *argv[]);
typedef struct { const char *name; const char *usage; ConFn fn; } ConCmd;

/* Register the command table (static storage, lives for the program). */
void        Con_Init(const ConCmd *cmds, uint32_t n);

/* Read received bytes (at most 64 per call) and run completed commands. */
void        Con_Poll(void);

/* printf-style output, integer formats only. */
void        Con_Printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Fixed-point text for a float: sign, integer part, dp (0..4) decimals,
 * e.g. "+12.3". Non-finite -> "nan", out of range -> "+ovf"/"-ovf". */
const char *Con_Fx(char buf[CON_FX_LEN], float v, uint32_t dp);

void        Con_Help(void);

#endif /* CONSOLE_H_ */
