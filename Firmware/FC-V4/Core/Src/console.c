/* ============================================================================
 * console.c  —  see console.h
 * ============================================================================ */
#include "console.h"
#include "usbio.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define LINE_MAX 96u

static const ConCmd *s_cmds;
static uint32_t      s_ncmds;
static char          s_line[LINE_MAX];
static uint32_t      s_len;

void Con_Init(const ConCmd *cmds, uint32_t n) { s_cmds = cmds; s_ncmds = n; s_len = 0u; }

void Con_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) { (void)UsbIo_Write(buf, (n < (int)sizeof buf) ? (uint32_t)n : (uint32_t)sizeof buf - 1u); }
}

const char *Con_Fx(char out[CON_FX_LEN], float v, uint32_t dp)
{
    static const uint32_t p10[5] = { 1u, 10u, 100u, 1000u, 10000u };
    if (dp > 4u) { dp = 4u; }
    if (!isfinite(v)) { (void)snprintf(out, CON_FX_LEN, "nan"); return out; }
    const float scaled = v * (float)p10[dp];
    if (fabsf(scaled) > 2.0e9f) {                 /* range check before the cast */
        (void)snprintf(out, CON_FX_LEN, "%covf", (v < 0.0f) ? '-' : '+');
        return out;
    }
    const int32_t  m = (int32_t)lroundf(scaled);
    const uint32_t a = (uint32_t)((m < 0) ? -m : m);
    const char     s = (m < 0) ? '-' : '+';
    if (dp == 0u) { (void)snprintf(out, CON_FX_LEN, "%c%lu", s, (unsigned long)a); }
    else {
        (void)snprintf(out, CON_FX_LEN, "%c%lu.%0*lu", s, (unsigned long)(a / p10[dp]),
                       (int)dp, (unsigned long)(a % p10[dp]));
    }
    return out;
}

void Con_Help(void)
{
    Con_Printf("commands:\r\n");
    for (uint32_t i = 0u; i < s_ncmds; i++) {
        Con_Printf("  %-8s %s\r\n", s_cmds[i].name, s_cmds[i].usage);
    }
}

static void dispatch(char *line)
{
    char *argv[CON_ARGS_MAX];
    uint32_t argc = 0u;
    char *p = line;
    while ((*p != '\0') && (argc < CON_ARGS_MAX)) {
        while (*p == ' ') { p++; }
        if (*p == '\0') { break; }
        argv[argc++] = p;
        while ((*p != ' ') && (*p != '\0')) { p++; }
        if (*p == ' ') { *p++ = '\0'; }
    }
    if (argc == 0u) { return; }
    for (uint32_t i = 0u; i < s_ncmds; i++) {
        if (strcmp(argv[0], s_cmds[i].name) == 0) { s_cmds[i].fn(argc, argv); return; }
    }
    Con_Printf("ERR unknown command '%s' (help)\r\n", argv[0]);
}

void Con_Poll(void)
{
    uint8_t buf[64];
    Con_Feed(buf, UsbIo_Read(buf, sizeof buf));
}

void Con_Feed(const uint8_t *buf, uint32_t n)
{
    for (uint32_t i = 0u; i < n; i++) {
        const char c = (char)buf[i];
        if ((c == '\r') || (c == '\n')) {
            if (s_len == 0u) { continue; }
            s_line[s_len] = '\0';
            s_len = 0u;
            Con_Printf("\r\n");
            dispatch(s_line);
        } else if ((c == '\b') || (c == 0x7F)) {
            if (s_len > 0u) { s_len--; Con_Printf("\b \b"); }
        } else if ((c >= ' ') && (s_len < (LINE_MAX - 1u))) {
            s_line[s_len++] = c;
            (void)UsbIo_Write(&c, 1u);                /* echo */
        } else {
            /* control character or over-long line: ignored */
        }
    }
}
