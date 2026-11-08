/* logq.c  —  see logq.h */
#include "logq.h"
#include <string.h>

void LogQ_Init(LogQ *q, uint8_t (*buf)[LOGREC_SIZE], uint16_t cap)
{
    q->buf = buf; q->cap = cap;
    LogQ_Reset(q);
}

void LogQ_Reset(LogQ *q)
{
    q->head = 0u; q->count = 0u; q->committed = false; q->dropped = 0u; q->pushed = 0u;
}

void LogQ_Commit(LogQ *q) { q->committed = true; }

bool LogQ_Push(LogQ *q, const uint8_t rec[LOGREC_SIZE])
{
    if ((q->buf == NULL) || (q->cap == 0u)) { q->dropped++; return false; }
    q->pushed++;
    if (q->count == q->cap) {
        if (q->committed) { q->dropped++; return false; }
        q->head = (uint16_t)((q->head + 1u) % q->cap);      /* overwrite the oldest */
        q->count--;
    }
    const uint16_t tail = (uint16_t)((q->head + q->count) % q->cap);
    memcpy(q->buf[tail], rec, LOGREC_SIZE);
    q->count++;
    return true;
}

const uint8_t *LogQ_Peek(const LogQ *q) { return (q->count > 0u) ? q->buf[q->head] : NULL; }

void LogQ_Pop(LogQ *q)
{
    if (q->count == 0u) { return; }
    q->head = (uint16_t)((q->head + 1u) % q->cap);
    q->count--;
}

uint16_t LogQ_Count(const LogQ *q) { return q->count; }
