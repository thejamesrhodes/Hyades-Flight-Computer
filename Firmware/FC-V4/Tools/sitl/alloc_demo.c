/* ALLOC-01 demo: the allocator's fin ticks for a slow roll ramp (mode 0) or a
 * zero command (mode 1). Built by alloc_figures.py with each variant's flags. */
#include "alloc.h"
#include <stdio.h>
#include <stdlib.h>
int main(int argc, char **argv)
{
    const int mode = (argc > 1) ? atoi(argv[1]) : 0;
    Alloc a; Alloc_Init(&a); float out[4];
    for (int k = 0; k < 400; k++) {
        const float u = (mode == 0) ? (0.0040f + (0.0060f * (float)k / 400.0f)) : 0.0f;
        const float c[4] = { u, u, u, u };
        Alloc_Apply(&a, c, true, out);
        printf("%d %.7f %d %d %d %d\n", k, (double)u, (int)a.tick[0], (int)a.tick[1], (int)a.tick[2], (int)a.tick[3]);
    }
    return 0;
}
