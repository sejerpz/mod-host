/* Self-check for the per-plugin CPU accounting used by effects_cpu_load_all().
 *
 * Mirrors the real thing: each worker resolves its own thread CPU clock the way
 * JackThreadInit() does, and the sampler reads those clocks from another thread
 * and turns successive readings into a percentage of one core.
 *
 * Build & run:  make -f Makefile cpuload-run
 */

#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define SLOT_NS     10000000LL      /* 10ms duty-cycle slot */
#define SAMPLE_S    2               /* measurement window */
#define TOLERANCE   8.0             /* percentage points */

typedef struct {
    double duty;                    /* fraction of wall time to burn */
    clockid_t cpu_clockid;          /* 0 until the thread resolves it */
    uint64_t cpu_ns_prev;
    volatile bool stop;
} worker_t;

static uint64_t clock_ns(clockid_t clk)
{
    struct timespec ts;

    if (clock_gettime(clk, &ts) != 0)
        return 0;

    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void *worker_main(void *arg)
{
    worker_t *w = arg;

    /* same as JackThreadInit() */
    if (pthread_getcpuclockid(pthread_self(), &w->cpu_clockid) != 0)
        w->cpu_clockid = 0;

    const long busy_ns = (long)(SLOT_NS * w->duty);

    while (!w->stop)
    {
        const uint64_t deadline = clock_ns(CLOCK_MONOTONIC) + busy_ns;
        while (clock_ns(CLOCK_MONOTONIC) < deadline && !w->stop) { }

        struct timespec idle = { 0, SLOT_NS - busy_ns };
        if (idle.tv_nsec > 0)
            nanosleep(&idle, NULL);
    }

    return NULL;
}

/* the arithmetic under test, lifted from effects_cpu_load_all() */
static int sample(worker_t *ws, int n, double *out)
{
    static uint64_t s_wall_ns_prev = 0;

    const uint64_t wall_ns = clock_ns(CLOCK_MONOTONIC);
    const uint64_t wall_delta = (wall_ns > s_wall_ns_prev) ? wall_ns - s_wall_ns_prev : 0;
    const uint64_t wall_prev = s_wall_ns_prev;
    s_wall_ns_prev = wall_ns;

    int reported = 0;

    for (int i = 0; i < n; i++)
    {
        if (ws[i].cpu_clockid == 0)
            continue;

        const uint64_t cpu_ns = clock_ns(ws[i].cpu_clockid);
        const uint64_t cpu_prev = ws[i].cpu_ns_prev;
        ws[i].cpu_ns_prev = cpu_ns;

        if (cpu_prev == 0 || wall_prev == 0 || wall_delta == 0)
            continue;

        out[i] = (double)(cpu_ns - cpu_prev) * 100.0 / (double)wall_delta;
        reported++;
    }

    return reported;
}

int main(void)
{
    worker_t ws[] = {
        { 0.50, 0, 0, false },
        { 0.25, 0, 0, false },
        { 0.00, 0, 0, false },
    };
    const int n = sizeof(ws) / sizeof(ws[0]);
    pthread_t tids[3];
    double pct[3] = { 0, 0, 0 };

    for (int i = 0; i < n; i++)
        assert(pthread_create(&tids[i], NULL, worker_main, &ws[i]) == 0);

    sleep(1); /* let every thread resolve its clock and settle */

    /* first sample establishes the baseline and must report nothing */
    assert(sample(ws, n, pct) == 0);

    sleep(SAMPLE_S);

    assert(sample(ws, n, pct) == n);

    for (int i = 0; i < n; i++)
        ws[i].stop = true;
    for (int i = 0; i < n; i++)
        pthread_join(tids[i], NULL);

    int failed = 0;
    for (int i = 0; i < n; i++)
    {
        const double want = ws[i].duty * 100.0;
        const double diff = pct[i] > want ? pct[i] - want : want - pct[i];
        const bool ok = diff <= TOLERANCE;

        printf("%s duty %5.1f%% -> measured %5.1f%%  (off by %4.1f)\n",
               ok ? "ok  " : "FAIL", want, pct[i], diff);

        if (!ok)
            failed++;
    }

    /* a busier thread must always outrank a quieter one -- this is what the
       ranked list in the UI is built on */
    assert(pct[0] > pct[1] && pct[1] > pct[2]);

    if (failed)
    {
        fprintf(stderr, "%d/%d outside +/-%.0f points\n", failed, n, TOLERANCE);
        return 1;
    }

    puts("all ok");
    return 0;
}
