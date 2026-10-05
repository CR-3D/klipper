// Host simulation of src/stepper.c + src/buffer_feed.c (the real firmware code)
// against a simulated scheduler, GPIO and a spring buffer model.
// Build and run with scripts/buffer_feed_sim/run.sh (set DBG=1 for a trace).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "autoconf.h"
#include "basecmd.h"
#include "sched.h"
#include "stepper.h"
#include "trsync.h"
#include "board/misc.h"
#include "board/gpio.h"

#define F 64000000.0
#define LAT 120            // ticks of "interrupt latency" per callback
enum { STEP=1, DIR=2, TRIG=10, STOP=11, ENTRY=12, EXIT=13, GATE=14 };

void command_config_stepper(uint32_t *);
void command_queue_step(uint32_t *);
void command_set_next_step_dir(uint32_t *);
void command_reset_step_clock(uint32_t *);
void command_stepper_get_position(uint32_t *);
void command_config_buffer_feed(uint32_t *);
void command_config_buffer_feed_stop(uint32_t *);
void command_config_buffer_feed_gate(uint32_t *);
void command_config_buffer_feed_load(uint32_t *);
void command_buffer_feed_set_profile(uint32_t *);
void command_buffer_feed_set_load_profile(uint32_t *);
void command_buffer_feed_enable(uint32_t *);
void command_buffer_feed_start(uint32_t *);
void command_buffer_feed_load(uint32_t *);
void command_buffer_feed_abort(uint32_t *);
void command_buffer_feed_query(uint32_t *);
void buffer_feed_task(void);

// ---------------------------------------------------------------- checks
static int failures;
#define CHECK(c, ...) do { if (!(c)) { failures++; \
    printf("  FAIL: " __VA_ARGS__); printf("   [line %d]\n", __LINE__); } \
    else { printf("  ok:   " __VA_ARGS__); printf("\n"); } } while (0)

// ---------------------------------------------------------------- time
static uint32_t vtime = 1000000;
uint32_t timer_read_time(void) { return vtime; }
static double secs(uint32_t t0) { return (double)(uint32_t)(vtime - t0) / F; }

// ---------------------------------------------------------------- sched
static struct timer *tl[16];
static int ntl;
static int late_adds;
void sched_add_timer(struct timer *t) {
    for (int i = 0; i < ntl; i++) if (tl[i] == t) return;
    if (timer_is_before(t->waketime, vtime)) late_adds++;
    tl[ntl++] = t;
}
void sched_del_timer(struct timer *t) {
    for (int i = 0; i < ntl; i++) if (tl[i] == t) { tl[i] = tl[--ntl]; return; }
}
void sched_wake_task(struct task_wake *w) { w->wake = 1; }
uint8_t sched_check_wake(struct task_wake *w) {
    if (!w->wake)
        return 0;
    w->wake = 0;
    return 1;
}
void sched_shutdown(uint_fast8_t r) { (void)r; exit(2); }
void sim_shutdown(const char *m) { printf("SHUTDOWN: %s\n", m); exit(2); }

// ---------------------------------------------------------------- step log
struct ev { uint32_t t; int kind; int dir; };
enum { E_MAIN, E_INJ };
static struct ev *evs; static int nevs;
static int in_feed;
static int lvl[32];
static long fed;               // net steps (forward +, reverse -)
static long inj_n, main_n;
static uint32_t first_inj_t;
static long exit_seen_inj;     // injected steps when exit sensor first read active

struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val) {
    lvl[pin] = val; return (struct gpio_out){ pin }; }
void gpio_out_write(struct gpio_out g, uint32_t v) { lvl[g.id] = v; }
void gpio_out_toggle_noirq(struct gpio_out g) {
    lvl[g.id] ^= 1;
    if (g.id == STEP) {
        evs[nevs++] = (struct ev){ vtime, in_feed ? E_INJ : E_MAIN, lvl[DIR] };
        fed += lvl[DIR] ? 1 : -1;
        if (in_feed) { if (!inj_n) first_inj_t = vtime; inj_n++; }
        else main_n++;
    }
}
void sim_inject_step(struct stepper *s) {
    in_feed = 1; stepper_inject_step(s); in_feed = 0; }
uint_fast8_t sim_set_idle_dir(struct stepper *s, uint_fast8_t l) {
    in_feed = 1; uint_fast8_t r = stepper_set_idle_dir(s, l); in_feed = 0;
    return r; }

// ---------------------------------------------------------------- sensors
// Buffer position (steps).  Trigger = buffer nearly empty, stop = buffer full.
static double base_p, lo = 100, hi = 400, rate;
static uint32_t consume_t0;
static int consuming, force_trig;
static int entry_on, gate_on;
static long entry_floor = -(1L << 40);  // entry active while fed >= floor
static long exit_pos = 1L << 40;     // exit sensor active when fed >= exit_pos
static double buf_p(void) {
    double p = base_p + fed;
    if (consuming) p -= rate * (double)(uint32_t)(vtime - consume_t0) / F;
    return p;
}
static void set_rate(double r) {      // change consumption without history
    base_p = buf_p() - fed;
    consume_t0 = vtime; rate = r;
}
struct gpio_in gpio_in_setup(uint32_t pin, int32_t pu) {
    (void)pu; return (struct gpio_in){ pin }; }
uint8_t gpio_in_read(struct gpio_in g) {
    int active = 0;
    switch (g.id) {
    case TRIG: active = force_trig || buf_p() <= lo; break;
    case STOP: active = buf_p() >= hi; break;
    case ENTRY: active = entry_on && fed >= entry_floor; break;
    case GATE: active = gate_on; break;
    case EXIT:
        active = fed >= exit_pos;
        if (active && exit_seen_inj < 0) exit_seen_inj = inj_n;
        break;
    }
    return active ? 0 : 1;          // all sensors are active low
}

// ---------------------------------------------------------------- basecmd
static struct { void *type; void *p; } oids[16];
void *oid_alloc(uint8_t oid, void *type, uint16_t size) {
    oids[oid].type = type; oids[oid].p = calloc(1, size); return oids[oid].p; }
void *oid_lookup(uint8_t oid, void *type) {
    if (oids[oid].type != type) { printf("bad oid %d\n", oid); exit(3); }
    return oids[oid].p; }
void *oid_next(uint8_t *i, void *type) {
    for (;;) { (*i)++; if (*i >= 16) return NULL;
        if (oids[*i].type == type) return oids[*i].p; } }
static int mv_size;
void move_free(void *m) { free(m); }
void *move_alloc(void) { return calloc(1, mv_size); }
int move_queue_empty(struct move_queue_head *h) { return !h->first; }
struct move_node *move_queue_first(struct move_queue_head *h) { return h->first; }
int move_queue_push(struct move_node *m, struct move_queue_head *h) {
    int was = !h->first; m->next = NULL;
    if (h->first) h->last->next = m; else h->first = m;
    h->last = m; return was; }
struct move_node *move_queue_pop(struct move_queue_head *h) {
    struct move_node *m = h->first; h->first = m->next; return m; }
void move_queue_clear(struct move_queue_head *h) { h->first = NULL; }
void move_queue_setup(struct move_queue_head *h, int size) {
    h->first = h->last = NULL; mv_size = size; }
struct trsync *trsync_oid_lookup(uint8_t o) { (void)o; return NULL; }
void trsync_add_signal(struct trsync *t, struct trsync_signal *s,
                       trsync_callback_t f) { (void)t; (void)s; (void)f; }

// ---------------------------------------------------------------- sendf
static long ev_reason = -1, ev_done = -1, ev_tag = -1, ev_count, last_pos = 0;
static uint32_t ev_time;
void sim_sendf(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    char name[64]; sscanf(fmt, "%63s", name);
    long vals[24]; int n = 0;
    for (const char *p = fmt; *p; p++) if (*p == '%') {
        p++; if (*p == 'h') p++;
        if (*p == 'i') vals[n++] = va_arg(ap, int);
        else vals[n++] = (long)va_arg(ap, uint32_t);
    }
    va_end(ap);
    if (!strcmp(name, "buffer_feed_event")) {
        ev_reason = vals[1]; ev_done = vals[2]; ev_tag = vals[3]; ev_count++; ev_time = vtime;
        if (getenv("DBG")) printf("    [evt t=%.4f reason=%ld done=%ld]\n",
                                  (double)(uint32_t)(vtime - 1000000) / F,
                                  ev_reason, ev_done);
    } else if (!strcmp(name, "stepper_position")) last_pos = vals[1];
}

// ---------------------------------------------------------------- run loop
static void run_until(uint32_t end) {
    for (;;) {
        int bi = -1;
        for (int i = 0; i < ntl; i++)
            if (bi < 0 || timer_is_before(tl[i]->waketime, tl[bi]->waketime))
                bi = i;
        if (bi < 0 || !timer_is_before(tl[bi]->waketime, end)) {
            if (timer_is_before(vtime, end)) vtime = end;
            buffer_feed_task();
            return;
        }
        struct timer *t = tl[bi];
        if (timer_is_before(vtime, t->waketime)) vtime = t->waketime;
        vtime += LAT;
        uint_fast8_t r = t->func ? t->func(t) : stepper_event(t);
        if (r == SF_DONE) sched_del_timer(t);
        buffer_feed_task();
    }
}
static void run_for(double s) { run_until(vtime + (uint32_t)(s * F)); }

// ---------------------------------------------------------------- config
struct cfg {
    uint32_t start, cruise, add, dadd;       // feed ramp (add in 1/256 ticks)
    uint32_t steps, max_runs, with_stop, with_gate, with_load;
    uint32_t l_start, l_cruise, l_add, l_dadd; // load ramp
    uint32_t clear_steps, clear_back, timeout_ticks, retract_steps;
};
static struct cfg C;
static void cfg_default(void) {
    memset(&C, 0, sizeof C);
    C.steps = 500; C.start = 200000; C.cruise = 15000;
    C.add = C.dadd = (200000 - 15000) * 256 / 100;      // 100 step ramps
    C.max_runs = 3; C.with_stop = 1;
    C.l_start = 200000; C.l_cruise = 30000;
    C.l_add = C.l_dadd = (200000 - 30000) * 256 / 100;
}
static uint32_t ns_mirror;
static int host_dir;
static void setup(void) {
    uint32_t a[10];
    memset(oids, 0, sizeof oids); ntl = 0; nevs = 0; fed = 0;
    ev_count = 0; ev_reason = -1; force_trig = 0; entry_on = gate_on = 0;
    exit_pos = 1L << 40; entry_floor = -(1L << 40); consuming = 0; host_dir = 0; in_feed = 0;
    late_adds = 0; inj_n = main_n = 0; first_inj_t = 0; exit_seen_inj = -1;
    base_p = 0; rate = 0; lo = 100; hi = 400;
    a[0]=0; a[1]=STEP; a[2]=DIR; a[3]=0xFFFFFFFFu; a[4]=6;
    command_config_stepper(a);
    a[0]=0; a[1]=ns_mirror = vtime; command_reset_step_clock(a);
    a[0]=1; a[1]=0; a[2]=TRIG; a[3]=1; a[4]=0; a[5]=32000; a[6]=4;
    a[7]=C.max_runs; command_config_buffer_feed(a);
    if (C.with_stop) { a[0]=1; a[1]=STOP; a[2]=1; a[3]=0; a[4]=2;
                       command_config_buffer_feed_stop(a); }
    if (C.with_gate) { a[0]=1; a[1]=GATE; a[2]=1; a[3]=0;
                       command_config_buffer_feed_gate(a); }
    if (C.with_load) {
        a[0]=1; a[1]=ENTRY; a[2]=1; a[3]=0; a[4]=4;
        a[5]=EXIT; a[6]=1; a[7]=0; a[8]=2;
        command_config_buffer_feed_load(a);
        a[0]=1; a[1]=C.l_start; a[2]=C.l_cruise; a[3]=C.l_add; a[4]=C.l_dadd;
        a[5]=1; a[6]=C.clear_steps; a[7]=C.clear_back; a[8]=C.timeout_ticks;
        command_buffer_feed_set_load_profile(a);
    }
    a[0]=1; a[1]=C.steps; a[2]=C.start; a[3]=C.cruise; a[4]=C.add;
    a[5]=C.dadd; a[6]=1; a[7]=C.retract_steps;
    command_buffer_feed_set_profile(a);
}
enum { BE_FEED = 1, BE_LOAD = 2 };
static void enable_mask(uint32_t mask) {
    uint32_t a[2] = { 1, mask }; command_buffer_feed_enable(a); }
static void enable_feed(long on) { enable_mask(on ? BE_FEED | BE_LOAD : 0); }
enum { SEL_NONE, SEL_STOP, SEL_TRIGGER, SEL_ENTRY, SEL_EXIT, SEL_GATE };
static uint32_t move_tag = 7;
static void start_move(uint32_t steps, uint32_t sel, uint32_t level,
                       uint32_t reverse) {
    uint32_t a[10] = { 1, steps, C.start, C.cruise, C.add, C.dadd, sel, level,
                       reverse, move_tag };
    command_buffer_feed_start(a); }
static void start_manual(uint32_t steps, uint32_t use_stop) {
    start_move(steps, use_stop ? SEL_STOP : SEL_NONE, 1, 0); }
static void host_move(int dir, uint32_t start, uint32_t iv, uint32_t count) {
    uint32_t a[4];
    if (dir != host_dir) { a[0]=0; a[1]=dir; command_set_next_step_dir(a);
                           host_dir = dir; }
    a[0]=0; a[1]=(start + iv) - ns_mirror; a[2]=1; a[3]=0; command_queue_step(a);
    if (count > 1) { a[0]=0; a[1]=iv; a[2]=count - 1; a[3]=0;
                     command_queue_step(a); }
    ns_mirror = start + count * iv;
}
static long count_kind(int kind, int dir) {
    long n = 0;
    for (int i = 0; i < nevs; i++)
        if (evs[i].kind == kind && (dir < 0 || evs[i].dir == dir)) n++;
    return n;
}
static long pos_query(void) {
    uint32_t a[1] = { 0 }; command_stepper_get_position(a); return last_pos; }
// interval (ticks) between the n-th last two injected steps
static uint32_t last_interval(int back) {
    int seen = 0; uint32_t t1 = 0;
    for (int i = nevs - 1; i >= 0; i--) if (evs[i].kind == E_INJ) {
        if (seen == back) t1 = evs[i].t;
        if (seen == back + 1) return t1 - evs[i].t;
        seen++; }
    return 0;
}

// ---------------------------------------------------------------- tests
static uint32_t T0;

static void test_ramp(void) {
    printf("\n== feed move: speed ramp and exact step count\n");
    cfg_default(); setup();
    base_p = -1000; hi = 1e9;
    force_trig = 1;
    T0 = vtime; enable_feed(1);
    run_for(0.02);
    force_trig = 0; base_p = 5000;
    run_for(1.5);
    CHECK(inj_n == 500, "exactly 500 steps were fed (got %ld)", inj_n);
    CHECK(ev_reason == 1 && ev_done == 500,
          "host notified: reason=done done=%ld", ev_done);
    uint32_t prev = 0, mn = ~0u, first_iv = 0;
    for (int i = 0; i < nevs; i++) if (evs[i].kind == E_INJ) {
        if (prev) {
            uint32_t iv = evs[i].t - prev;
            if (!first_iv) first_iv = iv;
            if (iv < mn) mn = iv;
        }
        prev = evs[i].t;
    }
    CHECK(first_iv > 150000 && first_iv < 210000, "starts slow (%u ticks)",
          first_iv);
    CHECK(mn > 14000 && mn < 16500, "reaches cruise speed (%u ticks)", mn);
    CHECK(last_interval(0) > 150000, "ends slow (%u ticks)", last_interval(0));
    enable_feed(0);
}

static void test_stop_ramp(void) {
    printf("\n== stop sensor is overrun with a deceleration ramp\n");
    cfg_default(); C.steps = 3000; setup();
    hi = 400;
    start_manual(3000, 1);
    run_for(1.5);
    CHECK(ev_reason == 2, "stopped by stop sensor (reason=%ld)", ev_reason);
    CHECK(inj_n >= 496 && inj_n <= 506,
          "sensor at step 400, overran by the ramp (%ld steps)", inj_n);
    CHECK(last_interval(0) > 150000,
          "decelerated before stopping (last interval %u ticks)",
          last_interval(0));
    // intervals grow monotonically during the deceleration
    int mono = 1; uint32_t prev = 0;
    for (int k = 60; k >= 0; k--) {
        uint32_t iv = last_interval(k);
        if (prev && iv + 200 < prev) mono = 0;
        prev = iv;
    }
    CHECK(mono, "intervals increase steadily while decelerating");
    long n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n, "no steps after the stop");

    printf("  -- without ramp the stop is immediate\n");
    cfg_default(); C.add = C.dadd = 0; C.start = C.cruise = 20000;
    C.steps = 3000; setup();
    hi = 50;
    start_manual(3000, 1);
    run_for(1.0);
    CHECK(ev_reason == 2 && inj_n >= 50 && inj_n <= 53,
          "stopped right at the sensor (%ld steps)", inj_n);
}

static void test_closed_loop(const char *title, int with_main) {
    printf("\n== %s\n", title);
    cfg_default(); C.with_gate = 1; setup();
    gate_on = 1;
    base_p = 150; rate = 500; consume_t0 = vtime; consuming = 1;
    T0 = vtime;
    enable_feed(1);
    double pmin = 1e9, pmax = -1e9;
    if (with_main) {
        host_move(1, vtime + 640000, 64000, 2000);   // 1000 steps/s extruder
        rate = 500 + 1000;
    }
    for (int i = 0; i < 300; i++) {
        run_for(0.01);
        if (with_main && i == 205) set_rate(500);    // host stream ended
        double p = buf_p();
        if (secs(T0) > 0.1) { if (p < pmin) pmin = p; if (p > pmax) pmax = p; }
    }
    printf("  inject steps=%ld main steps=%ld events=%ld late_adds=%d"
           " p=[%.0f..%.0f]\n", inj_n, main_n, ev_count, late_adds, pmin, pmax);
    CHECK(pmin >= lo - 40 && pmax <= hi + 120,
          "buffer stays in range (min %.0f, max %.0f, stop sensor at %.0f)",
          pmin, pmax, hi);
    CHECK(ev_count >= 3, "feed cycles ran autonomously (%ld events)", ev_count);
    CHECK(late_adds == 0, "no timer scheduled in the past");
    if (with_main)
        CHECK(main_n == 2000, "host moves unaffected: %ld/2000 steps", main_n);
    CHECK(count_kind(E_INJ, 0) == 0, "all injected steps used forward dir");
    enable_feed(0);
    run_for(0.05);
}

static void test_dir_and_position(void) {
    printf("\n== retract in progress: feed waits, host position stays exact\n");
    cfg_default(); C.steps = 300; C.max_runs = 5; setup();
    hi = 100000;
    enable_feed(1);
    run_for(0.001);
    force_trig = 1;
    host_move(0, vtime + 64000, 32000, 1000);       // retract 1000 steps
    run_for(0.30);
    CHECK(inj_n == 0, "no injected steps while host retracts (%ld)", inj_n);
    run_for(0.40);
    force_trig = 0; base_p = 1000;
    run_for(0.5);
    CHECK(main_n == 1000, "retract steps all executed (%ld/1000)", main_n);
    CHECK(count_kind(E_MAIN, 1) == 0, "retract steps had reverse dir level");
    CHECK(inj_n > 0 && count_kind(E_INJ, 0) == 0,
          "feed steps (%ld) used forward dir", inj_n);
    CHECK(pos_query() == -1000, "host position == -1000 (got %ld)", last_pos);
    host_move(1, vtime + 64000, 32000, 500);
    run_for(0.5);
    CHECK(count_kind(E_MAIN, 1) == 500, "forward host steps used forward dir");
    CHECK(pos_query() == -500, "host position == -500 (got %ld)", last_pos);
    host_move(0, vtime + 64000, 32000, 200);
    run_for(0.4);
    CHECK(count_kind(E_MAIN, 0) == 1200, "second retract used reverse dir");
    CHECK(pos_query() == -700, "host position == -700 (got %ld)", last_pos);
    enable_feed(0);
}

static void test_fault(void) {
    printf("\n== trigger never releases -> fault after max_runs\n");
    cfg_default(); C.steps = 200; C.add = C.dadd = 0; C.start = C.cruise = 20000;
    C.with_stop = 0; setup();
    force_trig = 1; lo = 1e9;
    enable_feed(1);
    run_for(1.0);
    CHECK(inj_n == 600, "3 runs x 200 steps fed then stopped (%ld)", inj_n);
    CHECK(ev_reason == 4, "fault reported to host (reason=%ld)", ev_reason);
    run_for(0.5);
    CHECK(inj_n == 600, "no further steps after fault");
    force_trig = 0; lo = 100; base_p = 5000;
    enable_feed(1);
    run_for(0.05);
    CHECK(inj_n == 600, "re-enabled, sensor satisfied: idle");
    base_p = 0 - fed; ev_reason = -1;
    run_for(1.0);
    CHECK(inj_n == 800 && ev_reason == 1,
          "re-enable after fault restarts feeding (%ld steps)", inj_n);
    enable_feed(0);
}

static void test_unload_while_feeding(void) {
    printf("\n== unload macro while an auto feed run is in progress\n");
    cfg_default(); C.with_load = 1; C.with_gate = 1; C.steps = 600;
    C.timeout_ticks = 64000000; setup();
    gate_on = 1; lo = 1e9; hi = 1e9; force_trig = 1;   // empty buffer
    exit_pos = 1000;                   // sensor 2 active while fed >= 1000
    fed = 1200;
    enable_feed(1);
    run_for(0.15);                      // feed run is now in progress
    long n0 = inj_n;
    CHECK(n0 > 0, "auto feed running (%ld steps)", n0);
    // macro: ENABLE=0, then immediately BUFFER_FEED_MOVE (reverse, exit, release)
    enable_feed(0);
    ev_reason = -1; ev_count = 0;
    force_trig = 0;
    start_move(3000, SEL_EXIT, 0, 1);
    run_for(2.5);
    CHECK(fed < 1000 - 50, "MOVE accepted and unloaded past sensor 2 (fed=%ld)",
          fed);
    CHECK(ev_count == 2 && ev_reason == 2 && ev_tag == 7,
          "events: abort of the old run, then stop of the move (%ld, reason=%ld,"
          " tag=%ld)", ev_count, ev_reason, ev_tag);
    printf("  -- second move right after the first\n");
    long f1 = fed; move_tag = 8;
    start_move(400, SEL_NONE, 1, 1);
    run_for(2.0);
    CHECK(ev_reason == 1 && ev_tag == 8 && fed <= f1 - 395,
          "second move ran %ld steps (reason=%ld tag=%ld)", f1 - fed,
          ev_reason, ev_tag);
    move_tag = 7;
}

static void test_unload_exit_already_open(void) {
    printf("\n== unload macro, sensor 2 already open when the macro starts\n");
    cfg_default(); C.with_load = 1; C.with_gate = 1; C.steps = 600;
    C.timeout_ticks = 64000000; setup();
    lo = -1e9; hi = 1e9;
    entry_on = 1; entry_floor = -(1L << 40); exit_pos = 1L << 40;  // exit open
    enable_feed(1);
    run_for(0.05);
    enable_feed(0);
    move_tag = 3;
    start_move(3000, SEL_EXIT, 0, 1);
    run_for(0.5);
    CHECK(ev_reason == 2 && ev_tag == 3 && inj_n <= 6,
          "first move stops at once (%ld steps)", inj_n);
    long n0 = inj_n; move_tag = 4;
    start_move(400, SEL_NONE, 1, 1);
    run_for(1.5);
    CHECK(ev_reason == 1 && ev_tag == 4 && inj_n - n0 == 400,
          "second move ran %ld/400 steps (reason=%ld)", inj_n - n0, ev_reason);
    move_tag = 7;
}

static void test_manual_abort(void) {
    printf("\n== host started move: abort and busy\n");
    cfg_default(); C.add = C.dadd = 0; C.start = C.cruise = 20000;
    C.steps = 1000; setup();
    hi = 1e9;
    start_manual(1000, 0);
    run_for(0.05);
    uint32_t b[1] = { 1 }; command_buffer_feed_abort(b);
    run_for(0.05);
    long n = inj_n;
    run_for(0.2);
    CHECK(ev_reason == 3, "aborted (reason=%ld)", ev_reason);
    CHECK(inj_n == n && n > 0 && n < 1000, "feed stopped after abort (%ld)", n);
    start_manual(100, 0);
    run_for(0.001);
    start_manual(100, 0);
    run_for(0.01);
    CHECK(ev_reason == 5, "second start while running -> busy");
    run_for(1.0);
}

static void test_gate(void) {
    printf("\n== gate sensor (sensor 3) enables the automatic feed\n");
    cfg_default(); C.with_gate = 1; setup();
    hi = 1e9;
    enable_feed(1);
    force_trig = 1;
    run_for(0.3);
    CHECK(inj_n == 0, "gate open: trigger active but no feed (%ld steps)",
          inj_n);
    gate_on = 1;
    uint32_t t = vtime;
    run_for(0.02);
    CHECK(inj_n > 0, "gate active: feed starts");
    CHECK(first_inj_t && (double)(first_inj_t - t) / F < 0.0035,
          "reaction time %.2f ms", (double)(first_inj_t - t) / F * 1000);
    force_trig = 0; base_p = 5000;
    run_for(1.0);
    enable_feed(0);
}

static void test_load(void) {
    printf("\n== load: entry sensor starts, exit sensor stops (forward clear)\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.clear_steps = 150;
    C.timeout_ticks = 64000000; setup();
    hi = 1e9; lo = -1e9;               // buffer sensors quiet
    exit_pos = 400;
    enable_feed(1);
    run_for(0.05);
    uint32_t t = vtime;
    entry_on = 1;
    run_for(2.5);
    CHECK(ev_reason == 7, "load finished (reason=%ld)", ev_reason);
    CHECK(first_inj_t && (double)(first_inj_t - t) / F < 0.0035,
          "load starts within %.2f ms after sensor 1",
          (double)(first_inj_t - t) / F * 1000);
    long after = inj_n - exit_seen_inj;
    CHECK(exit_seen_inj > 0 && after >= 149 && after <= 152,
          "moved %ld steps after sensor 2 (configured 150)", after);
    CHECK(last_interval(0) > 150000, "stopped with a ramp (last interval %u)",
          last_interval(0));
    CHECK(count_kind(E_INJ, 0) == 0, "all load steps used forward dir");
    long n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n, "entry sensor still active: no second load");
    entry_on = 0; run_for(0.05);
    long c = ev_count;
    entry_on = 1; run_for(0.1);
    CHECK(ev_count == c + 1 && ev_reason == 7 && ev_done == 0 && inj_n == n,
          "exit sensor already active: reported loaded, no move");
    enable_feed(0);
    run_for(0.02);
}

static void test_enable_with_entry(void) {
    printf("\n== enable while filament is already at sensor 1: no load\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.clear_steps = 150;
    C.timeout_ticks = 64000000; setup();
    hi = 1e9; lo = -1e9;
    entry_on = 1;
    enable_feed(0);
    enable_feed(1);
    run_for(0.5);
    CHECK(inj_n == 0, "no load started (%ld steps)", inj_n);
    entry_on = 0; run_for(0.05);
    entry_on = 1; run_for(0.2);
    CHECK(inj_n > 0, "new insertion starts a load (%ld steps)", inj_n);
    enable_feed(0);
    run_for(0.02);
}

static void test_load_reverse(void) {
    printf("\n== load: free move backwards (negative clear distance)\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.clear_steps = 100;
    C.clear_back = 1; C.timeout_ticks = 64000000; setup();
    hi = 1e9; lo = -1e9;
    exit_pos = 400;
    enable_feed(1);
    entry_on = 1;
    run_for(3.0);
    long fwd = count_kind(E_INJ, 1), rev = count_kind(E_INJ, 0);
    CHECK(ev_reason == 7, "load finished (reason=%ld)", ev_reason);
    CHECK(rev == 100, "reversed exactly 100 steps (%ld)", rev);
    CHECK(fed == fwd - 100, "net travel %ld = forward %ld - 100", fed, fwd);
    int order_ok = 1, seen_rev = 0;
    for (int i = 0; i < nevs; i++) if (evs[i].kind == E_INJ) {
        if (evs[i].dir == 0) seen_rev = 1;
        else if (seen_rev) order_ok = 0;
    }
    CHECK(order_ok, "all forward steps happen before the reverse move");
    CHECK(fwd - exit_pos > 80 && fwd - exit_pos < 130,
          "stopped with a ramp before reversing (%ld steps past sensor)",
          fwd - exit_pos);
    // host can use the stepper again afterwards (dir pin restored)
    entry_on = 0;
    host_move(1, vtime + 64000, 32000, 50);
    run_for(0.3);
    CHECK(count_kind(E_MAIN, 1) == 50, "host move after load used forward dir");
    CHECK(pos_query() == 50, "host position == 50 (got %ld)", last_pos);
    enable_feed(0);
}

static void test_load_timeout(void) {
    printf("\n== load: timeout when sensor 2 never triggers\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.clear_steps = 100;
    C.timeout_ticks = (uint32_t)(0.3 * F); setup();
    hi = 1e9; lo = -1e9;
    enable_feed(1);
    uint32_t t = vtime;
    entry_on = 1;
    run_for(1.0);
    double dt = (double)(ev_time - t) / F;
    CHECK(ev_reason == 6, "timeout reported (reason=%ld)", ev_reason);
    CHECK(dt >= 0.29 && dt <= 0.32, "after %.3f s (configured 0.300 s)", dt);
    CHECK(inj_n > 100, "filament was fed meanwhile (%ld steps)", inj_n);
    long n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n && ev_count == 1,
          "stopped, no automatic retry while sensor 1 stays active");
    entry_on = 0; run_for(0.05); entry_on = 1;
    run_for(0.6);
    CHECK(ev_count == 2 && ev_reason == 6 && inj_n > n,
          "new load attempt after releasing sensor 1 (second timeout)");
    // load start from the host
    entry_on = 0; run_for(0.05);
    uint32_t a[2] = { 1, 9 }; command_buffer_feed_load(a);
    exit_pos = fed + 200;
    run_for(1.5);
    CHECK(ev_reason == 7, "host started load completes (reason=%ld)", ev_reason);
    enable_feed(0);
}

static void test_load_retract_wait(void) {
    printf("\n== load waits while the host retracts, timeout keeps counting\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.clear_steps = 50;
    C.timeout_ticks = (uint32_t)(0.4 * F); setup();
    hi = 1e9; lo = -1e9;
    exit_pos = 100;
    enable_feed(1);
    host_move(0, vtime + 64000, 32000, 1000);      // 0.5 s retract
    entry_on = 1;
    run_for(0.45);
    CHECK(inj_n == 0, "no load steps during retract (%ld)", inj_n);
    CHECK(ev_reason == 6, "timeout while waiting (reason=%ld)", ev_reason);
    enable_feed(0);
    run_for(0.2);
}

static void test_move_reverse(void) {
    printf("\n== BUFFER_FEED_MOVE backwards, stop sensor selectable\n");
    cfg_default(); C.with_load = 1; C.with_gate = 1; C.steps = 1000;
    C.timeout_ticks = 64000000; setup();
    hi = 1e9; lo = -1e9;
    // plain reverse move
    start_move(300, SEL_NONE, 1, 1);
    run_for(1.5);
    CHECK(ev_reason == 1 && inj_n == 300, "reverse move: %ld/300 steps", inj_n);
    CHECK(count_kind(E_INJ, 1) == 0 && fed == -300,
          "all steps used the reverse dir level (net %ld)", fed);
    CHECK(last_interval(0) > 150000, "reverse move ends with a ramp");
    // host can move forward afterwards (dir pin restored)
    host_move(1, vtime + 64000, 32000, 50);
    run_for(0.3);
    CHECK(count_kind(E_MAIN, 1) == 50 && pos_query() == 50,
          "host move afterwards: forward dir, host position %ld", last_pos);

    printf("  -- unload: reverse until sensor 1 is released\n");
    long n0 = inj_n, f0 = fed;
    entry_on = 1; entry_floor = f0 - 300;   // filament leaves sensor 1 after 300
    start_move(3000, SEL_ENTRY, 0, 1);
    run_for(2.5);
    long moved = inj_n - n0;
    CHECK(ev_reason == 2, "stopped by sensor (reason=%ld)", ev_reason);
    CHECK(moved >= 396 && moved <= 408,
          "released after 300 steps, overran by the ramp (%ld steps)", moved);
    CHECK(last_interval(0) > 150000, "stopped with a ramp");

    printf("  -- forward until sensor 2 triggers\n");
    n0 = inj_n; exit_pos = fed + 200;
    start_move(3000, SEL_EXIT, 1, 0);
    run_for(2.5);
    moved = inj_n - n0;
    CHECK(ev_reason == 2 && moved >= 296 && moved <= 308,
          "triggered after 200 steps (%ld steps with ramp)", moved);

    printf("  -- stop on sensor 3 (gate), already in stop state\n");
    n0 = inj_n; gate_on = 1;
    start_move(3000, SEL_GATE, 1, 0);
    run_for(0.5);
    CHECK(ev_reason == 2 && inj_n - n0 <= 5,
          "sensor already triggered: stops at once (%ld steps)", inj_n - n0);

    printf("  -- sensor not configured: move is refused\n");
    cfg_default(); C.steps = 1000; setup();
    start_move(100, SEL_EXIT, 1, 0);
    run_for(0.2);
    CHECK(ev_reason == 5 && inj_n == 0, "refused (busy), no steps");
}

static void test_runout(void) {
    printf("\n== runout: no filament at the entry sensor -> no fault\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.steps = 200;
    C.add = C.dadd = 0; C.start = C.cruise = 20000; setup();
    force_trig = 1; lo = 1e9; hi = 1e9;
    entry_on = 0;
    enable_mask(BE_FEED);
    run_for(1.0);
    CHECK(inj_n == 600, "3 runs x 200 steps fed (%ld)", inj_n);
    CHECK(ev_reason == 8, "runout reported instead of a fault (reason=%ld)",
          ev_reason);
    run_for(0.5);
    CHECK(inj_n == 600, "feeding pauses during the runout");
    printf("  -- new filament at the entry sensor: feeding resumes\n");
    entry_on = 1;
    run_for(1.0);
    CHECK(inj_n == 1200, "3 more runs after the runout ended (%ld)", inj_n);
    CHECK(ev_reason == 4, "with filament present it is a fault (reason=%ld)",
          ev_reason);
    enable_feed(0);
}

static void test_enable_mask(void) {
    printf("\n== feeding and loading are enabled separately\n");
    cfg_default(); C.with_load = 1; C.with_stop = 0; C.steps = 200;
    C.add = C.dadd = 0; C.start = C.cruise = 20000;
    C.timeout_ticks = 64000000; setup();
    hi = 1e9; exit_pos = 300;
    force_trig = 1; lo = 1e9;
    enable_mask(BE_LOAD);
    run_for(0.3);
    CHECK(inj_n == 0, "load only: buffer_low active, no feed (%ld)", inj_n);
    entry_on = 1;
    run_for(1.0);
    CHECK(ev_reason == 7 && inj_n >= 300, "load only: entry starts a load"
          " (reason=%ld, %ld steps)", ev_reason, inj_n);
    enable_mask(0);
    cfg_default(); C.with_load = 1; C.with_stop = 0;
    C.timeout_ticks = 64000000; setup();
    hi = 1e9; lo = -1e9; exit_pos = 300;
    enable_mask(BE_FEED);
    run_for(0.05);
    entry_on = 1;
    run_for(0.5);
    CHECK(inj_n == 0, "feed only: entry does not start a load (%ld)", inj_n);
    printf("  -- switching loading off stops a running load\n");
    enable_mask(BE_FEED | BE_LOAD);
    entry_on = 0; run_for(0.05); entry_on = 1; exit_pos = 1L << 40;
    run_for(0.1);
    long n = inj_n;
    CHECK(n > 0, "load running (%ld steps)", n);
    enable_mask(BE_FEED);
    run_for(0.3);
    CHECK(inj_n == n && ev_reason == 3, "load aborted (reason=%ld)",
          ev_reason);
    enable_mask(0);
}

static void test_retract(void) {
    printf("\n== buffer_high: retract until it releases\n");
    cfg_default(); C.retract_steps = 150; setup();
    lo = 100; hi = 400;
    base_p = 450;                      // pushed beyond full
    enable_feed(1);
    run_for(1.0);
    CHECK(fed < 0 && ev_reason == 2,
          "moved back and stopped at the sensor (%ld steps, reason=%ld)",
          fed, ev_reason);
    CHECK(buf_p() < hi && buf_p() > lo, "buffer between low and high"
          " (p=%.0f)", buf_p());
    long n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n, "idle afterwards");
    printf("  -- long retraction pushes the buffer far beyond full\n");
    base_p += 350;                     // needs more than 2 retract runs
    ev_reason = -1;
    run_for(2.0);
    CHECK(buf_p() < hi && buf_p() > lo && ev_reason == 2,
          "retracted in several runs, no fault (p=%.0f, reason=%ld)",
          buf_p(), ev_reason);
    printf("  -- feed run overruns buffer_high, then backs off\n");
    base_p = 0 - fed;                  // buffer empty
    C.steps = 3000;
    {
        uint32_t a[8] = { 1, C.steps, C.start, C.cruise, C.add, C.dadd, 1,
                          C.retract_steps };
        command_buffer_feed_set_profile(a);
    }
    run_for(2.0);
    CHECK(buf_p() < hi && buf_p() > lo, "fed to high and backed off"
          " (p=%.0f)", buf_p());
    n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n, "no oscillation afterwards");
    printf("  -- buffer_high never releases -> fault\n");
    hi = -1e9;
    ev_reason = -1;
    run_for(2.0);
    CHECK(ev_reason == 9, "retract fault reported (reason=%ld)", ev_reason);
    n = inj_n;
    run_for(0.5);
    CHECK(inj_n == n, "no further steps after the fault");
    enable_feed(0);
}


int main(void) {
    evs = malloc(sizeof(struct ev) * 400000);
    test_ramp();
    test_stop_ramp();
    test_closed_loop("closed loop: spring buffer, feed only", 0);
    test_closed_loop("closed loop: feed in parallel to host extrusion", 1);
    test_dir_and_position();
    test_fault();
    test_unload_while_feeding();
    test_unload_exit_already_open();
    test_manual_abort();
    test_gate();
    test_load();
    test_enable_with_entry();
    test_load_reverse();
    test_load_timeout();
    test_load_retract_wait();
    test_move_reverse();
    test_runout();
    test_enable_mask();
    test_retract();
    printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL TESTS PASSED",
           failures);
    return failures != 0;
}
