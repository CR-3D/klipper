// Autonomous filament buffer feeding and loading
//
// Runs entirely on the MCU, without any host involvement in the timing:
//
//  * "Feeding": a trigger sensor (buffer nearly empty) immediately starts
//    extra steps on an existing stepper.  A stop sensor (buffer full) ends
//    the feed - the sensor may be overrun, the stepper decelerates with a
//    ramp.  An optional gate sensor (filament present) enables the feeding.
//  * "Retract": if the stop sensor triggers (buffer pushed beyond full, eg
//    by a long retraction), the stepper moves back until it releases.
//  * "Loading": an entry sensor starts the stepper, which runs until the exit
//    sensor triggers (or a timeout expires).  After the exit sensor
//    triggered the stepper moves a configured distance and stops.
//  * "Runout": when the entry sensor releases while feeding is enabled and
//    the exit sensor still detects filament, a runout is reported at once.
//    Feeding then reacts to edges of the trigger sensor only: one run per
//    trigger, no run limit and no fault.  Once the end of the filament has
//    left the feeder a run no longer releases the trigger and feeding stops.
//    This lasts until filament is inserted again.
//
// Feeding (and retract) and loading are enabled separately by the host.
//
// The extra steps are merged into the step stream of the stepper, so this
// works in parallel with normal (host queued) moves of the same stepper.
// This requires a stepper using "step on both edges" (the default with TMC
// drivers), where every toggle of the step pin is one step.
//
// Copyright (C) 2026  CR-3D
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_in_setup
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "stepper.h" // stepper_inject_step

enum {
    BF_ENABLED = 1<<0, BF_HAVE_STOP = 1<<1, BF_AUTO = 1<<2,
    BF_REPORT = 1<<3, BF_NEED_RELEASE = 1<<4, BF_TIMER = 1<<5,
    BF_ABORT = 1<<6, BF_HAVE_GATE = 1<<7, BF_HAVE_LOAD = 1<<8,
    BF_NEED_ENTRY_RELEASE = 1<<9, BF_STOPPING = 1<<10,
    BF_REVERSE_AFTER = 1<<11, BF_LOAD_ON = 1<<12, BF_RUNOUT = 1<<13,
    BF_RETRACT = 1<<14, BF_ENTRY_SEEN = 1<<15,
};

// Bits of the enable mask sent by the host
enum { BE_FEED = 1<<0, BE_LOAD = 1<<1 };

enum { BS_IDLE=0, BS_RUN=1, BS_FAULT=2 };

enum { BR_NONE=0, BR_DONE=1, BR_STOP=2, BR_ABORT=3, BR_FAULT=4, BR_BUSY=5,
       BR_TIMEOUT=6, BR_LOADED=7, BR_RUNOUT=8, BR_RETRACT_FAULT=9 };

enum { RK_FEED=0, RK_LOAD=1 };

// Sensors that can end a feed move
enum { SEL_NONE=0, SEL_STOP=1, SEL_TRIGGER=2, SEL_ENTRY=3, SEL_EXIT=4,
       SEL_GATE=5 };

// Stages of a load run
enum { ST_SEEK=0, ST_POST=1, ST_REVERSE=2 };

// Intervals are kept with 8 fractional bits
#define FP_SHIFT 8
#define MAX_INTERVAL (1UL << (32 - FP_SHIFT - 1))

struct bf_sensor {
    struct gpio_in pin;
    uint8_t active_val, samples, cnt;
};

// Speed profile (intervals and per-step interval changes in 1/256 ticks)
struct bf_profile {
    uint32_t start, cruise, add, dadd, steps;
    uint8_t dir;
};

struct buffer_feed {
    struct timer time;
    struct stepper *stepper;
    struct bf_sensor trig, stop, gate, entry, exit;
    uint32_t poll_ticks;
    uint32_t flags;
    uint8_t state, reason, tag, rtag, max_runs, runs, rruns, kind, stage;
    uint8_t clear_back, retract_max_runs, entry_rel;
    // Sensor that ends the active feed move
    struct bf_sensor *sel_sensor;
    uint8_t sel_level, sel_samples, sel_cnt;
    // Profiles and load parameters set by the host
    struct bf_profile feed_p, load_p;
    uint32_t clear_steps, load_timeout, retract_steps;
    // Active run
    struct bf_profile r;
    uint32_t remaining, done, interval, nd_max, time_left, total;
};

static struct task_wake buffer_feed_wake;

// Return the sensor for a SEL_x value (NULL if none or not configured)
static struct bf_sensor *
bf_sel_sensor(struct buffer_feed *b, uint8_t sel)
{
    switch (sel) {
    case SEL_STOP: return (b->flags & BF_HAVE_STOP) ? &b->stop : NULL;
    case SEL_TRIGGER: return &b->trig;
    case SEL_ENTRY: return (b->flags & BF_HAVE_LOAD) ? &b->entry : NULL;
    case SEL_EXIT: return (b->flags & BF_HAVE_LOAD) ? &b->exit : NULL;
    case SEL_GATE: return (b->flags & BF_HAVE_GATE) ? &b->gate : NULL;
    }
    return NULL;
}

static uint_fast8_t buffer_feed_poll_event(struct timer *t);
static void bf_abort_now(struct buffer_feed *b);
static uint_fast8_t buffer_feed_step_event(struct timer *t);

static uint8_t
bf_active(struct bf_sensor *s)
{
    return (!!gpio_in_read(s->pin)) == s->active_val;
}

// True once a sensor was active for 'samples' consecutive calls
static uint8_t
bf_sensor_hit(struct bf_sensor *s)
{
    if (!bf_active(s)) {
        s->cnt = 0;
        return 0;
    }
    if (++s->cnt < s->samples)
        return 0;
    s->cnt = 0;
    return 1;
}

static void
bf_sensor_setup(struct bf_sensor *s, uint32_t pin, uint32_t pull_up
                , uint32_t active, uint32_t samples)
{
    s->pin = gpio_in_setup(pin, pull_up);
    s->active_val = !!active;
    s->samples = samples ? samples : 1;
    s->cnt = 0;
}

// Safe next wake time (never in the past)
static uint32_t
bf_nextwake(uint32_t wake, uint32_t inc)
{
    uint32_t next = wake + inc;
    uint32_t min = timer_read_time() + timer_from_us(2);
    if (timer_is_before(next, min))
        return min;
    return next;
}

// Reschedule the step timer 'inc' ticks after its last wake time
static uint_fast8_t
bf_resched(struct buffer_feed *b, uint32_t inc)
{
    struct timer *t = &b->time;
    uint32_t next = bf_nextwake(t->waketime, inc);
    if (b->kind == RK_LOAD && b->stage == ST_SEEK && b->load_timeout) {
        uint32_t used = next - t->waketime;
        b->time_left = used >= b->time_left ? 0 : b->time_left - used;
    }
    t->waketime = next;
    return SF_RESCHEDULE;
}

// Begin a run.  Irqs must be disabled and the timer must not be queued.
static void
bf_start_run(struct buffer_feed *b, const struct bf_profile *p
             , uint32_t steps, uint8_t kind, uint8_t sel, uint8_t level)
{
    b->flags &= ~(BF_AUTO | BF_ABORT | BF_STOPPING | BF_REVERSE_AFTER
                  | BF_RETRACT);
    b->sel_sensor = bf_sel_sensor(b, sel);
    b->sel_level = !!level;
    b->sel_samples = sel == SEL_STOP ? b->stop.samples : 2;
    b->sel_cnt = 0;
    b->r = *p;
    b->tag = 0;
    b->kind = kind;
    b->stage = ST_SEEK;
    b->state = BS_RUN;
    // Without an acceleration ramp the run starts at full speed
    b->interval = p->add ? p->start : p->cruise;
    b->remaining = steps;
    b->done = 0;
    b->exit.cnt = 0;
    b->nd_max = p->dadd ? (p->start - p->cruise) / p->dadd + 1 : 0;
    b->time_left = b->load_timeout;
    b->time.func = buffer_feed_step_event;
    b->time.waketime = timer_read_time() + timer_from_us(5);
}

// Finish a run (called from timer context)
static uint_fast8_t
bf_finish(struct buffer_feed *b, uint8_t reason)
{
    b->state = BS_IDLE;
    b->reason = reason;
    b->rtag = b->tag;
    b->flags |= BF_REPORT;
    if (b->kind == RK_LOAD)
        // Do not load again until the entry sensor has been released
        b->flags |= BF_NEED_ENTRY_RELEASE;
    if (!(b->flags & BF_AUTO))
        // A load or manual move starts a new series of feed runs
        b->runs = 0;
    else if (!(b->flags & BF_RETRACT)
             && (reason == BR_STOP || reason == BR_ABORT))
        // Do not restart until the trigger sensor has been released
        b->flags |= BF_NEED_RELEASE;
    b->flags &= ~BF_RETRACT;
    b->time.func = buffer_feed_poll_event;
    b->time.waketime = bf_nextwake(b->time.waketime, b->poll_ticks);
    sched_wake_task(&buffer_feed_wake);
    return SF_RESCHEDULE;
}

// Interval for the next step (accelerate, cruise or decelerate)
static uint32_t
bf_next_interval(struct buffer_feed *b)
{
    const struct bf_profile *p = &b->r;
    uint32_t iv = b->interval, rem = b->remaining;
    if (b->flags & BF_STOPPING) {
        // Forced deceleration
        iv += p->dadd;
        return iv > p->start ? p->start : iv;
    }
    if (p->dadd && rem <= b->nd_max
        && rem <= (p->start - iv) / p->dadd) {
        // Last steps of the move - decelerate
        iv += p->dadd;
        return iv > p->start ? p->start : iv;
    }
    if (iv > p->cruise && p->add) {
        uint32_t niv = (iv - p->cruise > p->add) ? iv - p->add : p->cruise;
        // Only accelerate if there are enough steps left to stop again
        if (!p->dadd || rem > b->nd_max
            || rem > (p->start - niv) / p->dadd)
            iv = niv;
    }
    return iv;
}

// Start a controlled stop (a sensor was hit)
static void
bf_begin_decel(struct buffer_feed *b)
{
    const struct bf_profile *p = &b->r;
    b->flags |= BF_STOPPING;
    b->remaining = p->dadd ? (p->start - b->interval) / p->dadd : 0;
}

// The exit sensor was hit during a load run
static void
bf_load_hit(struct buffer_feed *b)
{
    b->stage = ST_POST;
    if (!b->clear_steps) {
        bf_begin_decel(b);
    } else if (!b->clear_back) {
        // Continue forward for the configured distance, then stop with ramp
        b->remaining = b->clear_steps;
    } else {
        // Stop, then reverse for the configured distance
        b->flags |= BF_REVERSE_AFTER;
        bf_begin_decel(b);
    }
}

// All steps of the current move have been taken
static uint_fast8_t
bf_run_end(struct buffer_feed *b)
{
    if (b->flags & BF_REVERSE_AFTER) {
        b->flags &= ~(BF_REVERSE_AFTER | BF_STOPPING);
        b->stage = ST_REVERSE;
        b->r.dir = !b->r.dir;
        b->interval = b->r.add ? b->r.start : b->r.cruise;
        b->remaining = b->clear_steps;
        return bf_resched(b, b->r.start >> FP_SHIFT);
    }
    if (b->kind == RK_LOAD)
        return bf_finish(b, BR_LOADED);
    return bf_finish(b, (b->flags & BF_STOPPING) ? BR_STOP : BR_DONE);
}

// Report a fault from the poll timer and stop polling
static uint_fast8_t
bf_poll_fault(struct buffer_feed *b, uint8_t reason)
{
    b->state = BS_FAULT;
    b->reason = reason;
    b->rtag = 0;
    b->flags |= BF_REPORT;
    b->flags &= ~BF_TIMER;
    sched_wake_task(&buffer_feed_wake);
    return SF_DONE;
}

// Timer callback while waiting for a sensor
static uint_fast8_t
buffer_feed_poll_event(struct timer *t)
{
    struct buffer_feed *b = container_of(t, struct buffer_feed, time);
    uint32_t flags = b->flags;
    if (!(flags & (BF_ENABLED | BF_LOAD_ON)) || b->state == BS_FAULT) {
        b->flags &= ~BF_TIMER;
        return SF_DONE;
    }
    // Entry sensor: load sequence and end of a runout
    if (flags & BF_HAVE_LOAD) {
        if (!bf_active(&b->entry)) {
            b->entry.cnt = 0;
            b->flags &= ~BF_NEED_ENTRY_RELEASE;
            if ((flags & (BF_ENABLED | BF_ENTRY_SEEN | BF_RUNOUT))
                == (BF_ENABLED | BF_ENTRY_SEEN)
                && ++b->entry_rel >= b->entry.samples) {
                b->flags &= ~BF_ENTRY_SEEN;
                if (bf_active(&b->exit)) {
                    // Filament end passed the entry sensor of a channel that
                    // is loaded through - runout (not a removed filament)
                    b->flags |= BF_RUNOUT | BF_REPORT;
                    b->reason = BR_RUNOUT;
                    b->rtag = 0;
                    b->done = 0;
                    b->runs = 0;
                    sched_wake_task(&buffer_feed_wake);
                }
            }
        } else {
            b->entry_rel = 0;
            b->flags |= BF_ENTRY_SEEN;
            if (flags & BF_RUNOUT) {
                // New filament - feeding may resume
                b->flags &= ~(BF_RUNOUT | BF_NEED_RELEASE);
                b->runs = b->rruns = 0;
            }
            if ((flags & BF_LOAD_ON) && !(flags & BF_NEED_ENTRY_RELEASE)
                && b->load_p.cruise && bf_sensor_hit(&b->entry)) {
                b->flags |= BF_NEED_ENTRY_RELEASE;
                if (bf_active(&b->exit)) {
                    // Filament is already at the exit sensor - nothing to load
                    b->done = 0;
                    b->kind = RK_LOAD;
                    b->reason = BR_LOADED;
                    b->rtag = 0;
                    b->flags |= BF_REPORT;
                    sched_wake_task(&buffer_feed_wake);
                } else {
                    bf_start_run(b, &b->load_p, 0xFFFFFFFF, RK_LOAD, SEL_NONE
                                 , 0);
                    return SF_RESCHEDULE;
                }
            }
        }
    }
    flags = b->flags;
    if (flags & BF_ENABLED) {
        // Retract: stop sensor (buffer pushed beyond full)
        if ((flags & BF_HAVE_STOP) && b->retract_steps
            && !(flags & BF_RUNOUT)) {
            if (!bf_active(&b->stop)) {
                b->stop.cnt = 0;
                b->rruns = 0;
            } else if (bf_sensor_hit(&b->stop)) {
                if (b->retract_max_runs
                    && b->rruns >= b->retract_max_runs)
                    // Still beyond full after all permitted retracts
                    return bf_poll_fault(b, BR_RETRACT_FAULT);
                b->rruns++;
                struct bf_profile p = b->feed_p;
                p.dir = !p.dir;
                bf_start_run(b, &p, b->retract_steps, RK_FEED, SEL_STOP, 0);
                b->flags |= BF_AUTO | BF_RETRACT;
                return SF_RESCHEDULE;
            }
        }
        // Feeding: trigger sensor (only while the gate sensor is active)
        uint8_t gate_ok = !(flags & BF_HAVE_GATE) || bf_active(&b->gate);
        if (!bf_active(&b->trig)) {
            b->trig.cnt = 0;
            b->runs = 0;
            b->flags &= ~BF_NEED_RELEASE;
        } else if (!gate_ok) {
            b->trig.cnt = 0;
        } else if (!(flags & BF_NEED_RELEASE) && b->feed_p.steps
                   && bf_sensor_hit(&b->trig)) {
            if (flags & BF_RUNOUT) {
                // Runout: one run per trigger edge, no limit, no fault
                bf_start_run(b, &b->feed_p, b->feed_p.steps, RK_FEED, SEL_STOP
                             , 1);
                b->flags |= BF_AUTO | BF_NEED_RELEASE;
                return SF_RESCHEDULE;
            }
            if (b->max_runs && b->runs >= b->max_runs) {
                // Trigger still active after all permitted runs
                if ((flags & BF_HAVE_LOAD) && !bf_active(&b->entry)) {
                    // No filament at the entry sensor - runout, no fault
                    b->flags |= BF_RUNOUT | BF_REPORT | BF_NEED_RELEASE;
                    b->reason = BR_RUNOUT;
                    b->rtag = 0;
                    b->done = 0;
                    sched_wake_task(&buffer_feed_wake);
                } else {
                    return bf_poll_fault(b, BR_FAULT);
                }
            } else {
                b->runs++;
                bf_start_run(b, &b->feed_p, b->feed_p.steps, RK_FEED, SEL_STOP
                             , 1);
                b->flags |= BF_AUTO;
                return SF_RESCHEDULE;
            }
        }
    }
    t->waketime = bf_nextwake(t->waketime, b->poll_ticks);
    return SF_RESCHEDULE;
}

// Timer callback that generates the extra steps
static uint_fast8_t
buffer_feed_step_event(struct timer *t)
{
    struct buffer_feed *b = container_of(t, struct buffer_feed, time);
    uint32_t flags = b->flags;
    if (flags & BF_ABORT)
        return bf_finish(b, BR_ABORT);
    uint8_t seeking = b->kind == RK_LOAD && b->stage == ST_SEEK;
    if (seeking && b->load_timeout && !b->time_left)
        return bf_finish(b, BR_TIMEOUT);
    // Sensors (sampled before every step)
    if (!(flags & BF_STOPPING)) {
        if (b->kind == RK_FEED && b->sel_sensor) {
            if (bf_active(b->sel_sensor) == b->sel_level) {
                if (++b->sel_cnt >= b->sel_samples)
                    bf_begin_decel(b);
            } else {
                b->sel_cnt = 0;
            }
        } else if (seeking) {
            if (bf_sensor_hit(&b->exit))
                bf_load_hit(b);
        }
    }
    if (!b->remaining)
        return bf_run_end(b);
    // Direction handling
    struct stepper *s = b->stepper;
    if (stepper_get_dir_level(s) != b->r.dir) {
        if (!stepper_set_idle_dir(s, b->r.dir))
            // Stepper is moving in the other direction (eg, retract) -
            // wait until it is done
            return bf_resched(b, b->poll_ticks);
        // Allow direction setup time before the first step
        return bf_resched(b, timer_from_us(20));
    }
    stepper_inject_step(s);
    b->done++;
    b->total++;
    if (!--b->remaining)
        return bf_run_end(b);
    uint32_t iv = bf_next_interval(b);
    b->interval = iv;
    return bf_resched(b, iv >> FP_SHIFT);
}

static void
bf_profile_set(struct bf_profile *p, uint32_t start, uint32_t cruise
               , uint32_t add, uint32_t dadd, uint32_t steps, uint32_t dir)
{
    if (start >= MAX_INTERVAL || cruise >= MAX_INTERVAL || !cruise
        || start < cruise)
        shutdown("buffer_feed interval out of range");
    p->start = start << FP_SHIFT;
    p->cruise = cruise << FP_SHIFT;
    p->add = add;
    p->dadd = dadd;
    p->steps = steps;
    p->dir = !!dir;
}

void
command_config_buffer_feed(uint32_t *args)
{
    struct buffer_feed *b = oid_alloc(
        args[0], command_config_buffer_feed, sizeof(*b));
    b->stepper = stepper_lookup_shared(args[1]);
    if (!b->stepper)
        shutdown("buffer_feed needs a stepper using step on both edges");
    bf_sensor_setup(&b->trig, args[2], args[3], args[4], args[6]);
    b->poll_ticks = args[5];
    b->max_runs = args[7];
}
DECL_COMMAND(command_config_buffer_feed,
             "config_buffer_feed oid=%c stepper_oid=%c trigger_pin=%c"
             " trigger_pull_up=%c trigger_active=%c poll_ticks=%u"
             " trigger_debounce=%c max_runs=%c");

void
command_config_buffer_feed_stop(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    bf_sensor_setup(&b->stop, args[1], args[2], args[3], args[4]);
    b->retract_max_runs = args[5];
    b->flags |= BF_HAVE_STOP;
}
DECL_COMMAND(command_config_buffer_feed_stop,
             "config_buffer_feed_stop oid=%c stop_pin=%c stop_pull_up=%c"
             " stop_active=%c stop_samples=%c retract_max_runs=%c");

void
command_config_buffer_feed_gate(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    bf_sensor_setup(&b->gate, args[1], args[2], args[3], 1);
    b->flags |= BF_HAVE_GATE;
}
DECL_COMMAND(command_config_buffer_feed_gate,
             "config_buffer_feed_gate oid=%c gate_pin=%c gate_pull_up=%c"
             " gate_active=%c");

void
command_config_buffer_feed_load(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    bf_sensor_setup(&b->entry, args[1], args[2], args[3], args[4]);
    bf_sensor_setup(&b->exit, args[5], args[6], args[7], args[8]);
    b->flags |= BF_HAVE_LOAD;
}
DECL_COMMAND(command_config_buffer_feed_load,
             "config_buffer_feed_load oid=%c entry_pin=%c entry_pull_up=%c"
             " entry_active=%c entry_debounce=%c exit_pin=%c exit_pull_up=%c"
             " exit_active=%c exit_samples=%c");

void
command_buffer_feed_set_profile(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    struct bf_profile p;
    bf_profile_set(&p, args[2], args[3], args[4], args[5], args[1], args[6]);
    irq_disable();
    b->feed_p = p;
    b->retract_steps = args[7];
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_set_profile,
             "buffer_feed_set_profile oid=%c steps=%u start_interval=%u"
             " cruise_interval=%u accel_add=%u decel_add=%u dir=%c"
             " retract_steps=%u");

void
command_buffer_feed_set_load_profile(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    struct bf_profile p;
    bf_profile_set(&p, args[1], args[2], args[3], args[4], 0, args[5]);
    irq_disable();
    b->load_p = p;
    b->clear_steps = args[6];
    b->clear_back = !!args[7];
    b->load_timeout = args[8];
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_set_load_profile,
             "buffer_feed_set_load_profile oid=%c start_interval=%u"
             " cruise_interval=%u accel_add=%u decel_add=%u dir=%c"
             " clear_steps=%u clear_back=%c timeout_ticks=%u");

// Enable feeding (BE_FEED) and/or loading (BE_LOAD)
void
command_buffer_feed_enable(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    uint8_t mask = args[1];
    irq_disable();
    uint32_t flags = b->flags & ~(BF_ENABLED | BF_LOAD_ON);
    if (mask & BE_FEED)
        flags |= BF_ENABLED;
    if (mask & BE_LOAD)
        flags |= BF_LOAD_ON;
    b->flags = flags;
    // Stop a run whose automatic function was switched off (all runs if
    // everything was switched off)
    if (b->state == BS_RUN) {
        uint8_t auto_load = b->kind == RK_LOAD && !b->tag;
        if (!mask || ((flags & BF_AUTO) && !(mask & BE_FEED))
            || (auto_load && !(mask & BE_LOAD)))
            bf_abort_now(b);
    }
    if (mask) {
        if (b->state == BS_FAULT)
            b->state = BS_IDLE;
        b->runs = b->rruns = b->trig.cnt = b->stop.cnt = b->entry.cnt = 0;
        b->flags &= ~(BF_RUNOUT | BF_ENTRY_SEEN);
        b->entry_rel = 0;
        // Filament that is already at sensor 1 does not start a load
        if ((b->flags & BF_HAVE_LOAD) && bf_active(&b->entry))
            b->flags |= BF_NEED_ENTRY_RELEASE | BF_ENTRY_SEEN;
        if (!(b->flags & BF_TIMER)) {
            b->flags |= BF_TIMER;
            b->time.func = buffer_feed_poll_event;
            b->time.waketime = timer_read_time() + b->poll_ticks;
            sched_add_timer(&b->time);
        }
    }
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_enable, "buffer_feed_enable oid=%c enable=%c");

// Stop a running move at once (called with irqs disabled from a command).
// The state is idle afterwards, so a command that follows can start a move.
static void
bf_abort_now(struct buffer_feed *b)
{
    if (b->state != BS_RUN)
        return;
    if (b->flags & BF_TIMER)
        sched_del_timer(&b->time);
    b->flags &= ~(BF_TIMER | BF_AUTO | BF_ABORT | BF_STOPPING
                  | BF_REVERSE_AFTER | BF_RETRACT);
    b->state = BS_IDLE;
    b->reason = BR_ABORT;
    b->rtag = b->tag;
    b->flags |= BF_REPORT;
    if (b->kind == RK_LOAD)
        b->flags |= BF_NEED_ENTRY_RELEASE;
    sched_wake_task(&buffer_feed_wake);
    if (b->flags & (BF_ENABLED | BF_LOAD_ON)) {
        b->flags |= BF_TIMER;
        b->time.func = buffer_feed_poll_event;
        b->time.waketime = timer_read_time() + b->poll_ticks;
        sched_add_timer(&b->time);
    }
}

// Report that a command could not be executed
static void
bf_report_busy(struct buffer_feed *b, uint8_t tag)
{
    b->reason = BR_BUSY;
    b->rtag = tag;
    b->flags |= BF_REPORT;
    sched_wake_task(&buffer_feed_wake);
}

// Start a feed move from the host
void
command_buffer_feed_start(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    struct bf_profile p;
    bf_profile_set(&p, args[2], args[3], args[4], args[5], args[1]
                   , args[8] ? !b->feed_p.dir : b->feed_p.dir);
    irq_disable();
    if (b->state != BS_IDLE || !args[1]
        || (args[6] != SEL_NONE && !bf_sel_sensor(b, args[6]))) {
        bf_report_busy(b, args[9]);
        irq_enable();
        return;
    }
    if (b->flags & BF_TIMER)
        sched_del_timer(&b->time);
    b->flags |= BF_TIMER;
    bf_start_run(b, &p, args[1], RK_FEED, args[6], args[7]);
    b->tag = args[9];
    sched_add_timer(&b->time);
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_start,
             "buffer_feed_start oid=%c steps=%u start_interval=%u"
             " cruise_interval=%u accel_add=%u decel_add=%u stop_sel=%c"
             " stop_level=%c reverse=%c tag=%c");

// Start the load sequence from the host
void
command_buffer_feed_load(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    irq_disable();
    if (b->state != BS_IDLE || !(b->flags & BF_HAVE_LOAD)
        || !b->load_p.cruise) {
        bf_report_busy(b, args[1]);
        irq_enable();
        return;
    }
    if (b->flags & BF_TIMER)
        sched_del_timer(&b->time);
    b->flags |= BF_TIMER;
    bf_start_run(b, &b->load_p, 0xFFFFFFFF, RK_LOAD, SEL_NONE, 0);
    b->tag = args[1];
    sched_add_timer(&b->time);
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_load, "buffer_feed_load oid=%c tag=%c");

void
command_buffer_feed_abort(uint32_t *args)
{
    struct buffer_feed *b = oid_lookup(args[0], command_config_buffer_feed);
    irq_disable();
    bf_abort_now(b);
    irq_enable();
}
DECL_COMMAND(command_buffer_feed_abort, "buffer_feed_abort oid=%c");

void
command_buffer_feed_query(uint32_t *args)
{
    uint8_t oid = args[0];
    struct buffer_feed *b = oid_lookup(oid, command_config_buffer_feed);
    irq_disable();
    uint8_t state = b->state, reason = b->reason, runs = b->runs;
    uint32_t flags = b->flags;
    uint32_t done = b->done, total = b->total, remaining = b->remaining;
    irq_enable();
    uint8_t trig = bf_active(&b->trig);
    uint8_t stop = (flags & BF_HAVE_STOP) && bf_active(&b->stop);
    uint8_t gate = (flags & BF_HAVE_GATE) && bf_active(&b->gate);
    uint8_t entry = (flags & BF_HAVE_LOAD) && bf_active(&b->entry);
    uint8_t exit = (flags & BF_HAVE_LOAD) && bf_active(&b->exit);
    sendf("buffer_feed_state oid=%c state=%c reason=%c enabled=%c runs=%c"
          " trigger=%c stop=%c gate=%c entry=%c exit=%c done=%u remaining=%u"
          " total=%u"
          , oid, state, reason
          , ((flags & BF_ENABLED) ? BE_FEED : 0)
            | ((flags & BF_LOAD_ON) ? BE_LOAD : 0)
            | ((flags & BF_RUNOUT) ? 4 : 0), runs, trig, stop
          , gate, entry, exit, done, remaining, total);
}
DECL_COMMAND(command_buffer_feed_query, "buffer_feed_query oid=%c");

// Report finished runs to the host
void
buffer_feed_task(void)
{
    if (!sched_check_wake(&buffer_feed_wake))
        return;
    uint8_t oid;
    struct buffer_feed *b;
    foreach_oid(oid, b, command_config_buffer_feed) {
        if (!(b->flags & BF_REPORT))
            continue;
        irq_disable();
        uint8_t reason = b->reason, tag = b->rtag;
        uint32_t done = b->done;
        b->flags &= ~BF_REPORT;
        irq_enable();
        sendf("buffer_feed_event oid=%c reason=%c done=%u tag=%c"
              , oid, reason, done, tag);
    }
}
DECL_TASK(buffer_feed_task);

void
buffer_feed_shutdown(void)
{
    uint8_t oid;
    struct buffer_feed *b;
    foreach_oid(oid, b, command_config_buffer_feed) {
        b->flags &= ~(BF_ENABLED | BF_LOAD_ON | BF_RUNOUT);
        b->state = BS_IDLE;
    }
}
DECL_SHUTDOWN(buffer_feed_shutdown);
