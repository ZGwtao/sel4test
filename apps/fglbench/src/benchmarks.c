/* C includes */
#include <stdio.h>
#include <stdlib.h>

/* seL4 libraries */
#include <sel4platsupport/device.h>
#include <sel4platsupport/platsupport.h>
#include <platsupport/timer.h>
#include <platsupport/local_time_manager.h>
#include <vka/vka.h>
#include <vka/capops.h>
#include <sel4/sel4.h>
#include <sel4bench/sel4bench.h>

/* FGLBench includes*/
#include <fglbench/gen_config.h>
#include "benchmarks.h"
#include "timer.h"

#define MAX_TIMER_IRQS 4

#define L1_CACHE_LINE_SIZE 64
#define PAD_TO_NEXT_CACHE_LN(used) char padding[L1_CACHE_LINE_SIZE - ((used) % L1_CACHE_LINE_SIZE)]

seL4_Word WCET[200];

void
ipc_client_throughput_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];

    for (;;) {
        seL4_Call(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        (*calls_complete)++;
    }
}

void
ipc_client_max_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];

    seL4_Call(ep, seL4_MessageInfo_new(0, 0, 0, 0));

    for (;;) {
        seL4_Word pre = sel4bench_get_cycle_count();
        seL4_Call(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        seL4_Word post = sel4bench_get_cycle_count();
        seL4_Word cost = post - pre;
        if (!WCET[101] && cost > WCET[100]) {
            WCET[100] = cost;
        }
        WCET[101] = 0;
    }
}

void
ipc_server_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr pingpong_ep = argv[0];
    seL4_CPtr init_ep = argv[1];
    seL4_CPtr ro = argv[2];

    seL4_NBSendRecv(init_ep, seL4_MessageInfo_new(0, 0, 0, 0), pingpong_ep, NULL, ro);

    for (;;) {
        seL4_ReplyRecv(pingpong_ep, seL4_MessageInfo_new(0, 0, 0, 0), NULL, ro);
    }
}

void
signal_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ntfn = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];
    for (;;) {
        seL4_Signal(ntfn);
        (*calls_complete)++;
    }
}

void
wait_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ntfn = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];
    for (;;) {
        seL4_Wait(ntfn, NULL);
        (*calls_complete)++;
    }
}

void
nbwait_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ntfn = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];
    for (;;) {
        seL4_NBWait(ntfn, NULL);
        (*calls_complete)++;
    }
}

void
nbsend_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];
    for (;;) {
        seL4_NBSend(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        (*calls_complete)++;
    }
}

void
signal_pingpong_fn(void *argv, void *unused0, void *unused1)
{
    seL4_Word *thread_params = (seL4_Word *)argv;

    seL4_CPtr ntfn = thread_params[0];
    seL4_CPtr init_ep = thread_params[1];
    seL4_CPtr ro = thread_params[2];
    volatile seL4_Word *calls_complete = (seL4_Word *)thread_params[3];
    seL4_CPtr pp_ep = thread_params[4];

    (*calls_complete)++;

    seL4_NBSendRecv(init_ep, seL4_MessageInfo_new(0, 0, 0, 0), pp_ep, NULL, ro);
    seL4_Signal(ntfn);

    for (;;) {
        seL4_ReplyRecv(pp_ep, seL4_MessageInfo_new(0, 0, 0, 0), NULL, ro);
        seL4_Signal(ntfn);
        (*calls_complete)++;
    }
}

void
send_and_recv_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    seL4_CPtr ro = argv[1];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[2];
    if (calls_complete) {
        for (;;) {
            seL4_Send(ep, seL4_MessageInfo_new(0, 0, 0, 0));
            seL4_Recv(ep, NULL, ro);
            (*calls_complete)++;
        }
    } else {
        for (;;) {
            seL4_Recv(ep, NULL, ro);
            seL4_Send(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        }
    }
}

void
signal_and_wait_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ntfn = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];
    if (calls_complete) {
        for (;;) {
            seL4_Signal(ntfn);
            seL4_Wait(ntfn, NULL);
            (*calls_complete)++;
        }
    } else {
        for (;;) {
            seL4_Wait(ntfn, NULL);
            seL4_Signal(ntfn);
        }
    }
}

void
nbsendrecv_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    seL4_CPtr ro = argv[1];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[2];
    if (calls_complete) {
        for (;;) {
            seL4_NBSendRecv(ep, seL4_MessageInfo_new(0, 0, 0, 0), ep, NULL, ro);
            (*calls_complete)++;
        }
    } else {
        for (;;) {
            seL4_NBSendRecv(ep, seL4_MessageInfo_new(0, 0, 0, 0), ep, NULL, ro);
        }
    }
}

void
nbsendwait_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr src_ep = argv[0];
    seL4_CPtr dest_ep = argv[1];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[2];
    if (calls_complete) {
        for (;;) {
            seL4_NBSendWait(dest_ep, seL4_MessageInfo_new(0, 0, 0, 0), src_ep, NULL);
            (*calls_complete)++;
        }
    } else {
        for (;;) {
            seL4_NBSendWait(dest_ep, seL4_MessageInfo_new(0, 0, 0, 0), src_ep, NULL);
        }
    }
}

void
configure_thread(env_t *env, sel4utils_thread_t *thread, uint8_t prio)
{
    int err;

    cspacepath_t cspace_cspath;
    vka_cspace_make_path(env->vka, 0, &cspace_cspath);
    sel4utils_thread_config_t config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, prio);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, thread);
    assert(!err);
}

void set_affinity(env_t *env, sel4utils_thread_t *thread, seL4_Word affinity) {
    seL4_Time timeslice = CONFIG_BOOT_THREAD_TIME_SLICE * US_IN_MS;
    seL4_CPtr sched_control = simple_get_sched_ctrl(env->simple, affinity);
    assert(sched_control != seL4_CapNull);
    int err = seL4_SchedControl_Configure(sched_control, thread->sched_context.cptr, timeslice, timeslice, 0, 0);
    assert(!err);
}

#define CONTENDER_COPY_REGISTERS 1

struct contender_core_state {
    sel4utils_thread_t thread;
    seL4_CPtr thread_params[3];

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t));
};

void
contender_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr *thread_params = (seL4_CPtr *)argv;
#if defined(CONTENDER_NULL)
    for (;;) seL4_BenchmarkNullSyscall();
#elif defined(CONTENDER_NBWAIT)
    seL4_CPtr ntfn = thread_params[0];
    for (;;) seL4_NBWait(ntfn, NULL);
#elif defined(CONTENDER_NBSEND_CAP)
    seL4_CPtr ep = thread_params[0];
    for (;;) seL4_NBSend(ep, seL4_MessageInfo_new(0, 0, 1, 0));
#elif defined(CONTENDER_NBSEND_NOCAP)
    seL4_CPtr ep = thread_params[0];
    for (;;) seL4_NBSend(ep, seL4_MessageInfo_new(0, 0, 0, 0));
#elif defined(CONTENDER_SETPRIO)
    seL4_CPtr tcb = thread_params[0];
    for (;;) seL4_TCB_SetPriority(tcb, tcb, 1);
#elif defined(CONTENDER_SET_TIMEOUT_EP)
    seL4_CPtr tcb = thread_params[0];
    for (;;) seL4_TCB_SetTimeoutEndpoint(tcb, 0);
#elif defined(CONTENDER_COPY_REGISTERS)
    seL4_CPtr src_tcb = thread_params[0];
    seL4_CPtr dest_tcb = thread_params[1];
    for (;;) seL4_TCB_CopyRegisters(src_tcb, dest_tcb, false, false, true, true, 0);
#endif
}

void start_contender_thread(env_t *env, seL4_Word core, struct contender_core_state *state)
{
    sel4utils_thread_t *thread = &state->thread;
    configure_thread(env, thread, WORKER_PRIORITY);

    seL4_CPtr *thread_params = state->thread_params;

#if defined(CONTENDER_NULL)
#elif defined(CONTENDER_NBWAIT)
    thread_params[0] = vka_alloc_notification_leaky(env->vka);
#elif defined(CONTENDER_NBSEND_CAP)
    thread_params[0] = vka_alloc_endpoint_leaky(env->vka);
#elif defined(CONTENDER_NBSEND_NOCAP)
    thread_params[0] = vka_alloc_endpoint_leaky(env->vka);
#elif defined(CONTENDER_SETPRIO)
    sel4utils_thread_t dummy_tcb;
    configure_thread(env, &dummy_tcb, 1);
    thread_params[0] = dummy_tcb.tcb.cptr;
#elif defined(CONTENDER_SET_TIMEOUT_EP)
    sel4utils_thread_t dummy_tcb;
    configure_thread(env, &dummy_tcb, 1);
    thread_params[0] = dummy_tcb.tcb.cptr;
#elif defined(CONTENDER_COPY_REGISTERS)
    sel4utils_thread_t src_tcb, dest_tcb;
    configure_thread(env, &src_tcb, 1);
    configure_thread(env, &dest_tcb, 1);
    thread_params[0] = src_tcb.tcb.cptr;
    thread_params[1] = dest_tcb.tcb.cptr;
#endif

    set_affinity(env, thread, core);
    sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)contender_fn, (void *)thread_params, NULL, 1);
}

void
nopper_fn(void *unused0, void *unused1, void *unused2)
{
    for (;;) asm volatile ("nop" ::);
}

void
interrupt_thread_throughput(env_t *env, void *counters_p)
{
    seL4_Word **counters = counters_p;
    int err = ltimer_set_timeout(&env->ltimer, NS_IN_S, TIMEOUT_PERIODIC);
    assert(!err);

    seL4_Word total_all_runs = 0;
    double average_cycles = 0;
    fglprintf("### Run\t");
    for (int i = 0; i < CONFIG_NUM_CORES; i++) {
        printf("Core %d\t", i);
    }
    printf("Total\n");
    for (int it = 0; it < ITERATIONS; it++) {
        seL4_Word results[CONFIG_NUM_CORES];
        seL4_Word results2[CONFIG_NUM_CORES];
        seL4_Word total = 0;

        seL4_Word cycles0, cycles1;

        /* warmup */
        for (int i = 0; i < WARMUPS; i++ ) {
            wait_for_timer_interrupt(env);
        }
        cycles0 = sel4bench_get_cycle_count();

        /* start */
        for (int i = 0; i < CONFIG_NUM_CORES; i++) {
            results[i] = (counters[i] != NULL) ? *counters[i] : 0;
        }
        wait_for_timer_interrupt(env);
        cycles1 = sel4bench_get_cycle_count();

        /* end */
        for (int i = 0; i < CONFIG_NUM_CORES; i++) {
            results2[i] = (counters[i] != NULL) ? *counters[i] : 0;
        }

        /* tally */
        fglprintf("### %d\t", it);
        for (int i = 0; i < CONFIG_NUM_CORES; i++) {
            seL4_Word diff = results2[i] - results[i];
            printf("%lu\t", diff);
            total += diff;
        }
        printf("%lu\n", total);
        total_all_runs += total;
        average_cycles += ((double)(cycles1 - cycles0) / ((double)total / CONFIG_NUM_CORES)) / ITERATIONS;
    }
    fglprintf("### Average across all runs: %lu\n", total_all_runs / ITERATIONS);
    fglprintf("### Average cycle cost: %lu\n", (uint64_t)average_cycles);
    fglprintf("All is well in the universe\n");
    while(1);
}

void
interrupt_thread_max(env_t *env, void *arg1)
{
    int err = ltimer_set_timeout(&env->ltimer, 60 * NS_IN_S, TIMEOUT_PERIODIC);
    assert(!err);

    seL4_Word local_max;
    WCET[100] = 0;
    for (int i = 0; i < MAX_ET_RUNTIME_MINS; i++) {
        WCET[101] = 1;
        wait_for_timer_interrupt(env);
        local_max = WCET[100];
    }
    fglprintf("### Maximum cycle cost: %lu\n", local_max);

    fglprintf("All is well in the universe\n");
    while(1);
}

static irq_id_t sel4test_timer_irq_register(void *cookie, ps_irq_t irq, irq_callback_fn_t callback,
                                            void *callback_data)
{
    static int num_timer_irqs = 0;

    env_t *env = cookie;

    int error;

    ZF_LOGF_IF(!callback, "Passed in a NULL callback");

    ZF_LOGF_IF(num_timer_irqs >= MAX_TIMER_IRQS, "Trying to register too many timer IRQs");

    /* Allocate the IRQ */
    error = sel4platsupport_copy_irq_cap(env->vka, env->simple, &irq,
                                         &env->timer_irqs[num_timer_irqs].handler_path);
    ZF_LOGF_IF(error, "Failed to allocate IRQ handler");
    assert(!error);

    /* Allocate the root notifitcation if we haven't already done so */
    if (env->timer_notification.cptr == seL4_CapNull) {
        error = vka_alloc_notification(env->vka, &env->timer_notification);
        ZF_LOGF_IF(error, "Failed to allocate notification object");
        assert(!error);
    }

    /* Mint a notification for the IRQ handler to pair with */
    error = vka_cspace_alloc_path(env->vka, &env->badged_timer_notifications[num_timer_irqs]);
    ZF_LOGF_IF(error, "Failed to allocate path for the badged notification");
    assert(!error);
    cspacepath_t root_notification_path = {0};
    vka_cspace_make_path(env->vka, env->timer_notification.cptr, &root_notification_path);
    error = vka_cnode_mint(&env->badged_timer_notifications[num_timer_irqs], &root_notification_path,
                           seL4_AllRights, BIT(num_timer_irqs));
    ZF_LOGF_IF(error, "Failed to mint notification for timer");
    assert(!error);

    /* Pair the notification and the handler */
    error = seL4_IRQHandler_SetNotification(env->timer_irqs[num_timer_irqs].handler_path.capPtr,
                                            env->badged_timer_notifications[num_timer_irqs].capPtr);
    ZF_LOGF_IF(error, "Failed to pair the notification and handler together");
    assert(!error);

    /* Ack the handler so interrupts can come in */
    error = seL4_IRQHandler_Ack(env->timer_irqs[num_timer_irqs].handler_path.capPtr);
    ZF_LOGF_IF(error, "Failed to ack the IRQ handler");
    assert(!error);

    /* Fill out information about the callbacks */
    env->timer_cbs[num_timer_irqs].callback = callback;
    env->timer_cbs[num_timer_irqs].callback_data = callback_data;

    return num_timer_irqs++;
}

void
run_interrupt_thread(env_t* env, bool max, seL4_Word **counters)
{
    int err;

    sel4utils_thread_t *thread = calloc(1, sizeof *thread);
    configure_thread(env, thread, INTERRUPT_PRIORITY);

    ps_irq_register_fn_t irq_register_fn_copy = env->ops.irq_ops.irq_register_fn;
    void *cookie_copy = env->ops.irq_ops.cookie;
    env->ops.irq_ops.irq_register_fn = sel4test_timer_irq_register;
    env->ops.irq_ops.cookie = env;

    /* setup the timers and have our wrapper around simple capture the IRQ caps */
    err = ltimer_default_init(&env->ltimer, env->ops, NULL, NULL);
    ZF_LOGF_IF(err, "Failed to setup the timers");
    assert(!err);

    /* Restore the IRQ interface's register function */
    env->ops.irq_ops.irq_register_fn = irq_register_fn_copy;
    env->ops.irq_ops.cookie = cookie_copy;

    /* Start thread */
    if (max) {
        err = sel4utils_start_thread(thread, (void*)interrupt_thread_max, env, NULL, 1);
    } else {
        err = sel4utils_start_thread(thread, (void*)interrupt_thread_throughput, env, (void *)counters, 1);
    }
    assert(!err);
}

struct ipc_pingpong_core_state {
    sel4utils_thread_t ping_thread, pong_thread;
    seL4_Word ping_params[2], pong_params[3], counter, wcet;

    PAD_TO_NEXT_CACHE_LN(2 * sizeof (sel4utils_thread_t) + 6 * sizeof (seL4_Word));
};

struct notification_pingpong_core_state {
    sel4utils_thread_t ping_thread, pong_thread, nopper_thread;
    seL4_Word ping_params[5], pong_params[5], counter;

    PAD_TO_NEXT_CACHE_LN(2 * sizeof (sel4utils_thread_t) + 11 * sizeof (seL4_Word));
};

struct signal_core_state {
    sel4utils_thread_t thread;
    seL4_Word params[2], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct nbwait_core_state {
    sel4utils_thread_t thread;
    seL4_Word params[2], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct nbsend_core_state {
    sel4utils_thread_t thread;
    seL4_Word params[2], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct send_and_recv_core_state {
    sel4utils_thread_t send_thread, recv_thread;
    seL4_Word send_params[3], recv_params[3], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct signal_and_wait_core_state {
    sel4utils_thread_t send_thread, recv_thread;
    seL4_Word send_params[2], recv_params[2], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct nbsendrecv_core_state {
    sel4utils_thread_t send_thread, recv_thread;
    seL4_Word send_params[3], recv_params[3], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

struct nbsendwait_core_state {
    sel4utils_thread_t send_thread, recv_thread;
    seL4_Word send_params[3], recv_params[3], counter;

    PAD_TO_NEXT_CACHE_LN(sizeof (sel4utils_thread_t) + 3 * sizeof (seL4_Word));
};

void
start_ipc_pingpong_pair(env_t *env, seL4_Word core, struct ipc_pingpong_core_state *state, bool max)
{
    configure_thread(env, &state->ping_thread, WORKER_PRIORITY);
    configure_thread(env, &state->pong_thread, WORKER_PRIORITY);

    state->pong_params[0] = vka_alloc_endpoint_leaky(env->vka);
    state->pong_params[1] = vka_alloc_endpoint_leaky(env->vka);
    state->pong_params[2] = vka_alloc_reply_leaky(env->vka);
    sel4utils_start_thread(&state->pong_thread, (sel4utils_thread_entry_fn)ipc_server_fn, (void *)state->pong_params, NULL, 1);
    seL4_Wait(state->pong_params[1], NULL);
    seL4_SchedContext_Unbind(state->pong_thread.sched_context.cptr);

    state->ping_params[0] = state->pong_params[0];
    state->ping_params[1] = (seL4_Word)&state->counter;
    set_affinity(env, &state->ping_thread, core);
    sel4utils_start_thread(
        &state->ping_thread,
        (sel4utils_thread_entry_fn)(max ? ipc_client_max_fn : ipc_client_throughput_fn),
        (void *)state->ping_params,
        NULL,
        1
    );
}

void
start_notification_pingpong_pair(env_t *env, seL4_Word core, struct notification_pingpong_core_state *state)
{
    sel4utils_thread_t *ping_thread = &state->ping_thread;
    sel4utils_thread_t *pong_thread = &state->pong_thread;
    sel4utils_thread_t *nopper_thread = &state->nopper_thread;
    configure_thread(env, ping_thread, WORKER_PRIORITY);
    configure_thread(env, pong_thread, WORKER_PRIORITY);
    // configure_thread(env, nopper_thread, WORKER_PRIORITY + 1);
    seL4_Word *ping_params = state->ping_params;
    seL4_Word *pong_params = state->pong_params;
    seL4_CPtr ping_ntfn = vka_alloc_notification_leaky(env->vka);
    seL4_CPtr pong_ntfn = vka_alloc_notification_leaky(env->vka);
    seL4_CPtr ping_init_ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_CPtr pong_init_ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_CPtr ping_ro = vka_alloc_reply_leaky(env->vka);
    seL4_CPtr pong_ro = vka_alloc_reply_leaky(env->vka);
    seL4_CPtr ping_pp_ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_CPtr pong_pp_ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_Word *calls_complete = &state->counter;

    pong_params[0] = pong_ntfn;
    pong_params[1] = pong_init_ep;
    pong_params[2] = pong_ro;
    pong_params[3] = (seL4_Word)calls_complete;
    pong_params[4] = pong_pp_ep;

    ping_params[0] = ping_ntfn;
    ping_params[1] = ping_init_ep;
    ping_params[2] = ping_ro;
    ping_params[3] = (seL4_Word)calls_complete;
    ping_params[4] = ping_pp_ep;

    sel4utils_start_thread(ping_thread, signal_pingpong_fn, ping_params, NULL, 1);
    sel4utils_start_thread(pong_thread, signal_pingpong_fn, pong_params, NULL, 1);
    // set_affinity(env, nopper_thread, core);
    // sel4utils_start_thread(nopper_thread, nopper_fn, NULL, NULL, 1);

    seL4_Wait(pong_init_ep, NULL);
    seL4_SchedContext_Unbind(pong_thread->sched_context.cptr);
    seL4_Wait(ping_init_ep, NULL);
    seL4_SchedContext_Unbind(ping_thread->sched_context.cptr);
    set_affinity(env, ping_thread, core);
    set_affinity(env, pong_thread, core);

    seL4_SchedContext_Bind(ping_thread->sched_context.cptr, pong_ntfn);
    seL4_SchedContext_Bind(pong_thread->sched_context.cptr, ping_ntfn);
    seL4_TCB_BindNotification(ping_thread->tcb.cptr, pong_ntfn);
    seL4_TCB_BindNotification(pong_thread->tcb.cptr, ping_ntfn);

    seL4_Signal(ping_ntfn);
}

void
start_signaller_thread(env_t *env, seL4_Word core, struct signal_core_state *state)
{
    sel4utils_thread_t *thread = &state->thread;
    configure_thread(env, thread, WORKER_PRIORITY);
    seL4_Word *params = state->params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ntfn = vka_alloc_notification_leaky(env->vka);

    params[0] = ntfn;
    params[1] = (seL4_Word)calls_complete;

    set_affinity(env, thread, core);
    sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)signal_fn, (void *)params, NULL, 1);
}

void
start_signaller_waiter_thread_pair(env_t *env, seL4_Word core, struct signal_core_state *signaller_state, struct signal_core_state *waiter_state)
{
    sel4utils_thread_t *signaller_thread = &signaller_state->thread;
    configure_thread(env, signaller_thread, WORKER_PRIORITY);
    seL4_Word *signaller_params = signaller_state->params;
    seL4_Word *signaller_calls_complete = &signaller_state->counter;

    sel4utils_thread_t *waiter_thread = &waiter_state->thread;
    configure_thread(env, waiter_thread, WORKER_PRIORITY-1);
    seL4_Word *waiter_params = waiter_state->params;
    seL4_Word *waiter_calls_complete = &waiter_state->counter;

    seL4_CPtr ntfn = vka_alloc_notification_leaky(env->vka);

    signaller_params[0] = ntfn;
    signaller_params[1] = (seL4_Word)signaller_calls_complete;
    waiter_params[0] = ntfn;
    waiter_params[1] = (seL4_Word)waiter_calls_complete;

    set_affinity(env, signaller_thread, 0);
    set_affinity(env, waiter_thread, core);
    sel4utils_start_thread(signaller_thread, (sel4utils_thread_entry_fn)signal_fn, (void *)signaller_params, NULL, 1);
    sel4utils_start_thread(waiter_thread, (sel4utils_thread_entry_fn)wait_fn, (void *)waiter_params, NULL, 1);
}

void
start_nbwaiter_thread(env_t *env, seL4_Word core, struct nbwait_core_state *state)
{
    sel4utils_thread_t *thread = &state->thread;
    configure_thread(env, thread, WORKER_PRIORITY);
    seL4_Word *params = state->params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ntfn = vka_alloc_notification_leaky(env->vka);

    params[0] = ntfn;
    params[1] = (seL4_Word)calls_complete;

    set_affinity(env, thread, core);
    sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)nbwait_fn, (void *)params, NULL, 1);
}

void
start_nbsend_thread(env_t *env, seL4_Word core, struct nbsend_core_state *state)
{
    sel4utils_thread_t *thread = &state->thread;
    configure_thread(env, thread, WORKER_PRIORITY);
    seL4_Word *params = state->params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ep = vka_alloc_endpoint_leaky(env->vka);

    params[0] = ep;
    params[1] = (seL4_Word)calls_complete;

    set_affinity(env, thread, core);
    sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)nbsend_fn, (void *)params, NULL, 1);
}

void
start_send_and_recv_threads(env_t *env, seL4_Word core, struct send_and_recv_core_state *state)
{
    sel4utils_thread_t *send_thread = &state->send_thread;
    sel4utils_thread_t *recv_thread = &state->recv_thread;
    configure_thread(env, send_thread, WORKER_PRIORITY);
    configure_thread(env, recv_thread, WORKER_PRIORITY);

    seL4_Word *send_params = state->send_params;
    seL4_Word *recv_params = state->recv_params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_CPtr ro0 = vka_alloc_reply_leaky(env->vka);
    seL4_CPtr ro1 = vka_alloc_reply_leaky(env->vka);

    send_params[0] = ep;
    send_params[1] = ro0;
    send_params[2] = (seL4_Word)calls_complete;

    recv_params[0] = ep;
    recv_params[1] = ro1;
    recv_params[2] = (seL4_Word)NULL;

    set_affinity(env, send_thread, core);
    set_affinity(env, recv_thread, core);
    sel4utils_start_thread(send_thread, (sel4utils_thread_entry_fn)send_and_recv_fn, (void *)send_params, NULL, 1);
    sel4utils_start_thread(recv_thread, (sel4utils_thread_entry_fn)send_and_recv_fn, (void *)recv_params, NULL, 1);
}

void
start_signal_and_wait_threads(env_t *env, seL4_Word core, struct signal_and_wait_core_state *state)
{
    sel4utils_thread_t *send_thread = &state->send_thread;
    sel4utils_thread_t *recv_thread = &state->recv_thread;
    configure_thread(env, send_thread, WORKER_PRIORITY);
    configure_thread(env, recv_thread, WORKER_PRIORITY);

    seL4_Word *send_params = state->send_params;
    seL4_Word *recv_params = state->recv_params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ntfn = vka_alloc_notification_leaky(env->vka);

    send_params[0] = ntfn;
    send_params[1] = (seL4_Word)calls_complete;

    recv_params[0] = ntfn;
    recv_params[1] = (seL4_Word)NULL;

    set_affinity(env, send_thread, core);
    set_affinity(env, recv_thread, core);
    sel4utils_start_thread(send_thread, (sel4utils_thread_entry_fn)signal_and_wait_fn, (void *)send_params, NULL, 1);
    sel4utils_start_thread(recv_thread, (sel4utils_thread_entry_fn)signal_and_wait_fn, (void *)recv_params, NULL, 1);
}

void
start_nbsendrecv_threads(env_t *env, seL4_Word core, struct nbsendrecv_core_state *state)
{
    sel4utils_thread_t *send_thread = &state->send_thread;
    sel4utils_thread_t *recv_thread = &state->recv_thread;
    configure_thread(env, send_thread, WORKER_PRIORITY);
    configure_thread(env, recv_thread, WORKER_PRIORITY);

    seL4_Word *send_params = state->send_params;
    seL4_Word *recv_params = state->recv_params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ep = vka_alloc_endpoint_leaky(env->vka);
    seL4_CPtr ro0 = vka_alloc_reply_leaky(env->vka);
    seL4_CPtr ro1 = vka_alloc_reply_leaky(env->vka);

    send_params[0] = ep;
    send_params[1] = ro0;
    send_params[2] = (seL4_Word)calls_complete;

    recv_params[0] = ep;
    recv_params[1] = ro1;
    recv_params[2] = (seL4_Word)NULL;

    set_affinity(env, send_thread, core);
    set_affinity(env, recv_thread, core);
    sel4utils_start_thread(send_thread, (sel4utils_thread_entry_fn)nbsendrecv_fn, (void *)send_params, NULL, 1);
    sel4utils_start_thread(recv_thread, (sel4utils_thread_entry_fn)nbsendrecv_fn, (void *)recv_params, NULL, 1);
}

void
start_nbsendwait_threads(env_t *env, seL4_Word core, struct nbsendwait_core_state *state)
{
    sel4utils_thread_t *send_thread = &state->send_thread;
    sel4utils_thread_t *recv_thread = &state->recv_thread;
    configure_thread(env, send_thread, WORKER_PRIORITY);
    configure_thread(env, recv_thread, WORKER_PRIORITY);

    seL4_Word *send_params = state->send_params;
    seL4_Word *recv_params = state->recv_params;
    seL4_Word *calls_complete = &state->counter;

    seL4_CPtr ep0 = vka_alloc_notification_leaky(env->vka);
    seL4_CPtr ep1 = vka_alloc_notification_leaky(env->vka);

    send_params[0] = ep0;
    send_params[1] = ep0;
    send_params[2] = (seL4_Word)calls_complete;

    recv_params[0] = ep0;
    recv_params[1] = ep0;
    recv_params[2] = (seL4_Word)NULL;

    set_affinity(env, send_thread, core);
    set_affinity(env, recv_thread, core);
    sel4utils_start_thread(send_thread, (sel4utils_thread_entry_fn)nbsendwait_fn, (void *)send_params, NULL, 1);
    sel4utils_start_thread(recv_thread, (sel4utils_thread_entry_fn)nbsendwait_fn, (void *)recv_params, NULL, 1);
}

void
bm_ipc_tp(env_t *env)
{
    struct ipc_pingpong_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }

    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_ipc_pingpong_pair(env, core, &core_state[core], false);
    }
}

void
bm_ipc_tp_nc(env_t *env)
{
    struct ipc_pingpong_core_state pingpong_core_state = {0};
    struct contender_core_state contender_core_state[CONFIG_NUM_CORES];
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    counters[0] = &pingpong_core_state.counter;

    run_interrupt_thread(env, false, counters);

    start_ipc_pingpong_pair(env, 0, &pingpong_core_state, false);

    seL4_Word num_contenders = CONFIG_NUM_CORES - 1;
    for (seL4_Word i = 0; i < num_contenders; i++) {
        start_contender_thread(env, i + 1, &contender_core_state[i]);
    }
}

void
bm_ipc_tp_1c(env_t *env)
{
    struct ipc_pingpong_core_state pingpong_core_state[CONFIG_NUM_CORES] = {0};
    struct contender_core_state contender_core_state = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &pingpong_core_state[core].counter;
    }

    run_interrupt_thread(env, false, counters);

    start_ipc_pingpong_pair(env, 0, &pingpong_core_state[0], false);
    for (seL4_Word core = 1; core < CONFIG_NUM_CORES - 1; core++) {
        start_ipc_pingpong_pair(env, core, &pingpong_core_state[core], false);
    }

    if (CONFIG_NUM_CORES > 1) {
        start_contender_thread(env, CONFIG_NUM_CORES - 1, &contender_core_state);
    }
}

void
bm_ipc_max_nc(env_t *env)
{
    struct ipc_pingpong_core_state pingpong_core_state = {0};
    struct contender_core_state contender_core_state[CONFIG_NUM_CORES];

    run_interrupt_thread(env, true, NULL);

    start_ipc_pingpong_pair(env, 0, &pingpong_core_state, true);

    seL4_Word num_contenders = CONFIG_NUM_CORES - 1;
    for (seL4_Word i = 0; i < num_contenders; i++) {
        start_contender_thread(env, i+1, &contender_core_state[i]);
    }
}

void
bm_ntfn_tp(env_t *env)
{
    struct notification_pingpong_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }

    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_notification_pingpong_pair(env, core, &core_state[core]);
    }
}

void
bm_ntfn_tp_nc(env_t *env)
{
    struct notification_pingpong_core_state pingpong_core_state = {0};
    struct contender_core_state contender_core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    counters[0] = &pingpong_core_state.counter;

    run_interrupt_thread(env, false, counters);

    start_notification_pingpong_pair(env, 3, &pingpong_core_state);

    for (seL4_Word core = 1; core < CONFIG_NUM_CORES; core++) {
        start_contender_thread(env, 3-core, &contender_core_state[core - 1]);
    }
}

void
bm_ntfn_tp_1c(env_t *env)
{
    struct notification_pingpong_core_state pingpong_core_state[CONFIG_NUM_CORES] = {0};
    struct contender_core_state contender_core_state = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &pingpong_core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    start_notification_pingpong_pair(env, 0, &pingpong_core_state[0]);
    for (seL4_Word core = 1; core < CONFIG_NUM_CORES - 1; core++) {
        start_notification_pingpong_pair(env, core, &pingpong_core_state[core]);
    }

    if (CONFIG_NUM_CORES > 1) {
        start_contender_thread(env, CONFIG_NUM_CORES - 1, &contender_core_state);
    }
}

void
bm_signal_tp(env_t *env)
{
    struct signal_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }

    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_signaller_thread(env, core, &core_state[core]);
    }
}

void
bm_signal_tp_nc(env_t *env)
{
    struct signal_core_state signal_core_state = {0};
    struct contender_core_state contender_core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    counters[0] = &signal_core_state.counter;

    run_interrupt_thread(env, false, counters);

    start_signaller_thread(env, 0, &signal_core_state);

    for (seL4_Word core = 1; core < CONFIG_NUM_CORES; core++) {
        start_contender_thread(env, core, &contender_core_state[core - 1]);
    }
}

void
bm_signal_tp_1c(env_t *env)
{
    struct signal_core_state signal_core_state[CONFIG_NUM_CORES] = {0};
    struct contender_core_state contender_core_state = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &signal_core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    start_signaller_thread(env, 0, &signal_core_state[0]);
    for (seL4_Word core = 1; core < CONFIG_NUM_CORES - 1; core++) {
        start_signaller_thread(env, core, &signal_core_state[core]);
    }

    if (CONFIG_NUM_CORES > 1) {
        start_contender_thread(env, CONFIG_NUM_CORES - 1, &contender_core_state);
    }
}

void
bm_nbwait_tp(env_t *env)
{
    struct nbwait_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_nbwaiter_thread(env, core, &core_state[core]);
    }
}

void
bm_nbsend_tp(env_t *env)
{
    struct nbsend_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_nbsend_thread(env, core, &core_state[core]);
    }
}

void
bm_send_and_recv_tp(env_t *env)
{
    struct send_and_recv_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_send_and_recv_threads(env, core, &core_state[core]);
    }
}

void bm_signal_and_wait_tp(env_t *env)
{
    struct signal_and_wait_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_signal_and_wait_threads(env, core, &core_state[core]);
    }
}

void bm_nbsendrecv_tp(env_t *env)
{
    struct nbsendrecv_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_nbsendrecv_threads(env, core, &core_state[core]);
    }
}

void bm_nbsendwait_tp(env_t *env)
{
    struct nbsendwait_core_state core_state[CONFIG_NUM_CORES] = {0};
    seL4_Word *counters[CONFIG_NUM_CORES] = {0};

    for (int core = 0; core < CONFIG_NUM_CORES; core++) {
        counters[core] = &core_state[core].counter;
    }
    run_interrupt_thread(env, false, counters);

    for (seL4_Word core = 0; core < CONFIG_NUM_CORES; core++) {
        start_nbsendwait_threads(env, core, &core_state[core]);
    }
}
