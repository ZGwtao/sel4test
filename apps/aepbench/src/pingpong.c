/* Author: Sean Peters */
/* Date: 24 May 2014 */
/* A ping pong benchmark for async ipc. Used to benchmark fastpath AEP IPC */
/* Implemented for x86 */

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
// #include <platsupport/clock.h>
#include <sel4/sel4.h>
#include <sel4bench/sel4bench.h>

/* AEPBench includes*/
#include <aepbench/gen_config.h>
#include "pingpong.h"
#include "timer.h"

#define MAX_TIMER_IRQS 4

typedef struct ipc_count {
    volatile seL4_Word calls_complete;
    volatile seL4_Word wcet;
    volatile seL4_Word wcets[100];
    // padd so we don't share cache lines
    uint32_t padding[14];
} ipc_count_t;

typedef struct ipc_counts {
    ipc_count_t ipc_counts[CONFIG_NUM_THREADS];
} ipc_counts_t;

ipc_counts_t *ipc_counts;

// ipc_count_t ipc_counts[CONFIG_NUM_THREADS];

static inline void set_ipc_buffer(thread_config_t *config) {
    seL4_SetUserData((seL4_Word)config->ipc_buf);
}

static inline void do_work() {
#if CONFIG_WORK_UNITS >= 0
    int i;
    for (i = 0; i < 1 << CONFIG_WORK_UNITS; i++) {
        asm volatile("nop");
    }
#endif
}

void
ping_thread_fn(thread_config_t *thread_config, void *unused)
{
    seL4_CPtr ep = thread_config->arg1;
    uint32_t affinity = thread_config->arg2;

    set_ipc_buffer(thread_config);

    // if (affinity == 0) printf("Ping start\n");

    uint32_t ca, cb;
    for (int i = 0; ; i++) {
        // if (i % 5000000 == 0) {
        //     printf("Ping %u\n", affinity);
        // }
        // ca = sel4bench_get_cycle_count();
        seL4_Call(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        // seL4_BenchmarkNullSyscall();
        // cb = sel4bench_get_cycle_count();
        ipc_counts->ipc_counts[affinity].calls_complete++;
        do_work();
    }
}

void
pong_thread_fn(thread_config_t *thread_config, void *init_endpoint)
{
    // while (true);
    seL4_CPtr init_ep = (seL4_CPtr)init_endpoint;
    seL4_CPtr pp_ep = thread_config->arg1;
    uint64_t affinity = thread_config->arg2;
    seL4_CPtr reply = thread_config->arg3;

    // printf("Pong start\n");

    for (;;);

    set_ipc_buffer(thread_config);

#ifdef CONFIG_KERNEL_MCS
#ifdef CONFIG_PASSIVE_SERVER
    seL4_NBSendRecv(init_ep, seL4_MessageInfo_new(0, 0, 0, 0), pp_ep, NULL, reply);
#else
    seL4_Recv(pp_ep, NULL, reply);
#endif
#else
    seL4_Wait(pp_ep, NULL);
#endif

    for (int i =0 ; ; i++) {
        // if (i % 5000000 == 0) {
        //     printf("Pong\n");
        // }
#ifdef CONFIG_KERNEL_MCS
        seL4_ReplyRecv(pp_ep, seL4_MessageInfo_new(0, 0, 0, 0), NULL, reply);
#else
        seL4_ReplyRecv(pp_ep, seL4_MessageInfo_new(0, 0, 0, 0), NULL);
#endif
        // seL4_BenchmarkNullSyscall();
    }
}

void
notification_ping_thread_fn(thread_config_t *thread_config, void *unused)
{
    seL4_CPtr ep = thread_config->arg1;
    uint32_t affinity = thread_config->arg2;

    if (affinity == 0) {
        ipc_counts->ipc_counts[affinity].wcet = 0;
        for (seL4_Word i = 0; ; i++) {
            seL4_Word pre = sel4bench_get_cycle_count();
            seL4_Signal(ep);
            seL4_Word et = sel4bench_get_cycle_count() - pre;
            ipc_counts->ipc_counts[affinity].calls_complete++;
        }
    } else {
        for (;;) {
            seL4_NBWait(ep, NULL);
        }
    }
}

void
notification_pong_thread_fn(thread_config_t *thread_config, void *init_endpoint)
{
    seL4_CPtr ep = thread_config->arg1;

    for (;;) {
        seL4_Wait(ep, NULL);
    }
}

void
interrupt_thread(thread_config_t *thread_config, void *unused)
{
    int err;

    set_ipc_buffer(thread_config);
    env_t *env = (env_t *)thread_config->arg1;

    err = ltimer_set_timeout(&env->ltimer, NS_IN_S, TIMEOUT_PERIODIC);
    assert(!err);
    uint32_t total_all_runs = 0;
    for (int it = 0; it < ITERATIONS; it ++) {
        uint32_t results[CONFIG_NUM_THREADS];
        uint32_t results2[CONFIG_NUM_THREADS];
        uint32_t total = 0;

        uint32_t cycles0, cycles1;

        /* warmup */
        for (int i = 0; i < WARMUPS; i++ ) {
            wait_for_timer_interrupt(env);
        }
        cycles0 = sel4bench_get_cycle_count();

        /* start */
        for (int i = 0; i < CONFIG_NUM_THREADS; i++) {
            results[i] = ipc_counts->ipc_counts[i].calls_complete;
        }

        wait_for_timer_interrupt(env);
        cycles1 = sel4bench_get_cycle_count();

        /* end */
        for (int i = 0; i < CONFIG_NUM_THREADS; i++) {
            results2[i] = ipc_counts->ipc_counts[i].calls_complete;
        }

        /* tally */
        aepprintf("###");
        for (int i = 0; i < CONFIG_NUM_THREADS; i++) {
            uint32_t diff = results2[i] - results[i];
            printf("%d:%d,%lu; ",i, diff, ipc_counts->ipc_counts[i].wcet);
            total += diff;
        }
        printf("Total:%d\n", total);
        total_all_runs += total;
    }

    printf("Average across all runs: %d\n", total_all_runs / ITERATIONS);
    aepprintf("All is well in the universe\n");
    while(1);
}

void
configure_pingpong_threads(env_t* env, sel4utils_thread_t* int_thread,
        sel4utils_thread_t* ping_threads, sel4utils_thread_t* pong_threads)
{
    int err;

    cspacepath_t cspace_cspath;
    vka_cspace_make_path(env->vka, 0, &cspace_cspath);

    /* Configure interrupt thread */
    sel4utils_thread_config_t int_config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, INTERRUPT_PRIORITY);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, int_config, int_thread);
    assert(!err);

    /* Configure CONFIG_NUM_THREADS ping pong threads */
    sel4utils_thread_config_t worker_config;
    worker_config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, WORKER_PRIORITY);
    for (int i = 0; i < CONFIG_NUM_THREADS; i++) {
        worker_config.sched_params.core = i;
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &ping_threads[i]);
        assert(!err);
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &pong_threads[i]);
        assert(!err);
    }
}

void
start_pingpong_pair(env_t *env, int thread_pair_id, sel4utils_thread_t* ping_thread, sel4utils_thread_t* pong_thread)
{
    vka_object_t pp_ep;
    int err;

    thread_config_t *ping_config = malloc(sizeof(thread_config_t));
    thread_config_t *pong_config = malloc(sizeof(thread_config_t));
    ping_config->ipc_buf = (void*)ping_thread->ipc_buffer_addr;
    pong_config->ipc_buf = (void*)pong_thread->ipc_buffer_addr;

    /* allocate ping and pong async endpoitns */
    err = vka_alloc_endpoint(env->vka, &pp_ep);
    assert(!err);

    /* Assign AEP to notify */
    ping_config->arg1 = /* pp_ep.cptr */ vka_alloc_notification_leaky(env->vka);
    pong_config->arg1 = pp_ep.cptr;
    /* Assign thread pair ID, which duals as CPU affinity index */
    ping_config->arg2 = thread_pair_id;
    pong_config->arg2 = thread_pair_id;

    vka_object_t reply;
    err = vka_alloc_reply(env->vka, &reply);
    assert(!err);
    pong_config->arg3 = reply.cptr;

    vka_object_t init_ep;
    err = vka_alloc_endpoint(env->vka, &init_ep);
    assert(!err);

    void *ping_function = notification_ping_thread_fn;
    void *pong_function = pong_thread_fn;

    /* Start threads */
    err = sel4utils_start_thread(ping_thread, (void*)ping_function, ping_config, NULL, 0);
    assert(!err);
    err = sel4utils_start_thread(pong_thread, (void*)pong_function, pong_config, (void *)init_ep.cptr, 0);
    assert(!err);

#if CONFIG_MAX_NUM_NODES > 1
    seL4_Time timeslice = CONFIG_BOOT_THREAD_TIME_SLICE * US_IN_S;
    seL4_CPtr sched_control = simple_get_sched_ctrl(env->simple, thread_pair_id);
    assert(sched_control != seL4_CapNull);
    err = seL4_SchedControl_Configure(sched_control, ping_thread->sched_context.cptr, timeslice, timeslice, 0, 0);
    assert(!err);
#endif

    /* seL4_TCB_Resume(pong_thread->tcb.cptr); */

    /* seL4_Wait(init_ep.cptr, NULL); */
    /* seL4_SchedContext_Unbind(pong_thread->sched_context.cptr); */

    seL4_TCB_Resume(ping_thread->tcb.cptr);
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
start_interrupt_thread(env_t* env, sel4utils_thread_t* int_thread)
{
    int err;

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

    thread_config_t *int_thread_config = malloc(sizeof(*int_thread_config));
    int_thread_config->ipc_buf = (void*)int_thread->ipc_buffer_addr;
    int_thread_config->arg1 = (seL4_Word)env;

    /* Start thread */
    err = sel4utils_start_thread(int_thread, (void*)interrupt_thread, int_thread_config, NULL, 1);
    assert(!err);
}

void
pingpong_benchmark(env_t* env)
{
    sel4utils_thread_t int_thread;
    sel4utils_thread_t ping_threads[CONFIG_NUM_THREADS];
    sel4utils_thread_t pong_threads[CONFIG_NUM_THREADS];
    int i;

    seL4_Word ipc_counts_addr = (seL4_Word)malloc((sizeof *ipc_counts) + 64);
    ipc_counts_addr = (ipc_counts_addr & ~0xF) + 0x10;
    ipc_counts = (ipc_counts_t *)ipc_counts_addr;

    aepprintf("Measuring AEP ping pong throughput\n");

    /* Configure threads */
    D("Configuring threads for ping pong benchmark.\n");
    configure_pingpong_threads(env, &int_thread, ping_threads, pong_threads);

    /* Start ping pong threads */
    D("Starting ping pong threads.\n");
    for (i = 0; i < CONFIG_NUM_THREADS; i++ ) {
        start_pingpong_pair (env, i, &ping_threads[i], &pong_threads[i]);
    }

    /* Start interrupt thread */
    D("Start interrupt thread.\n");
    start_interrupt_thread(env, &int_thread);
}
