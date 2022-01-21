/* Author: Sean Peters */
/* Date: 24 May 2014 */
/* A ping pong benchmark for async ipc. Used to benchmark fastpath AEP IPC */
/* Implemented for x86 */

/* C includes */
#include <stdio.h>
#include <stdlib.h>

/* seL4 libraries */
#include <sel4platsupport/timer.h>
#include <sel4platsupport/device.h>
#include <sel4platsupport/platsupport.h>
#include <platsupport/timer.h>
#include <platsupport/local_time_manager.h>
#include <vka/vka.h>
#include <vka/capops.h>
#include <platsupport/clock.h>
#include <sel4/sel4.h>
#include <sel4bench/sel4bench.h>

/* AEPBench includes*/
#include "pingpong.h"
#include "timer.h"

#define MAX_TIMER_IRQS 4

typedef struct ipc_count {
    volatile uint64_t calls_complete;
    // padd so we don't share cache lines
    seL4_Word padding[7];
} ipc_count_t;

ipc_count_t ipc_counts[NUM_THREADS];

static inline void set_ipc_buffer(thread_config_t *config) {
    seL4_SetUserData((seL4_Word)config->ipc_buf);
}

void 
ping_thread_fn(thread_config_t *thread_config, void *unused) 
{
    volatile uint64_t *counter = &ipc_counts[thread_config->arg2].calls_complete;
    while(1) {
        seL4_Signal(thread_config->arg1);
        seL4_Wait(thread_config->arg3, NULL);
        (*counter)++;
    }
}

void
pong_thread_fn(thread_config_t *thread_config, void *unused)
{
    while(1) {
        seL4_Wait(thread_config->arg3, NULL);
        seL4_Signal(thread_config->arg1);
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
    for (int it = 0; it < ITERATIONS; it ++) {
        uint32_t results[NUM_THREADS];
        uint32_t results2[NUM_THREADS];
        uint32_t total = 0;

        /* warmup */
        for (int i = 0; i < WARMUPS; i++ ) {
            wait_for_timer_interrupt(env);
        }

        /* start */
        for (int i = 0; i < NUM_THREADS; i++) {
            results[i] = ipc_counts[i].calls_complete;
        }

        wait_for_timer_interrupt(env);

        /* end */
        for (int i = 0; i < NUM_THREADS; i++) {
            results2[i] = ipc_counts[i].calls_complete;
        }

        /* tally */
        aepprintf("###");
        for (int i = 0; i < NUM_THREADS; i++) {
            uint32_t diff = results2[i] - results[i];
            printf("%d:%d,",i, diff);
            total += diff;
        }
        printf("Total:%d\n", total);
    }

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

    /* Configure NUM_THREADS ping pong threads */
    sel4utils_thread_config_t worker_config;
    worker_config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, WORKER_PRIORITY);
    for (int i = 0; i < NUM_THREADS; i++) {
        worker_config.sched_params.core = i;
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &ping_threads[i]);
        assert(!err);
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &pong_threads[i]);
        assert(!err);
    }
}

void
start_pingpong_pair(env_t *env, int thread_pair_id, sel4utils_thread_t* ping_thread, 
        sel4utils_thread_t* pong_thread)
{
    vka_object_t ping_aep;
    vka_object_t pong_aep;
    int err;

    thread_config_t *ping_config = malloc(sizeof(thread_config_t));
    thread_config_t *pong_config = malloc(sizeof(thread_config_t));
    ping_config->ipc_buf = (void*)ping_thread->ipc_buffer_addr;
    pong_config->ipc_buf = (void*)pong_thread->ipc_buffer_addr;

    /* allocate ping and pong async endpoitns */
    err = vka_alloc_notification(env->vka, &ping_aep);
    assert(!err);
    err = vka_alloc_notification(env->vka, &pong_aep);
    assert(!err);

    /* Assign AEP to notify */
    ping_config->arg1 = ping_aep.cptr;
    pong_config->arg1 = pong_aep.cptr;
    /* Assign thread pair ID, which duals as CPU affinity index */
    ping_config->arg2 = thread_pair_id;
    pong_config->arg2 = thread_pair_id;
    /* Assign AEP to wait on */
    ping_config->arg3 = pong_aep.cptr;
    pong_config->arg3 = ping_aep.cptr;

    /* Start threads */
    err = sel4utils_start_thread(ping_thread, (void*)ping_thread_fn, ping_config, NULL, 0); assert(!err);
    err = sel4utils_start_thread(pong_thread, (void*)pong_thread_fn, pong_config, NULL, 0); assert(!err);

#ifdef CONFIG_KERNEL_MCS
    seL4_Time timeslice = CONFIG_BOOT_THREAD_TIME_SLICE * US_IN_S;
    seL4_CPtr sched_control = simple_get_sched_ctrl(env->simple, thread_pair_id);
    assert(sched_control != seL4_CapNull);
    err = seL4_SchedControl_Configure(sched_control,
                                            ping_thread->sched_context.cptr,
                                            timeslice, timeslice, 0, 0);
    assert(!err);
    err = seL4_SchedControl_Configure(sched_control,
                                            pong_thread->sched_context.cptr,
                                            timeslice, timeslice, 0, 0);
    assert(!err);
#else
    err = seL4_TCB_SetAffinity(ping_thread->tcb.cptr, thread_pair_id); assert(!err);
    err = seL4_TCB_SetAffinity(pong_thread->tcb.cptr, thread_pair_id); assert(!err);
#endif

    seL4_TCB_Resume(ping_thread->tcb.cptr);
    seL4_TCB_Resume(pong_thread->tcb.cptr);
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

    err = vka_alloc_notification(env->vka, &env->timer_notify_test);
    ZF_LOGF_IF(err, "Failed to allocate notification object for tests");
    assert(!err);
    
    /* set up the timer manager */
    err = tm_init(&env->tm, &env->ltimer, &env->ops, 1);
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
    sel4utils_thread_t ping_threads[NUM_THREADS];
    sel4utils_thread_t pong_threads[NUM_THREADS];
    int i;
    
    aepprintf("Measuring AEP ping pong throughput\n");

    /* Configure threads */
    D("Configuring threads for ping pong benchmark.\n");
    configure_pingpong_threads(env, &int_thread, ping_threads, pong_threads);

    /* Start ping pong threads */
    D("Starting ping pong threads.\n");
    for (i = 0; i < NUM_THREADS; i++ ) {
        start_pingpong_pair (env, i, &ping_threads[i], &pong_threads[i]);
    }

    printf("started all the pingpong threads\n");

#ifdef CONFIG_RTM_PMC
    start_rtm_performance_counters();
#endif

    /* Start interrupt thread */
    D("Start interrupt thread.\n");
    start_interrupt_thread(env, &int_thread);

    printf("started the interrupt thread\n");
}
