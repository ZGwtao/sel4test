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
#include <sel4/sel4.h>
#include <sel4bench/sel4bench.h>

/* AEPBench includes*/
#include <aepbench/gen_config.h>
#include "pingpong.h"
#include "timer.h"

#define MAX_TIMER_IRQS 4
#define NUM_THREADS 4

typedef struct ipc_count {
    volatile seL4_Word calls_complete;
    // padd so we don't share cache lines
    seL4_Word padding[7];
} ipc_count_t;

ipc_count_t ipc_counts[NUM_THREADS];

static inline void do_work() {
#if CONFIG_WORK_UNITS >= 0
    int i;
    for (i = 0; i < 1 << CONFIG_WORK_UNITS; i++) {
        asm volatile("nop");
    }
#endif
}


sel4utils_thread_t int_thread = {0};
seL4_CPtr int_ntfns[2];
sel4utils_thread_t ping_threads[NUM_THREADS] = {0};
sel4utils_thread_t pong_threads[NUM_THREADS] = {0};
seL4_Word params[NUM_THREADS][2][6] = {0};

void
ipc_client_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ep = argv[0];
    volatile seL4_Word *calls_complete = (seL4_Word *)argv[1];

    for (;;) {
        seL4_Call(ep, seL4_MessageInfo_new(0, 0, 0, 0));
        (*calls_complete)++;
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
signal_pingpong_fn(void *core_, void *is_pong_, void *unused1)
{
    seL4_Word core = (seL4_Word)core_;
    int is_ping = (int)!!(bool)(seL4_Word)is_pong_;
    seL4_Word *thread_params = params[core][is_ping];

    seL4_CPtr ntfn = thread_params[0];
    seL4_CPtr init_ep = thread_params[1];
    seL4_CPtr ro = thread_params[2];
    volatile seL4_Word *calls_complete = (seL4_Word *)thread_params[3];
    seL4_CPtr pp_ep = thread_params[4];

    seL4_NBSendRecv(init_ep, seL4_MessageInfo_new(0, 0, 0, 0), pp_ep, NULL, ro);
    seL4_Signal(ntfn);

    for (;;) {
        seL4_ReplyRecv(pp_ep, seL4_MessageInfo_new(0, 0, 0, 0), NULL, ro);
        seL4_Signal(ntfn);
        (*calls_complete)++;
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
contender_fn(seL4_Word *argv, void *unused0, void *unused1)
{
    seL4_CPtr ntfn = argv[0];
    for (;;) {
        seL4_NBWait(ntfn, NULL);
    }
}

void
interrupt_thread(env_t *env, void **ntfns)
{
    int err = ltimer_set_timeout(&env->ltimer, NS_IN_S, TIMEOUT_PERIODIC);
    assert(!err);
    for (;;) {
        seL4_Wait((seL4_CPtr)ntfns[0], NULL);
        seL4_Word total_all_runs = 0;
        aepprintf("### Run\tCore 0\tCore 1\tCore 2\tCore 3\tTotal\n");
        for (int it = 0; it < ITERATIONS; it ++) {
            seL4_Word results[NUM_THREADS];
            seL4_Word results2[NUM_THREADS];
            seL4_Word total = 0;

            seL4_Word cycles0, cycles1;

            /* warmup */
            for (int i = 0; i < WARMUPS; i++ ) {
                wait_for_timer_interrupt(env);
            }
            cycles0 = sel4bench_get_cycle_count();

            /* start */
            for (int i = 0; i < NUM_THREADS; i++) {
                results[i] = ipc_counts[i].calls_complete;
            }

            wait_for_timer_interrupt(env);
            cycles1 = sel4bench_get_cycle_count();

            /* end */
            for (int i = 0; i < NUM_THREADS; i++) {
                results2[i] = ipc_counts[i].calls_complete;
            }

            /* tally */
            aepprintf("### %d\t", it);
            for (int i = 0; i < NUM_THREADS; i++) {
                seL4_Word diff = results2[i] - results[i];
                printf("%lu\t", diff);
                total += diff;
            }
            printf("%lu\n", total);
            total_all_runs += total;
        }
        aepprintf("### Average across all runs: %d\n", total_all_runs / ITERATIONS);
        seL4_Signal((seL4_CPtr)ntfns[1]);
    }
}

void
configure_pingpong_threads(env_t* env, cspacepath_t* cspace_cspath,
        sel4utils_thread_t* ping_threads, sel4utils_thread_t* pong_threads)
{
    int err;

    // cspacepath_t cspace_cspath;
    // vka_cspace_make_path(env->vka, 0, &cspace_cspath);

    /* Configure NUM_THREADS ping pong threads */
    sel4utils_thread_config_t worker_config;
    worker_config = thread_config_default(env->simple, cspace_cspath->root, seL4_NilData, 0, WORKER_PRIORITY);
    for (int i = 0; i < NUM_THREADS; i++) {
        worker_config.sched_params.core = i;
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &ping_threads[i]);
        assert(!err);
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, worker_config, &pong_threads[i]);
        assert(!err);
    }
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
start_interrupt_thread(env_t* env, sel4utils_thread_t* int_thread, seL4_CPtr *ntfns)
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

    /* Start thread */
    err = sel4utils_start_thread(int_thread, (void*)interrupt_thread, env, (void *)ntfns, 1);
    assert(!err);
}

void set_affinity(env_t *env, sel4utils_thread_t *thread, seL4_Word affinity) {
    seL4_Time timeslice = CONFIG_BOOT_THREAD_TIME_SLICE * US_IN_MS;
    seL4_CPtr sched_control = simple_get_sched_ctrl(env->simple, affinity);
    assert(sched_control != seL4_CapNull);
    int err = seL4_SchedControl_Configure(sched_control, thread->sched_context.cptr, timeslice, timeslice, 0, 0);
    assert(!err);
}

void
pingpong_benchmark(env_t* env)
{
    int err;

    aepprintf("Start interrupt thread.\n");
    cspacepath_t cspace_cspath;
    vka_cspace_make_path(env->vka, 0, &cspace_cspath);
    sel4utils_thread_config_t int_config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, INTERRUPT_PRIORITY);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, int_config, &int_thread);
    assert(!err);
    int_ntfns[0] = vka_alloc_notification_leaky(env->vka);
    int_ntfns[1] = vka_alloc_notification_leaky(env->vka);
    start_interrupt_thread(env, &int_thread, int_ntfns);

    // vka_object_t pingpong_eps[NUM_THREADS];
    // vka_object_t pong_init_eps[NUM_THREADS];
    // vka_object_t pong_ros[NUM_THREADS];
    // vka_object_t ntfns[NUM_THREADS];
    // for (int i = 0; i < NUM_THREADS; i++) {
    //     vka_alloc_endpoint(env->vka, &pingpong_eps[i]);
    //     vka_alloc_endpoint(env->vka, &pong_init_eps[i]);
    //     vka_alloc_reply(env->vka, &pong_ros[i]);
    //     vka_alloc_notification(env->vka, &ntfns[i]);
    // }


    // for (int nr_cores = 1; nr_cores <= simple_get_core_count(env->simple); nr_cores++) {
    //     aepprintf("Running IPC throughput benchmark on %d cores.\n", nr_cores);

    //     configure_pingpong_threads(env, &cspace_cspath, ping_threads, pong_threads);

    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_thread_t *ping_thread = &ping_threads[core];
    //         sel4utils_thread_t *pong_thread = &pong_threads[core];
    //         seL4_Word *ping_params = params[core][0];
    //         seL4_Word *pong_params = params[core][1];
    //         vka_object_t *pingpong_ep = &pingpong_eps[core];
    //         vka_object_t *pong_init_ep = &pong_init_eps[core];
    //         vka_object_t *pong_ro = &pong_ros[core];
    //         volatile seL4_Word *calls_complete = &ipc_counts[core].calls_complete;

    //         pong_params[0] = pingpong_ep->cptr;
    //         pong_params[1] = pong_init_ep->cptr;
    //         pong_params[2] = pong_ro->cptr;
    //         sel4utils_start_thread(pong_thread, (sel4utils_thread_entry_fn)ipc_server_fn, (void *)pong_params, NULL, 1);
    //         seL4_Wait(pong_init_ep->cptr, NULL);
    //         seL4_SchedContext_Unbind(pong_thread->sched_context.cptr);

    //         ping_params[0] = pingpong_ep->cptr;
    //         ping_params[1] = (seL4_Word)calls_complete;
    //         set_affinity(env, ping_thread, core);
    //         sel4utils_start_thread(ping_thread, (sel4utils_thread_entry_fn)ipc_client_fn, (void *)ping_params, NULL, 1);
    //     }
    //     seL4_Signal(int_ntfns[0]);
    //     seL4_Wait(int_ntfns[1], NULL);
    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_suspend_thread(&ping_threads[core]);
    //         sel4utils_suspend_thread(&pong_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &ping_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &pong_threads[core]);
    //     }
    //     aepprintf("\n");
    // }

    for (int nr_cores = 1; nr_cores <= simple_get_core_count(env->simple); nr_cores++) {
        aepprintf("Running notification ping-pong benchmark on %d cores.\n", nr_cores);

        configure_pingpong_threads(env, &cspace_cspath, ping_threads, pong_threads);

        for (seL4_Word core = 0; core < nr_cores; core++) {
            sel4utils_thread_t *ping_thread = &ping_threads[core];
            sel4utils_thread_t *pong_thread = &pong_threads[core];
            seL4_Word *ping_params = params[core][0];
            seL4_Word *pong_params = params[core][1];
            seL4_CPtr ping_ntfn = vka_alloc_notification_leaky(env->vka);
            seL4_CPtr pong_ntfn = vka_alloc_notification_leaky(env->vka);
            seL4_CPtr ping_init_ep = vka_alloc_endpoint_leaky(env->vka);
            seL4_CPtr pong_init_ep = vka_alloc_endpoint_leaky(env->vka);
            seL4_CPtr ping_ro = vka_alloc_reply_leaky(env->vka);
            seL4_CPtr pong_ro = vka_alloc_reply_leaky(env->vka);
            seL4_CPtr ping_pp_ep = vka_alloc_endpoint_leaky(env->vka);
            seL4_CPtr pong_pp_ep = vka_alloc_endpoint_leaky(env->vka);
            volatile seL4_Word *calls_complete = &ipc_counts[core].calls_complete;

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

            sel4utils_start_thread(ping_thread, signal_pingpong_fn, (void *)core, (void *)0, 1);
            sel4utils_start_thread(pong_thread, signal_pingpong_fn, (void *)core, (void *)1, 1);
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
        seL4_Signal(int_ntfns[0]);
        seL4_Wait(int_ntfns[1], NULL);
        for (int core = 0; core < nr_cores; core++) {
            sel4utils_suspend_thread(&ping_threads[core]);
            sel4utils_suspend_thread(&pong_threads[core]);
            sel4utils_clean_up_thread(env->vka, env->vspace, &ping_threads[core]);
            sel4utils_clean_up_thread(env->vka, env->vspace, &pong_threads[core]);
        }
        aepprintf("\n");
    }

    // for (int nr_cores = 1; nr_cores <= simple_get_core_count(env->simple); nr_cores++) {
    //     aepprintf("Running IPC contention benchmark on %d cores.\n", nr_cores);

    //     configure_pingpong_threads(env, &cspace_cspath, ping_threads, pong_threads);
    //     for (int core = 0; core < nr_cores; core++) {
    //         if (core == 0) {
    //             sel4utils_thread_t *ping_thread = &ping_threads[core];
    //             sel4utils_thread_t *pong_thread = &pong_threads[core];
    //             seL4_Word *ping_params = params[core][0];
    //             seL4_Word *pong_params = params[core][1];
    //             vka_object_t *pingpong_ep = &pingpong_eps[core];
    //             vka_object_t *pong_init_ep = &pong_init_eps[core];
    //             vka_object_t *pong_ro = &pong_ros[core];
    //             volatile seL4_Word *calls_complete = &ipc_counts[core].calls_complete;

    //             pong_params[0] = pingpong_ep->cptr;
    //             pong_params[1] = pong_init_ep->cptr;
    //             pong_params[2] = pong_ro->cptr;
    //             sel4utils_start_thread(pong_thread, (sel4utils_thread_entry_fn)ipc_server_fn, (void *)pong_params, NULL, 1);
    //             seL4_Wait(pong_init_ep->cptr, NULL);
    //             seL4_SchedContext_Unbind(pong_thread->sched_context.cptr);

    //             ping_params[0] = pingpong_ep->cptr;
    //             ping_params[1] = (seL4_Word)calls_complete;
    //             set_affinity(env, ping_thread, core);
    //             sel4utils_start_thread(ping_thread, (sel4utils_thread_entry_fn)ipc_client_fn, (void *)ping_params, NULL, 1);
    //         } else {
    //             sel4utils_thread_t *contender_thread = &ping_threads[core];
    //             seL4_Word *contender_params = params[core][0];
    //             vka_object_t *contender_ntfn = &ntfns[core];

    //             contender_params[0] = contender_ntfn->cptr;

    //             set_affinity(env, contender_thread, core);
    //             err = sel4utils_start_thread(contender_thread, (sel4utils_thread_entry_fn)contender_fn, (void *)contender_params, NULL, 1);
    //             assert(!err);
    //         }
    //     }
    //     seL4_Signal(int_ntfns[0]);
    //     seL4_Wait(int_ntfns[1], NULL);
    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_suspend_thread(&ping_threads[core]);
    //         sel4utils_suspend_thread(&pong_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &ping_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &pong_threads[core]);
    //     }
    //     aepprintf("\n");
    // }

    // for (int nr_cores = 1; nr_cores <= simple_get_core_count(env->simple); nr_cores++) {
    //     aepprintf("Running signal throughput benchmark on %d cores.\n", nr_cores);

    //     configure_pingpong_threads(env, &cspace_cspath, ping_threads, pong_threads);
    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_thread_t *thread = &ping_threads[core];
    //         seL4_Word *thread_params = params[core][0];
    //         vka_object_t *signaller_ntfn = &ntfns[core];

    //         thread_params[0] = signaller_ntfn->cptr;
    //         thread_params[1] = (seL4_Word)&ipc_counts[core].calls_complete;

    //         set_affinity(env, thread, core);
    //         err = sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)signal_fn, (void *)thread_params, NULL, 1);
    //         assert(!err);
    //     }
    //     seL4_Signal(int_ntfns[0]);
    //     seL4_Wait(int_ntfns[1], NULL);
    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_suspend_thread(&ping_threads[core]);
    //         sel4utils_suspend_thread(&pong_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &ping_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &pong_threads[core]);
    //     }
    //     aepprintf("\n");
    // }

    // for (int nr_cores = 1; nr_cores <= simple_get_core_count(env->simple); nr_cores++) {
    //     aepprintf("Running signal contention benchmark on %d cores.\n", nr_cores);

    //     configure_pingpong_threads(env, &cspace_cspath, ping_threads, pong_threads);
    //     for (int core = 0; core < nr_cores; core++) {
    //         if (core == 0) {
    //             sel4utils_thread_t *thread = &ping_threads[core];
    //             seL4_Word *thread_params = params[core][0];
    //             vka_object_t *signaller_ntfn = &ntfns[core];

    //             thread_params[0] = signaller_ntfn->cptr;
    //             thread_params[1] = (seL4_Word)&ipc_counts[core].calls_complete;

    //             set_affinity(env, thread, core);
    //             err = sel4utils_start_thread(thread, (sel4utils_thread_entry_fn)signal_fn, (void *)thread_params, NULL, 1);
    //             assert(!err);
    //         } else {
    //             sel4utils_thread_t *contender_thread = &ping_threads[core];
    //             seL4_Word *contender_params = params[core][0];
    //             vka_object_t *contender_ntfn = &ntfns[core];

    //             contender_params[0] = contender_ntfn->cptr;

    //             set_affinity(env, contender_thread, core);
    //             err = sel4utils_start_thread(contender_thread, (sel4utils_thread_entry_fn)contender_fn, (void *)contender_params, NULL, 1);
    //             assert(!err);
    //         }
    //     }
    //     seL4_Signal(int_ntfns[0]);
    //     seL4_Wait(int_ntfns[1], NULL);
    //     for (int core = 0; core < nr_cores; core++) {
    //         sel4utils_suspend_thread(&ping_threads[core]);
    //         sel4utils_suspend_thread(&pong_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &ping_threads[core]);
    //         sel4utils_clean_up_thread(env->vka, env->vspace, &pong_threads[core]);
    //     }
    //     aepprintf("\n");
    // }

    aepprintf("All is well in the universe\n");
    while(1);
}
