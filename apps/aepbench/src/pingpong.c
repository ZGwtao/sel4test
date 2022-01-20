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

#define MS_IN_SECOND 1000
#define uS_IN_SECOND MS_IN_SECOND * 1000
#define NS_IN_uS 1000

static env_t *ENV;

typedef struct ipc_count {
    volatile uint32_t calls_complete;
    // padd so we don't share cache lines
    uint32_t padding[15];
} ipc_count_t;

ipc_count_t ipc_counts[NUM_THREADS];

static inline void set_ipc_buffer(thread_config_t *config) {
    seL4_SetUserData((seL4_Word)config->ipc_buf);
}

void
start_rtm_performance_counters()
{
    // sel4bench_init(); 

    /* Counts all cycles in RTM region */
    //sel4bench_set_count_event(0, SEL4BENCH_IA32_EVENT_CYCLE_CORE);
    //sel4bench_set_count_intx_bits(0, 1, 0);
    /* Counts only commited cycles in RTM region */
    //sel4bench_set_count_event(2, SEL4BENCH_IA32_EVENT_CYCLE_CORE);
    //sel4bench_set_count_intx_bits(2, 1, 1);

    // sel4bench_set_count_event(0, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_START);
    // sel4bench_set_count_event(1, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_COMMIT);
    // sel4bench_set_count_event(2, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED);
    // sel4bench_set_count_event(3, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED_MEM_EVENT);
    // sel4bench_set_count_event(4, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED_RARE);
    // sel4bench_set_count_event(5, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED_HLE);
    // sel4bench_set_count_event(6, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED_MEM_TYPE);
    // sel4bench_set_count_event(7, SEL4BENCH_IA32_HASWELL_EVENT_RTM_RETIRED_ABORTED_OTHER);

    // sel4bench_start_counters(0xFF);
}

void
report_rtm_performance_counters()
{
    sel4bench_stop_counters(0xFF);

    //aepprintf("RTM all:         %u (cycles)\n", (unsigned int)sel4bench_get_counter(0));
    //aepprintf("RTM commit:      %u (cycles)\n", (unsigned int)sel4bench_get_counter(2));
    // aepprintf("RTM start:       %u (events)\n", (unsigned int)sel4bench_get_counter(0));
    // aepprintf("RTM commit:      %u (events)\n", (unsigned int)sel4bench_get_counter(1));
    // aepprintf("RTM abrt:        %u (events)\n", (unsigned int)sel4bench_get_counter(2));
    // aepprintf("RTM abrt mem ev: %u (events)\n", (unsigned int)sel4bench_get_counter(3));
    // aepprintf("RTM abrt rare:   %u (events)\n", (unsigned int)sel4bench_get_counter(4));
    // aepprintf("RTM abrt hle:    %u (events)\n", (unsigned int)sel4bench_get_counter(5));
    // aepprintf("RTM abrt mem ty: %u (events)\n", (unsigned int)sel4bench_get_counter(6));
    // aepprintf("RTM abrt other:  %u (events)\n", (unsigned int)sel4bench_get_counter(7));
}

void 
ping_thread_fn(thread_config_t *thread_config, void *unused) 
{
    /* arg1 is async endpoint to notify on
     * arg2 is thread id
     * arg3 is async endpoint to wait on */

    volatile uint32_t *counter = &ipc_counts[thread_config->arg2].calls_complete;
    while(1) {
        seL4_Signal(thread_config->arg1);
        seL4_Wait(thread_config->arg3, NULL);
        (*counter)++;
    }
}

void 
pong_thread_fn(thread_config_t *thread_config, void *unused) 
{
    /* arg1 is async endpoint to notify on
     * arg2 is thread id
     * arg3 is async endpoint to wait on */

    while(1) {
        seL4_Wait(thread_config->arg3, NULL);
        seL4_Signal(thread_config->arg1);
    }
}

void 
interrupt_thread(thread_config_t *thread_config, void *unused) 
{
    int i, j;
    int it;
    int err;

    /* arg1 is an empty slot
     * arg2 is async endpoint
     * arg3 is vaddr of timer */

    set_ipc_buffer(thread_config);
    // seL4_timer_t *timer = (seL4_timer_t*)thread_config->arg3;
    // assert(timer);
    seL4_CPtr aep = (seL4_CPtr)thread_config->arg2;
    // err = timer_periodic(timer->timer, uS_IN_SECOND); FIX
    err = ltimer_set_timeout(&ENV->ltimer, NS_IN_S, TIMEOUT_PERIODIC);
    assert(!err);
    for (it = 0; it < ITERATIONS; it ++) {
        uint32_t results[NUM_THREADS];
        uint32_t results2[NUM_THREADS];
        uint32_t total = 0;

        /* warmup */
        for (i = 0; i < WARMUPS; i++ ) {
            for (j = 0; j < NUM_THREADS; ++j) {
                wait_for_timer_interrupt(ENV);
                // seL4_Word badge;
                // seL4_Wait(aep, &badge);
                // sel4_timer_handle_single_irq(timer); FIX
                // sel4platsupport_handle_timer_irq(timer, badge);
            }
        }

        /* start */
        for (i = 0; i < NUM_THREADS; i++) {
            results[i] = ipc_counts[i].calls_complete;
        }
        
        /* wait */
        for (j = 0; j < NUM_THREADS; ++j) {
            wait_for_timer_interrupt(ENV);
            // seL4_Word badge;
            // seL4_Wait(aep, &badge);
            // sel4_timer_handle_single_irq(timer); FIX
            // sel4platsupport_handle_timer_irq(timer, badge);
        }

        /* end */
        for (i = 0; i < NUM_THREADS; i++) {
            results2[i] = ipc_counts[i].calls_complete;
        }

        /* tally */
        aepprintf("###");
        for (i = 0; i < NUM_THREADS; i++) {
            uint32_t diff = results2[i] - results[i];
            printf("%d:%d,",i, diff);
            total += diff;
        }
        printf("Total:%d\n", total);
    }

#ifdef CONFIG_RTM_PMC
    report_rtm_performance_counters();
#endif

    aepprintf("All is well in the universe\n");
    while(1);
}

void
configure_pingpong_threads(env_t* env, sel4utils_thread_t* int_thread, 
        sel4utils_thread_t* ping_threads, sel4utils_thread_t* pong_threads)
{
    int err;
    int i;

    cspacepath_t cspace_cspath;
    vka_cspace_make_path(env->vka, 0, &cspace_cspath);

    /* Configure interrupt thread */
    // err = sel4utils_configure_thread(env->vka, env->vspace, 0, INTERRUPT_PRIORITY, seL4_CapInitThreadCNode,
    //         seL4_CapData_Guard_new(0, 32 - env->bootinfo->initThreadCNodeSizeBits), int_thread);
    // assert(!err);
    // replaced by
    sel4utils_thread_config_t config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, INTERRUPT_PRIORITY);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, int_thread);
    assert(!err);

    
    /* Configure NUM_THREADS ping pong threads */
    for (i = 0; i < NUM_THREADS; i++) {
        // config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, waiter_prio);
        // err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, wait_thread);

        // err = sel4utils_configure_thread(env->vka, env->vspace, 0, WORKER_PRIORITY, seL4_CapInitThreadCNode, 
        //         seL4_CapData_Guard_new(0, 32 - env->bootinfo->initThreadCNodeSizeBits), 
        //         &ping_threads[i]);
        // assert(!err);

        // err = sel4utils_configure_thread(env->vka, env->vspace, 0, WORKER_PRIORITY, seL4_CapInitThreadCNode, 
        //         seL4_CapData_Guard_new(0, 32 - env->bootinfo->initThreadCNodeSizeBits), 
        //         &pong_threads[i]);
        // assert(!err);
        // replaced by
        sel4utils_thread_config_t config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, WORKER_PRIORITY);
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, &ping_threads[i]);
        assert(!err);
        err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, &pong_threads[i]);
        assert(!err);
    }
}

void
start_pingpong_pair(vka_t* vka, int thread_pair_id, sel4utils_thread_t* ping_thread, 
        sel4utils_thread_t* pong_thread)
{ 
    // cspacepath_t ping_aep, pong_aep;
    // vka_object_t ping_aep_ut, pong_aep_ut;
    // replaced by
    vka_object_t ping_aep, pong_aep;
    int err;

    thread_config_t *ping_config = malloc(sizeof(thread_config_t));
    thread_config_t *pong_config = malloc(sizeof(thread_config_t));
    ping_config->ipc_buf = (void*)ping_thread->ipc_buffer_addr;
    pong_config->ipc_buf = (void*)pong_thread->ipc_buffer_addr;

    /* allocate ping and pong async endpoitns */
    // err = vka_alloc_untyped(vka, 8, &ping_aep_ut); assert(!err);
    // err = vka_alloc_untyped(vka, 8, &pong_aep_ut); assert(!err);
    // err = vka_cspace_alloc_path(vka, &ping_aep); assert(!err);
    // err = vka_cspace_alloc_path(vka, &pong_aep); assert(!err);
    // err = vka_untyped_retype(&ping_aep_ut, seL4_AsyncEndpointObject, seL4_EndpointBits, 1, &ping_aep); 
    // assert(!err);
    // err = vka_untyped_retype(&pong_aep_ut, seL4_AsyncEndpointObject, seL4_EndpointBits, 1, &pong_aep);
    // assert(!err);
    // replaced by
    err = vka_alloc_notification(vka, &ping_aep);
    assert(!err);
    err = vka_alloc_notification(vka, &pong_aep);
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
    // err = seL4_TCB_SetAffinity(ping_thread->tcb.cptr, thread_pair_id); assert(!err);
    // err = seL4_TCB_SetAffinity(pong_thread->tcb.cptr, thread_pair_id); assert(!err);
    // REPLACE with SOMETHING USING SCHED CONTEXT FOR MCS

    seL4_TCB_Resume(ping_thread->tcb.cptr);
    seL4_TCB_Resume(pong_thread->tcb.cptr);
}

#define MAX_TIMER_IRQS 4

static irq_id_t sel4test_timer_irq_register(UNUSED void *cookie, ps_irq_t irq, irq_callback_fn_t callback,
                                            void *callback_data)
{
    static int num_timer_irqs = 0;

    int error;

    ZF_LOGF_IF(!callback, "Passed in a NULL callback");

    ZF_LOGF_IF(num_timer_irqs >= MAX_TIMER_IRQS, "Trying to register too many timer IRQs");

    /* Allocate the IRQ */
    error = sel4platsupport_copy_irq_cap(ENV->vka, ENV->simple, &irq,
                                         &ENV->timer_irqs[num_timer_irqs].handler_path);
    ZF_LOGF_IF(error, "Failed to allocate IRQ handler");
    assert(!error);

    /* Allocate the root notifitcation if we haven't already done so */
    if (ENV->timer_notification.cptr == seL4_CapNull) {
        error = vka_alloc_notification(ENV->vka, &ENV->timer_notification);
        ZF_LOGF_IF(error, "Failed to allocate notification object");
        assert(!error);
    }

    /* Mint a notification for the IRQ handler to pair with */
    error = vka_cspace_alloc_path(ENV->vka, &ENV->badged_timer_notifications[num_timer_irqs]);
    ZF_LOGF_IF(error, "Failed to allocate path for the badged notification");
    assert(!error);
    cspacepath_t root_notification_path = {0};
    vka_cspace_make_path(ENV->vka, ENV->timer_notification.cptr, &root_notification_path);
    error = vka_cnode_mint(&ENV->badged_timer_notifications[num_timer_irqs], &root_notification_path,
                           seL4_AllRights, BIT(num_timer_irqs));
    ZF_LOGF_IF(error, "Failed to mint notification for timer");
    assert(!error);

    /* Pair the notification and the handler */
    error = seL4_IRQHandler_SetNotification(ENV->timer_irqs[num_timer_irqs].handler_path.capPtr,
                                            ENV->badged_timer_notifications[num_timer_irqs].capPtr);
    ZF_LOGF_IF(error, "Failed to pair the notification and handler together");
    assert(!error);

    /* Ack the handler so interrupts can come in */
    error = seL4_IRQHandler_Ack(ENV->timer_irqs[num_timer_irqs].handler_path.capPtr);
    ZF_LOGF_IF(error, "Failed to ack the IRQ handler");
    assert(!error);

    /* Fill out information about the callbacks */
    ENV->timer_cbs[num_timer_irqs].callback = callback;
    ENV->timer_cbs[num_timer_irqs].callback_data = callback_data;

    return num_timer_irqs++;
}

static void init_timer(void)
{
    int error;

    /* setup the timers and have our wrapper around simple capture the IRQ caps */
    error = ltimer_default_init(&ENV->ltimer, ENV->ops, NULL, NULL);
    ZF_LOGF_IF(error, "Failed to setup the timers");
    assert(!error);

    error = vka_alloc_notification(ENV->vka, &ENV->timer_notify_test);
    ZF_LOGF_IF(error, "Failed to allocate notification object for tests");
    assert(!error);

    // error = seL4_TCB_BindNotification(simple_get_tcb(ENV->simple), ENV->timer_notification.cptr);
    // ZF_LOGF_IF(error, "Failed to bind timer notification to sel4test-driver\n");
    // assert(!error);

    /* set up the timer manager */
    error = tm_init(&ENV->tm, &ENV->ltimer, &ENV->ops, 1);
    assert(!error);
}


void 
start_interrupt_thread(env_t* env, sel4utils_thread_t* int_thread)
{
    thread_config_t* int_thread_config;
    vka_object_t int_endpoint;
    int err;

    ENV = env;

    int_thread_config = malloc(sizeof(*int_thread_config));
    int_thread_config->ipc_buf = (void*)int_thread->ipc_buffer_addr;

    err = vka_alloc_notification(env->vka, &int_endpoint); assert(!err);

    int_thread_config->arg2 = int_endpoint.cptr;

//     err = ltimer_default_init(&env->ltimer, env->ops, NULL, NULL);
//     assert(!err);
//     err = tm_init(&env->tm, &env->ltimer, &env->ops, 1);
//     assert(!err);

    ps_irq_register_fn_t irq_register_fn_copy = env->ops.irq_ops.irq_register_fn;
    env->ops.irq_ops.irq_register_fn = sel4test_timer_irq_register;
    /* Initialise ltimer */
    init_timer();
    /* Restore the IRQ interface's register function */
    env->ops.irq_ops.irq_register_fn = irq_register_fn_copy;

    // err = tm_alloc_id_at(&env->tm, TIMER_ID);
    // assert(!err);
    // err = tm_register_periodic_cb(&env->tm, NS_IN_S, 0, TIMER_ID, NULL, 0);
    // assert(!err);

    // timeout(env, 20 * NS_IN_S, TIMEOUT_PERIODIC);

    int_thread_config->ltimer = &env->ltimer;

    // assert(int_thread_config->arg3);
    // FIX

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
        start_pingpong_pair (env->vka, i, &ping_threads[i], &pong_threads[i]);
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
