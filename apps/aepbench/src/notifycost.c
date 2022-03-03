/* C includes */
#include <stdlib.h>
#include <stdio.h>

/* seL4 libraries */
#include <sel4/sel4.h>
#include <vka/vka.h>
#include <vka/capops.h>
#include <sel4utils/vspace.h>
#include <sel4utils/thread.h>
#include <simple-default/simple-default.h>
#include <sel4bench/sel4bench.h>

/* AEPBench includes */
#include "notifycost.h"

#define NOTIFY_WARMUPS 10
#define NOTIFY_RUNS    10

int 
notify_fn(thread_config_t *thread_config, void *unused)
{
    int i;
    unsigned int start;
    unsigned int end;

    for (i = 0; i < NOTIFY_WARMUPS + 1; ++i) {
        /* If the waiter thread has the same priority we want it to be waiting before we notify */
        seL4_Yield();
        seL4_Yield();

        /* Get the actual measurement */
        start = sel4bench_get_cycle_count();
        /* seL4_Signal(thread_config->arg1); */
        // seL4_Yield();
        /* seL4_NBSend(thread_config->arg3, seL4_MessageInfo_new(0, 0, 0, 0)); */
        seL4_Call(thread_config->arg3, seL4_MessageInfo_new(0, 0, 0, 0));
        /* seL4_Send(thread_config->arg3, seL4_MessageInfo_new(0, 0, 0, 0)); */
        // seL4_BenchmarkNullSyscall();
        end = sel4bench_get_cycle_count();
    }

    seL4_Send(thread_config->arg3, seL4_MessageInfo_new(0, 0, 0, 0));

    /* Send back the result */
    seL4_SetMR(0, end - start);
    seL4_Send(thread_config->arg2, seL4_MessageInfo_new(0, 0, 0, 1));

    // return end - start;

    while(1);
}

void 
wait_fn(thread_config_t *thread_config, void *unused)
{
    int i;

    seL4_Recv(thread_config->arg3, NULL);
    
    for (i = 0; i < NOTIFY_WARMUPS + 1; ++i) {
        /* seL4_Wait(thread_config->arg1, NULL); */
        /* seL4_Recv(thread_config->arg3, NULL); */
        seL4_ReplyRecv(thread_config->arg3, seL4_MessageInfo_new(0, 0, 0, 0), NULL);
    }
    
    
    while(1);
}

void
analyze_syscall_results(int* idle_cycles, int* waiting_cycles)
{
    float idle_mean, waiting_mean;
    float idle_stddev, waiting_stddev;
    int i;

    idle_mean = calc_mean(idle_cycles, NOTIFY_RUNS);
    waiting_mean = calc_mean(waiting_cycles, NOTIFY_RUNS); 

    idle_stddev = calc_stddev(idle_cycles, NOTIFY_RUNS, idle_mean);
    waiting_stddev = calc_stddev(waiting_cycles, NOTIFY_RUNS, waiting_mean);

    aepprintf("Idle\tWaiting\n");
    for(i = 0; i < NOTIFY_RUNS; ++i) {
        aepprintf("%u\t%u\n", idle_cycles[i], waiting_cycles[i]);
    }
    aepprintf("idle\t\tmean: %f, stddev: %f\n", idle_mean, idle_stddev);
    aepprintf("waiting\tmean: %f, stddev: %f\n", waiting_mean, waiting_stddev);
}

void
configure_notifier_wait_threads(env_t* env, sel4utils_thread_t* notifier_thread, 
        sel4utils_thread_t* wait_thread, int notifier_prio, int waiter_prio)
{
    int err;

    D("Configuring notifier and waiter threads\n");

    // err = sel4utils_configure_thread(env->vka, env->vspace, 0, notifier_prio, seL4_CapInitThreadCNode, 
    //         seL4_CapData_Guard_new(0, 32 - env->bootinfo->initThreadCNodeSizeBits), 
    //         notifier_thread);
    // assert(!err);

    // err = sel4utils_configure_thread(env->vka, env->vspace, 0, waiter_prio, seL4_CapInitThreadCNode, 
    //         seL4_CapData_Guard_new(0, 32 - env->bootinfo->initThreadCNodeSizeBits), 
    //         wait_thread);
    // assert(!err);
    // replaced by
    cspacepath_t cspace_cspath;
    vka_cspace_make_path(env->vka, 0, &cspace_cspath);

    sel4utils_thread_config_t config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, notifier_prio);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, notifier_thread);

    config = thread_config_default(env->simple, cspace_cspath.root, seL4_NilData, 0, waiter_prio);
    err = sel4utils_configure_thread_config(env->vka, env->vspace, env->vspace, config, wait_thread);
}

int
run_notifier_wait_threads (vka_t* vka, sel4utils_thread_t* notify_thread, 
        sel4utils_thread_t* wait_thread)
{
    // cspacepath_t aep;
    // replaced by
    vka_object_t aep;
    vka_object_t result_ep;
    vka_object_t aep_ut;
    int err;

    D("Running notifier and wait threads\n");

    thread_config_t *notify_config = malloc(sizeof(thread_config_t));
    thread_config_t *wait_config = malloc(sizeof(thread_config_t));

    notify_config->ipc_buf = (void*)notify_thread->ipc_buffer_addr;
    wait_config->ipc_buf = (void*)wait_thread->ipc_buffer_addr;

    /* Allocate AEP. Slightly more effort getting it cache aligned for RTM optimisation */
    // err = vka_alloc_untyped(vka, 8, &aep_ut);
    // assert(!err);
    // err = vka_cspace_alloc_path(vka, &aep); 
    // assert(!err);
    // err = vka_untyped_retype(&aep_ut, seL4_NotificationObject, seL4_EndpointBits, 1, &aep); 
    // assert(!err);
    // replaced by
    // err = vka_alloc_notification(vka, &aep);
    err = vka_alloc_endpoint(vka, &aep);
    assert(!err);

    /* Allocate endpoint receive results */
    err = vka_alloc_endpoint(vka, &result_ep);
    assert(!err);
    // err = vka_alloc_object(vka, seL4_EndpointObject, CACHE_LINE_BYTES * 8, &result_ep);
    // replaced by
    // err = vka_alloc_object(vka, seL4_EndpointObject, 64 * 8, &result_ep);
    // assert(!err);

    // /* Assign AEP and EP to thread configs */
    notify_config->arg1 = aep.cptr;
    wait_config->arg1 = aep.cptr;
    notify_config->arg2 = result_ep.cptr;
    wait_config->arg2 = result_ep.cptr;
    notify_config->arg3 = (seL4_Word)vka_alloc_endpoint_leaky(vka);
    wait_config->arg3 = notify_config->arg3;

    // /* Start threads */
    /* err = sel4utils_start_thread(notify_thread, (void*)notify_fn, notify_config, NULL, 0);  */
    /* assert(!err); */
    err = sel4utils_start_thread(wait_thread, (void*)wait_fn, wait_config, NULL, 0); 
    assert(!err);

    // /* Both threads on the same core */
    // err = seL4_TCB_SetAffinity(notify_thread->tcb.cptr, 0); assert(!err);
    /* err = seL4_TCB_SetAffinity(wait_thread->tcb.cptr, 0); assert(!err); */
    // replaced by nothing
    // on MCS, the sel4utils thread wrapper runs the threads on core 0

    // seL4_TCB_Resume(notify_thread->tcb.cptr);
    seL4_TCB_Resume(wait_thread->tcb.cptr);

    seL4_Wait(result_ep.cptr, NULL);

    return seL4_GetMR(0);
    // return notify_fn(notify_config, NULL);
}

void
cleanup_notifier_wait_threads(vka_t* vka, vspace_t* vspace, sel4utils_thread_t* notifier_thread,
        sel4utils_thread_t* wait_thread)
{
    D("Cleaning up notifier and wait threads\n");

    sel4utils_clean_up_thread(vka, vspace, notifier_thread);
    sel4utils_clean_up_thread(vka, vspace, wait_thread);
}

void 
syscall_cost(env_t* env)
{
    sel4utils_thread_t notifier_thread;
    sel4utils_thread_t wait_thread;
    int i;
    int idle_cycles [NOTIFY_RUNS];
    int wait_cycles [NOTIFY_RUNS];

    aepprintf("Measuring seL4_Notify cycle cost\n");
    aepprintf("SIZE OF WORD = %d\n", sizeof (seL4_Word));

    for (i = 0; i < NOTIFY_RUNS; ++i) {
        /* Destination thread idle upon notify */
        configure_notifier_wait_threads(env, &notifier_thread, &wait_thread, 100, 1);
        idle_cycles[i] = run_notifier_wait_threads (env->vka, &notifier_thread, &wait_thread);
        cleanup_notifier_wait_threads(env->vka, env->vspace, &notifier_thread, &wait_thread);

        /* Destination thread waiting upon notify */
        configure_notifier_wait_threads(env, &notifier_thread, &wait_thread, 100, 100);
        wait_cycles[i] = run_notifier_wait_threads (env->vka, &notifier_thread, &wait_thread);
        cleanup_notifier_wait_threads(env->vka, env->vspace, &notifier_thread, &wait_thread);
    }

    analyze_syscall_results(idle_cycles, wait_cycles);
}
