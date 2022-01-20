/* Author: Sean Peters */
/* Date: 24 May 2014 */
/* Measures the raw syscall cost of seL4_Notify for the various fastpath cases. */

#ifndef NOTIFYCOST_H
#define NOTIFYCOST_H

/* seL4 libraries */
#include <vka/vka.h>
#include <sel4utils/vspace.h>
#include <sel4utils/thread.h>

/* AEPBench includes */
#include "utils.h"

void 
notify_fn(thread_config_t *thread_config, void *unused);

void 
wait_fn(thread_config_t *thread_config, void *unused);

void
analyze_syscall_results(int* idle_cycles, int* waiting_cycles);

void
configure_notifier_wait_threads(env_t* env, sel4utils_thread_t* notifier_thread, 
        sel4utils_thread_t* wait_thread, int notifier_prio, int waiter_prio);

int
run_notifier_wait_threads (vka_t* vka, sel4utils_thread_t* notify_thread, 
        sel4utils_thread_t* wait_thread);

void
cleanup_notifier_wait_threads(vka_t* vka, vspace_t* vspace, sel4utils_thread_t* notifier_thread,
        sel4utils_thread_t* wait_thread);

void 
syscall_cost(env_t* env);

void
syscall_cost(env_t* env);

#endif /* NOTIFYCOST_H */
