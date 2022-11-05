#pragma once

/* seL4 libraries */
#include <vka/vka.h>
#include <sel4utils/thread.h>

/* AEPBench includes */
#include "utils.h"

/* Ping pong defines */
#define WARMUPS 2
#define ITERATIONS 3
#define INTERRUPT_PRIORITY 200
#define WORKER_PRIORITY 100

void 
ping_thread_fn(thread_config_t *thread_config, void *unused);

void 
ping_thread_fn(thread_config_t *thread_config, void *unused);

void 
interrupt_thread(thread_config_t *thread_config, void *unused);

void
configure_pingpong_threads(env_t* env, sel4utils_thread_t* int_thread, 
        sel4utils_thread_t* ping_threads, sel4utils_thread_t* pong_threads);

void
start_pingpong_pair(env_t* vka, int thread_pair_id, sel4utils_thread_t* ping_thread, 
        sel4utils_thread_t* pong_thread);

void 
start_interrupt_thread(env_t* env, sel4utils_thread_t* int_thread);

void
pingpong_benchmark(env_t* env);
