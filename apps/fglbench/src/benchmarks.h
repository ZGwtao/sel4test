#pragma once

/* seL4 libraries */
#include <vka/vka.h>
#include <sel4utils/thread.h>

/* FGLBench includes */
#include "utils.h"

/* Ping pong defines */
#define WARMUPS 1
#define ITERATIONS 1
#define INTERRUPT_PRIORITY 200
#define WORKER_PRIORITY 100

void ipc_benchmark_0(env_t *env);
void ipc_benchmark_1(env_t *env);
void ipc_benchmark_2(env_t *env);
void notification_benchmark_0(env_t *env);
void notification_benchmark_1(env_t *env);
void notification_benchmark_2(env_t *env);
void signal_benchmark_0(env_t *env);
void signal_benchmark_1(env_t *env);
void signal_benchmark_2(env_t *env);
