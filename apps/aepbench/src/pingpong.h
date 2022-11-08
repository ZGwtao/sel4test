#pragma once

/* seL4 libraries */
#include <vka/vka.h>
#include <sel4utils/thread.h>

/* AEPBench includes */
#include "utils.h"

/* Ping pong defines */
#define WARMUPS 0
#define ITERATIONS 3
#define INTERRUPT_PRIORITY 200
#define WORKER_PRIORITY 100

void
pingpong_benchmark(env_t* env);
