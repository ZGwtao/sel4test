#pragma once

/* seL4 libraries */
#include <vka/vka.h>
#include <sel4utils/thread.h>

/* FGLBench includes */
#include "utils.h"

/* Ping pong defines */
#define WARMUPS 1
#define ITERATIONS 1
#define MAX_ET_RUNTIME_MINS 20
#define INTERRUPT_PRIORITY 200
#define WORKER_PRIORITY 100

void bm_ipc_tp(env_t *env);
void bm_ipc_tp_nc(env_t *env);
void bm_ipc_tp_1c(env_t *env);
void bm_ipc_max_nc(env_t *env);
void bm_ntfn_tp(env_t *env);
void bm_ntfn_tp_nc(env_t *env);
void bm_ntfn_tp_1c(env_t *env);
void bm_signal_tp(env_t *env);
void bm_signal_tp_nc(env_t *env);
void bm_signal_tp_1c(env_t *env);
void bm_nbwait_tp(env_t *env);
void bm_nbsend_tp(env_t *env);
void bm_send_and_recv_tp(env_t *env);
void bm_signal_and_wait_tp(env_t *env);
void bm_nbsendrecv_tp(env_t *env);
