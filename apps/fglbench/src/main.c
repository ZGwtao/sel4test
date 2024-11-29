#include <autoconf.h>
#include <fglbench/gen_config.h>

/* C includes */
#include <stdio.h>
#include <stdlib.h>

/* seL4 libraries */
#include <sel4platsupport/platsupport.h>
#include <sel4platsupport/io.h>
#include <sel4/sel4.h>
#include <vka/vka.h>
#include <allocman/bootstrap.h>
#include <allocman/allocman.h>
#include <allocman/vka.h>
#include <sel4utils/vspace.h>
#include <sel4utils/stack.h>
#include <simple-default/simple-default.h>
#include <sel4bench/sel4bench.h>
#include <sel4platsupport/timer.h>
#include <sel4platsupport/platsupport.h>
#include <platsupport/timer.h>

/* FGLBench includes */
#include "utils.h"
#include "benchmarks.h"

#define MEM_POOL_SIZE (1 * 1024 * 256 * 64)

static char initial_mem_pool[MEM_POOL_SIZE];

env_t env_global;
vspace_t vspace_global;
simple_t simple_global;
sel4utils_alloc_data_t global_alloc_data;

void
bootstrap(env_t* env)
{
    allocman_t* allocman;
    int err;

    /* first create an allocman */
    env->bootinfo = platsupport_get_bootinfo();
    allocman = bootstrap_use_bootinfo(env->bootinfo, sizeof(initial_mem_pool), initial_mem_pool);
    assert(allocman);

    /* create a vka out of the allocman */
    env->vka = allocman_mspace_alloc(allocman, sizeof(*env->vka), &err);
    assert(!err);
    allocman_make_vka(env->vka, allocman);

    /* now we can initialize a vspace using allocman */
    err = sel4utils_bootstrap_vspace_with_bootinfo_leaky(env->vspace, &global_alloc_data,
            seL4_CapInitThreadPD, env->vka, env->bootinfo);
    assert(!err);

    /* Setup printf */
    simple_default_init_bootinfo(env->simple, env->bootinfo);
    err = platsupport_serial_setup_simple(env->vspace, env->simple, env->vka);
    assert(!err);

    err = sel4platsupport_new_io_ops(env->vspace, env->vka, env->simple, &env->ops);
    assert(!err);
}

void *
main_continued(void *arg)
{
    typedef void (*benchmark_t)(env_t *);
    benchmark_t benchmarks[] = {
#ifdef CONFIG_CORE_TAGGED_OBJECT
        proto_active_receiver,
#endif
        bm_ipc_tp,
        bm_ipc_cc_tp,
        bm_ipc_tp_nc,
        bm_ipc_tp_1c,
        bm_ipc_max_nc,
        bm_signal_tp,
        bm_signal_tp_nc,
        bm_signal_tp_1c,
        bm_nbwait_tp,
        bm_nbsend_tp,
        bm_send_and_recv_tp,
        bm_signal_and_wait_tp,
        bm_nbsendrecv_tp,
    };
    fglprintf("Running benchmark %d on %d cores\n", CONFIG_WHICH_BENCHMARK, CONFIG_NUM_CORES);
    benchmarks[CONFIG_WHICH_BENCHMARK](&env_global);

    /* We are done */
    seL4_TCB_Suspend(seL4_CapInitThreadTCB);
    return 0;
}

int
main(void)
{
    sel4bench_init();

    env_global.vspace = &vspace_global;
    env_global.simple = &simple_global;

    /* bootstrap into environment */
    bootstrap(&env_global);
    fglprintf("Bootstrapped into environment.\n");

    sel4utils_run_on_stack(env_global.vspace, main_continued, NULL, NULL);
}
