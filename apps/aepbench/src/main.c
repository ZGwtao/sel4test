#include <autoconf.h>
#include <aepbench/gen_config.h>

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

/* AEPBench includes */
#include "utils.h"
#include "notifycost.h"
#include "pingpong.h"

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
    /* seL4_Word counters = sel4bench_get_num_counters(); */
    /* for (seL4_Word i = 0; i < 64; i++) { */
    /*     printf("i=%lu : ", i); */

    /*     // if (counters & i) { */
    /*         const char *description = sel4bench_get_counter_description(i); */
    /*         printf("%s", description); */
    /*     // } */
    /*     printf("\n"); */
    /* } */

#ifndef CONFIG_PINGPONG_ONLY
    /* syscall_cost(&env_global); */
#ifdef CONFIG_SYSCALL_ONLY
    aepprintf("All is well in the universe\n");
#else
    printf("==================================================================================\n");
#endif
#endif

#ifndef CONFIG_SYSCALL_ONLY
    /* This technically should never return */
    pingpong_benchmark(&env_global);
#endif

    /* We are done */
    seL4_TCB_Suspend(seL4_CapInitThreadTCB);
    return 0;
}

int
main(void)
{
    sel4bench_init();

    // for (;;) seL4_BenchmarkNullSyscall();

    env_global.vspace = &vspace_global;
    env_global.simple = &simple_global;

    /* bootstrap into environment */
    bootstrap(&env_global);
    aepprintf("Bootstrapped into environment.\n");

    sel4utils_run_on_stack(env_global.vspace, main_continued, NULL, NULL);
}
