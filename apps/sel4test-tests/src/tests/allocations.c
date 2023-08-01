/***
 * Copyright 2023, zhuguangtao@iie.ac.cn, SKLOIS
 * ----------------------------------------------
 * Testcases for measuring performance of memory
 * requests under cascading split-based Buddy and
 * CapBuddy.
 * ----------------------------------------------
 */

#include <autoconf.h>
#include <sel4vka/gen_config.h>
#include <sel4allocman/gen_config.h>
#include <assert.h>
#include <stdio.h>
#include <sel4/sel4.h>
#include <vka/object.h>
#include <sel4/benchmark_utilisation_types.h>

#include "../helpers.h"

typedef struct untyped_object {
    /***
     * @param: target: support vka request interface
     */
    vka_object_t target;
    /***
     * @param: next: support list structure
     */
    struct untyped_object *next;
} untyped_object_t;

#ifdef CONFIG_KERNEL_BENCHMARK
/* Benchmarking function hook */
static size_t test_benchmark_cycle_cnt(env_t env, size_t (*func)(vka_t *vka))
{
    size_t r;
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(simple_get_tcb(&env->simple));
    seL4_BenchmarkResetLog();
    r = func(&env->vka);
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(simple_get_tcb(&env->simple));
    printf("\n---------- Utilisation ----------\n");
    printf("\nCPU cycles spent: %ld\n", ipcbuffer[BENCHMARK_TCB_UTILISATION]);
    printf("\n=========== Benchmark ===========\n");
    return r;
}
#endif

/* figure out if it's fine to use CapBuddy support */
#ifndef ENABLE_CAPBUDDY_EXTENSION
#define ENABLE_CAPBUDDY_EXTENSION \
    (config_set(CONFIG_LIB_VKA_ALLOW_POOL_OPERATIONS) && \
     config_set(CONFIG_LIB_ALLOCMAN_ALLOW_POOL_OPERATIONS))
#endif

static size_t test_collect_untypeds(vka_t *vka, int bot)
{
    int err;
    size_t frame_cnt = 0;
    vka_object_t tx;
    /***
     * Let's assume we will only utilize untypeds with size smaller
     * than 4G (size_t=32 they are), so if unfortunatly vka provides
     * us with them, just free it through vka's freeing interface.
     * NOTICE:
     *  We should not provide device-based memory untypeds for these
     * testcase. Well it's good to see there is no untyped with device
     * attribute provided here.
     */
    for (int i = 40; i > 32; --i) {
        while (err == seL4_NoError) {
            err = vka_alloc_untyped(vka, i, &tx);
            if (err) {
                break;
            }
        }
        err = seL4_NoError;
    }
    printf("\n >> Cleanup super large untypeds\n");
    /* After cleaning up large untypeds, count untypeds (size >= 4M) */
    printf(" >> Count medium-levels untypeds\n\n");

    untyped_object_t *head = NULL;
    head = (untyped_object_t *)malloc(sizeof(*head));
    if (!head) {
        assert(0);
    }
    head = (untyped_object_t *)memset(head, 0, sizeof(*head));
    untyped_object_t *curr = head;

    for (int i = 32; i >= bot; --i) {
        while (err == seL4_NoError) {
            err = vka_alloc_untyped(vka, i, &curr->target);
            if (err) {
                break;
            }
            curr->next = (untyped_object_t *)malloc(sizeof(untyped_object_t));
            if (!curr->next) {
                ZF_LOGF("[ALLOCTEST] Failed to allocate space for untyped user-level metadata");
                break;
            }
            curr->next = (untyped_object_t *)memset(curr->next, 0, sizeof(untyped_object_t));
            curr = curr->next;
            frame_cnt += BIT(i-(seL4_PageBits));
        }
        err = seL4_NoError;
    }

    while (head) {
        if (head->target.cptr) {
            vka_free_object(vka, &head->target);
        }
        curr = head->next;
        free(head);
        head = curr;
    }
    printf("\n >> Total Frame count: %ld * 4K Pages\n\n", frame_cnt);
    return frame_cnt;
}

static int test_frame_allocation_single(env_t env)
{
    /***
     * It's okay if we only request memory without freeing them later on,
     * this's because seL4test-driver will invoke 'Revoke' operations on
     * the untypeds' capabilities that were givin to the seL4test-clients.
     */
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, 12);
#endif
    int err;
    int cnt = 0;
    vka_object_t tx;

#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(simple_get_tcb(&env->simple));
    seL4_BenchmarkResetLog();
#endif
    while (cnt < frame_cnt) {
        err = vka_alloc_frame(&env->vka, seL4_PageBits, &tx);
        if (err != seL4_NoError) {
            /* ZF_LOGE("Failed to allocate frame no.[%d]", cnt); */
            break;
        }
        cnt += 1;
    }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(simple_get_tcb(&env->simple));
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage cycles per alloc: %ld\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / frame_cnt);
    printf("\nCPU cycles spent: %ld\n\n", ipcbuffer[BENCHMARK_TCB_UTILISATION]);
    printf("\n=========== Benchmark ===========\n");
#endif
    return sel4test_get_result();
}

DEFINE_TEST(ALLOC0001, "Testcase for testing frame allocations for CapBuddy", test_frame_allocation_single, true)

static int test_frame_allocation_contiguous(env_t env)
{
    /***
     * NOTICE:
     *  @param: bib -- memory region size of memory request (can be modified)
     *  @param: batch -- frame number of memory request (based on 'bib')
     *  @param: frame_cnt -- available frames (overall) to meet all requests
     */
    int bib = 6;
    int batch = BIT(bib);
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, seL4_PageBits + bib);
#endif
    //!frame_cnt = frame_cnt % batch ? frame_cnt : frame_cnt - (frame_cnt % batch);
    /* frame_cnt should be 2^n * batch */
    assert(frame_cnt % batch == 0);

    int err;
    int cnt = 0;

    /* below is a serie of temporary varibale */
    /* no need to free -> no need to do the bookkeeping */
#if ENABLE_CAPBUDDY_EXTENSION
    int fn; /* frame (number) metadata for the requested memory region */
#endif
    seL4_CPtr fptr;     /* frame (cptr) metadata for the requested memory region */
    cspacepath_t fpath; /* frame (cspacepath) metadata for the requested memory region */
    vka_object_t tx;    /* target untyped metadata for the requested memory region */
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(simple_get_tcb(&env->simple));
    seL4_BenchmarkResetLog();
#endif
    while (cnt < frame_cnt) {
#if ENABLE_CAPBUDDY_EXTENSION
        err = vka_alloc_frame_contiguous(&env->vka, bib + seL4_PageBits, &fn, &tx);
        if (err != seL4_NoError) {
            ZF_LOGE("Failed to allocate frame no.[%d]", cnt);
            break;
        }
#else
        err = vka_alloc_untyped(&env->vka, bib + seL4_PageBits, &tx);
        if (err != seL4_NoError) {
            ZF_LOGE("Failed to allocate frame no.[%d]", cnt);
            break;
        }
        for (int i = 0; i < batch; ++i) {
            err = vka_cspace_alloc(&env->vka, &fptr);
            if (err != seL4_NoError) {
                ZF_LOGE("Not enough cspace slot for frame objects\n");
                assert(0);
            }
            vka_cspace_make_path(&env->vka, fptr, &fpath);
            err = seL4_Untyped_Retype(tx.cptr, seL4_ARCH_4KPage, seL4_PageBits,
                                      fpath.root, fpath.dest, fpath.destDepth, fpath.offset, 1);
            if (err != seL4_NoError) {
                ZF_LOGE("Failed to retype new frames from the givin untyped");
                assert(0);
            }
        }
#endif
        cnt += batch;
    }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(simple_get_tcb(&env->simple));
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage cycles per alloc: %ld\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / cnt);
    printf("\nCPU cycles spent: %ld\n\n", ipcbuffer[BENCHMARK_TCB_UTILISATION]);
    printf("\n=========== Benchmark ===========\n");
#endif
    return sel4test_get_result();
}

DEFINE_TEST(ALLOC0002, "Testcase for testing frame allocations (contiguous) for CapBuddy",
            test_frame_allocation_contiguous, true)