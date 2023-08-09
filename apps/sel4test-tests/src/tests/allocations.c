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
#include <sel4/benchmark_tracepoints_types.h>

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
    printf("\nCPU cycles spent: %llu\n", ipcbuffer[BENCHMARK_TCB_UTILISATION]);
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
    printf("\n >> Total Frame count: %d * 4K Pages\n\n", frame_cnt);
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
    size_t frame_cnt = BIT(15u);//test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, 12);
#endif
    if (frame_cnt > BIT(15u)) {
        frame_cnt = BIT(15u);
    }

    int err;
    int cnt = 0;
    uint64_t average = 0;
    vka_object_t tx;
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(env->tcb);
    seL4_BenchmarkResetLog();
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkDumpAllThreadsUtilisation();
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
#endif
    while (cnt < frame_cnt) {
        seL4_BenchmarkResetThreadUtilisation(env->tcb);
        seL4_BenchmarkResetLog();
        err = vka_alloc_frame(&env->vka, seL4_PageBits, &tx);
        seL4_BenchmarkFinalizeLog();
        seL4_BenchmarkGetThreadUtilisation(env->tcb);
        if (err != seL4_NoError) {
            /* ZF_LOGE("Failed to allocate frame no.[%d]", cnt); */
            break;
        } else {
            //printf("[%llu] ", ipcbuffer[BENCHMARK_TCB_UTILISATION] + ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION]);
            ;
        }
        average += (ipcbuffer[BENCHMARK_TCB_UTILISATION] + ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION]);
        cnt += 1;
    }
    printf("\navr: %llu\n", average / cnt);
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkDumpAllThreadsUtilisation();
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    return sel4test_get_result();
}

DEFINE_TEST(AXMR_ALLOC0001, "Testcase for testing frame allocations for CapBuddy", test_frame_allocation_single, true)

static int test_frame_allocation_contiguous(env_t env)
{
    /***
     * NOTICE:
     *  @param: bib -- memory region size of memory request (can be modified)
     *  @param: batch -- frame number of memory request (based on 'bib')
     *  @param: frame_cnt -- available frames (overall) to meet all requests
     */
    int bib = 10;
    int batch = BIT(bib);
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, seL4_PageBits + bib);
#endif
    //!frame_cnt = frame_cnt % batch ? frame_cnt : frame_cnt - (frame_cnt % batch);
    /* frame_cnt should be 2^n * batch */
    assert(frame_cnt % batch == 0);

    if (frame_cnt > BIT(15u)) {
        frame_cnt = BIT(15u);
    }

    int err;
    size_t cnt = 0;

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
    seL4_BenchmarkResetThreadUtilisation(env->tcb);
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
    seL4_BenchmarkGetThreadUtilisation(env->tcb);
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    return sel4test_get_result();
}

DEFINE_TEST(AXMR_ALLOC0002, "Testcase for testing frame allocations (contiguous) for CapBuddy",
            test_frame_allocation_contiguous, true)

static int test_frame_deallocation_single(env_t env)
{
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, 12);
#endif

    if (frame_cnt > BIT(15u)) {
        frame_cnt = BIT(15u);
    }

    int err;
    size_t cnt = 0;
    size_t alloc_cnt;   /* due to metadata restriction, not all frames (frame_cnt) are available */
    /***
     * We should collect the freeing materials here and they are
     * the pre-allocated frames. So one main difference between
     * alloc and dealloc testcase is we should bookkeeping their
     * metadata to free them later on.
     */
    untyped_object_t *head = (untyped_object_t *)malloc(sizeof(untyped_object_t));
    if (!head) {
        assert(0);
    }
    head = (untyped_object_t *)memset(head, 0, sizeof(untyped_object_t));
    untyped_object_t *curr = head;
    /***
     * Start our allocation here
     */
    while (cnt < frame_cnt) {
        err = vka_alloc_frame(&env->vka, seL4_PageBits, &curr->target);
        if (err != seL4_NoError) {
            /* ZF_LOGE("Failed to allocate frame no.[%d]", cnt); */
            break;
        }
        curr->next = (untyped_object_t *)malloc(sizeof(untyped_object_t));
        if (!curr->next) {
            break;
        }
        curr = curr->next;
        cnt += 1;
    }
    alloc_cnt = cnt;
    /***
     * Now it's time to free them one by one.
     */
    cnt = 0;
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(env->tcb);
    seL4_BenchmarkResetLog();
#endif
    while (cnt < alloc_cnt) {
        if (head->target.cptr) {
            vka_free_object(&env->vka, &head->target);
        } else {
            break;
        }
        curr = head;
        head = head->next;
        /* free(curr); */ // avoid it to not interfere with performance
        cnt += 1;
    }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(env->tcb);
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    return sel4test_get_result();
}

DEFINE_TEST(AXMR_FREE0001, "Testcase for testing frame deallocations for CapBuddy",
            test_frame_deallocation_single, true)

typedef struct frame_object {
    seL4_CPtr cptr;
    struct frame_object *next;
} frame_object_t;

typedef struct untyped_object_frames {
    vka_object_t target;
    struct untyped_object_frames *next;
    frame_object_t *frame_list;
} untyped_object_frames_t;

static int test_frame_deallocation_contiguous(env_t env)
{
    /***
     * NOTICE:
     *  @param: bib -- memory region size of memory request (can be modified)
     *  @param: batch -- frame number of memory request (based on 'bib')
     *  @param: frame_cnt -- available frames (overall) to meet all requests
     */
    int bib = 10;
    int batch = BIT(bib);
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, seL4_PageBits + bib);
#endif
    //!frame_cnt = frame_cnt % batch ? frame_cnt : frame_cnt - (frame_cnt % batch);
    /* frame_cnt should be 2^n * batch */
    assert(frame_cnt % batch == 0);

    if (frame_cnt > BIT(15u)) {
        frame_cnt = BIT(15u);
    }

    int err;
    size_t cnt = 0;
    size_t alloc_cnt;

    /* below is a serie of temporary varibale */
#if ENABLE_CAPBUDDY_EXTENSION
    int fn; /* frame (number) metadata for the requested memory region */
#endif
    cspacepath_t path; /* frame (cspacepath) metadata for the requested memory region */

    untyped_object_frames_t *head = (untyped_object_frames_t *)malloc(sizeof(untyped_object_frames_t));
    if (!head) {
        assert(0);
    }
    head = (untyped_object_frames_t *)memset(head, 0, sizeof(untyped_object_frames_t));
    untyped_object_frames_t *curr = head;

    frame_object_t *fx;
    frame_object_t *ft;

    while (cnt < frame_cnt) {
#if ENABLE_CAPBUDDY_EXTENSION
        err = vka_alloc_frame_contiguous(&env->vka, bib + seL4_PageBits, &fn, &curr->target);
        if (err != seL4_NoError) {
            ZF_LOGE("Failed to allocate frame no.[%d]", cnt);
            break;
        }
        curr->next = (untyped_object_frames_t *)malloc(sizeof(untyped_object_frames_t));
        if (!curr->next) {
            assert(0);
        }
        curr = curr->next;
        curr = (untyped_object_frames_t *)memset(curr, 0, sizeof(untyped_object_frames_t));
#else
    /***
     * Allocate one untyped object from allocator through vka with size: bib+12
     * We need to slice it into BIT(bib) frames through untyped_retype
     */
        err = vka_alloc_untyped(&env->vka, bib + seL4_PageBits, &curr->target);
        if (err != seL4_NoError) {
            ZF_LOGE("Failed to allocate frame no.[%d]", cnt);
            break;
        }
        /***
         * Create metadata (allocated on heap) for the frames dynamically
         */
        fx = (frame_object_t *)malloc(sizeof(frame_object_t));
        if (!fx) {
            ZF_LOGE("Failed to allocate frame metadata");
            break;
        }
        fx = (frame_object_t *)memset(fx, 0, sizeof(frame_object_t));
        ft = fx;
        /***
         * Now start to create pages for the original untyped object
         */
        for (int i = 0; i < batch; ++i) {
            err = vka_cspace_alloc(&env->vka, &ft->cptr);
            if (err != seL4_NoError) {
                ZF_LOGE("Not enough cspace slot for frame objects\n");
                assert(0);
            }
            vka_cspace_make_path(&env->vka, ft->cptr, &path);
            err = seL4_Untyped_Retype(curr->target.cptr, seL4_ARCH_4KPage, seL4_PageBits,
                                      path.root, path.dest, path.destDepth, path.offset, 1);
            if (err != seL4_NoError) {
                ZF_LOGE("Failed to retype new frames from the givin untyped");
                assert(0);
            }
            ft->next = (frame_object_t *)malloc(sizeof(frame_object_t));
            if (!ft->next) {
                assert(0);
            }
            ft = ft->next;
            ft = (frame_object_t *)memset(ft, 0, sizeof(frame_object_t));
        }

        /***
         * Store the created frame list in untyped_object_frames list, which means
         * these newly created frames belongs to the original object (curr)
         */
        curr->frame_list = fx;
        curr->next = (untyped_object_frames_t *)malloc(sizeof(untyped_object_frames_t));
        if (!curr->next) {
            break;
        }
        curr = curr->next;
        curr = (untyped_object_frames_t *)memset(curr, 0, sizeof(untyped_object_frames_t));
#endif
        cnt += batch;
    }
    alloc_cnt = cnt;
    
    cnt = 0;
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(simple_get_tcb(&env->simple));
    seL4_BenchmarkResetLog();
#endif

    while (cnt < alloc_cnt) {
#if ENABLE_CAPBUDDY_EXTENSION
        if (head->target.cptr) {
            vka_free_object(&env->vka, &head->target);
        }
#else
        if (head->target.cptr) {
            vka_cspace_make_path(&env->vka, head->target.cptr, &path);
            err = seL4_CNode_Revoke(path.root, path.capPtr, path.capDepth);
            if (err != seL4_NoError) {
                ZF_LOGE("Failed to Revoke original untyped cptr");
                assert(0);
            }
            fx = head->frame_list;
            ft = fx;
            for (int i = 0; i < batch; ++i) {
                if (ft->cptr) {
                    vka_cspace_free(&env->vka, ft->cptr);
                } else {
                    ZF_LOGE("Unable to free allocated frame capabilites");
                    assert(0);
                }
                fx = fx->next;
                free(ft); //! speed up performance
                ft = fx;
            }
            head->frame_list = NULL;
            vka_free_object(&env->vka, &head->target);
        }
#endif
        curr = head;
        head = head->next;
        free(curr); //! speed up performance
        cnt += batch;
    }

#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(simple_get_tcb(&env->simple));
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    while (true);
    return sel4test_get_result();    
}

DEFINE_TEST(AXMR_FREE0002, "Testcase for testing frame deallocations (contiguous) for CapBuddy",
            test_frame_deallocation_contiguous, true)

static int test_frame_alloc_free_volume(env_t env)
{
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, 12);
#endif
    if (frame_cnt > BIT(15u)) {
        frame_cnt = BIT(15u);
    }
    int err;
    size_t cnt = 0;
    size_t alloc_cnt;   /* due to metadata restriction, not all frames (frame_cnt) are available */
    /***
     * We should collect the freeing materials here and they are
     * the pre-allocated frames. So one main difference between
     * alloc and dealloc testcase is we should bookkeeping their
     * metadata to free them later on.
     */
    untyped_object_t *head = (untyped_object_t *)malloc(sizeof(untyped_object_t));
    if (!head) {
        assert(0);
    }
    head = (untyped_object_t *)memset(head, 0, sizeof(untyped_object_t));
    untyped_object_t *curr = head;
    /***
     * Start our allocation here
     */
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    uint64_t *ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(env->tcb);
    seL4_BenchmarkResetLog();
#endif
    while (cnt < frame_cnt) {
        err = vka_alloc_frame(&env->vka, seL4_PageBits, &curr->target);
        if (err != seL4_NoError) {
            /* ZF_LOGE("Failed to allocate frame no.[%d]", cnt); */
            break;
        }
        curr->next = (untyped_object_t *)malloc(sizeof(untyped_object_t));
        if (!curr->next) {
            break;
        }
        curr = curr->next;
        cnt += 1;
    }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(env->tcb);
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    alloc_cnt = cnt;
    /***
     * Now it's time to free them one by one.
     */
    cnt = 0;
#ifdef CONFIG_KERNEL_BENCHMARK
    printf("\n*********** Benchmark ***********\n\n");
    ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
    seL4_BenchmarkResetThreadUtilisation(env->tcb);
    seL4_BenchmarkResetLog();
#endif
    while (cnt < alloc_cnt) {
        if (head->target.cptr) {
            vka_free_object(&env->vka, &head->target);
        } else {
            break;
        }
        curr = head;
        head = head->next;
        free(curr); // avoid it to not interfere with performance
        cnt += 1;
    }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(env->tcb);
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\n=========== Benchmark ===========\n\n");
#endif
    return 0;
}

static int test_frame_alloc_free(env_t env) {
    int err;
    for (int i = 0; i < 5; ++i) {
        err = test_frame_alloc_free_volume(env);
        if (err != seL4_NoError) {
            assert(0);
        }
    }
    return sel4test_get_result();
}

DEFINE_TEST(AXMR_TBOTH0001, "Testcase for testing frame single-opertation for CapBuddy",
            test_frame_alloc_free, true)

static int test_frame_both_iteration(env_t env)
{
#if ENABLE_CAPBUDDY_EXTENSION
    size_t frame_cnt = test_collect_untypeds(&env->vka, 22);
#else
    size_t frame_cnt = test_collect_untypeds(&env->vka, 12);
#endif
    vka_object_t array[10];
    memset(array, 0, sizeof(vka_object_t));

    int err;

    for (int i = 0; i < 10; ++i) {
        err = vka_alloc_untyped(&env->vka, 22, array + i);
        if (err) {
            assert(0);
        }
    }

    vka_object_t vx;
    for (int i = 40; i >= 12; --i) {
        while (true) {
            err = vka_alloc_untyped(&env->vka, i, &vx);
            if (err != seL4_NoError) {
                break;
            }
        }
    }

    untyped_object_t *head = (untyped_object_t *)malloc(sizeof(untyped_object_t));
    if (!head) {
        assert(0);
    }
    head = (untyped_object_t *)memset(head, 0, sizeof(untyped_object_t));
    untyped_object_t *curr = head;

    uint64_t *ipcbuffer;
    int cnt = 0;
    printf("\n*********** Benchmark ***********\n\n");
    for (int i = 0; i < 10; ++i) {
        vka_free_object(&env->vka, &array[i]);
#ifdef CONFIG_KERNEL_BENCHMARK
        ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
        seL4_BenchmarkResetThreadUtilisation(env->tcb);
        seL4_BenchmarkResetLog();
#endif
        for (int j = 0; j < 1024; ++j) {
            err = vka_alloc_frame(&env->vka, seL4_PageBits, &curr->target);
            if (err != seL4_NoError) {
                /* ZF_LOGE("Failed to allocate frame no.[%d]", cnt); */
                break;
            }
            curr->next = (untyped_object_t *)malloc(sizeof(untyped_object_t));
            if (!curr->next) {
                break;
            }
            curr = curr->next;
            cnt += 1;
        }
#ifdef CONFIG_KERNEL_BENCHMARK
    seL4_BenchmarkFinalizeLog();
    seL4_BenchmarkGetThreadUtilisation(env->tcb);
    printf("\n---------- Utilisation ----------\n");
    printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
    printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
#endif
    }

    for (int i = 0; i < 10; ++i) {
    #ifdef CONFIG_KERNEL_BENCHMARK
        ipcbuffer = (uint64_t *)&(seL4_GetIPCBuffer()->msg[0]);
        seL4_BenchmarkResetThreadUtilisation(env->tcb);
        seL4_BenchmarkResetLog();
    #endif
        for (int j = 0; j < 1024; ++j) {
            if (head->target.cptr) {
                vka_free_object(&env->vka, &head->target);
            } else {
                break;
            }
            curr = head;
            head = head->next;
            free(curr); // avoid it to not interfere with performance
        }
    #ifdef CONFIG_KERNEL_BENCHMARK
        seL4_BenchmarkFinalizeLog();
        seL4_BenchmarkGetThreadUtilisation(env->tcb);
        printf("\n---------- Utilisation ----------\n");
        printf("\nAverage TCB cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_UTILISATION] / (uint64_t)cnt, cnt);
        printf("\nAverage K+T cycles per alloc: %llu, cnt: %d\n", ipcbuffer[BENCHMARK_TCB_KERNEL_UTILISATION] / (uint64_t)cnt, cnt);
    #endif
    }
    printf("\n=========== Benchmark ===========\n\n");
    return sel4test_get_result();
}

DEFINE_TEST(AXMR_TBOTH0002, "Testcase for testing frame deallocations for CapBuddy",
            test_frame_both_iteration, true)