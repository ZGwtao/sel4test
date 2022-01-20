#include <stdio.h>
#include <stdlib.h>
// #include <sel4platsupport/plat/epit2.h>
// #include <sel4platsupport/arch/epit2.h>
#include <sel4platsupport/platsupport.h>
#include <sel4/sel4.h>
#include <vka/vka.h>
#include <allocman/bootstrap.h>
#include <allocman/allocman.h>
#include <allocman/vka.h>
#include <sel4utils/vspace.h>
#include <sel4utils/thread.h>

#define NUM_THREADS 16

#define WARMUPS 3

#define ITERATIONS 16
#define WORKUNITS 16

typedef struct thread_config {
    void *ipc_buf;
    uint32_t arg1;
    uint32_t arg2;
    uint32_t arg3;
} thread_config_t;

typedef struct counter {
    uint32_t val;
    /* force 64byte alignment */
    uint32_t padding[15];
} counter_t;

typedef struct bench_counters {
    counter_t counters[NUM_THREADS];
} bench_counters_t;

#define INTERRUPT_PRIORITY 200
#define WORKER_PRIORITY 100

void thread1_main(void *blah) {
    printf("hello world\n");
    while(1);
}

static inline void set_ipc_buffer(thread_config_t *config) {
    asm volatile("mcr p15, 0, %0, c13, c0, 2" :: "r"(config->ipc_buf) : "memory");
}

static inline void do_work() {
    int i;
    for (i = 0; i < 1 << WORKUNITS; i++) {
        asm volatile("nop");
    }
}

void ping_thread(thread_config_t *thread_config, volatile bench_counters_t *counters) {
    /* arg1 is endpoint to use
     * arg2 is thread id */
    set_ipc_buffer(thread_config);
    while(1) {
        // __seL4_CallWithMRs(thread_config->arg1, seL4_MessageInfo_new(0, 0, 0, 0), NULL, NULL, NULL, NULL);
        // replaced with
        seL4_CallWithMRs(thread_config->arg1, seL4_MessageInfo_new(0, 0, 0, 0), NULL, NULL, NULL, NULL);
        counters->counters[thread_config->arg2].val++;
        do_work();
    }
}

void pong_thread(thread_config_t *thread_config, bench_counters_t *counters) {
    /* arg1 is endpoint to use
     * arg2 is thread id */
    set_ipc_buffer(thread_config);
    seL4_Wait(thread_config->arg1, NULL);
    while(1) {
        // __seL4_ReplyWaitWithMRs(thread_config->arg1, seL4_MessageInfo_new(0, 0, 0, 0), NULL, NULL, NULL, NULL, NULL);
        // replaced with
        seL4_ReplyRecvWithMRs(thread_config->arg1, seL4_MessageInfo_new(0, 0, 0, 0), NULL, NULL, NULL, NULL, NULL);
        counters->counters[thread_config->arg2].val++;
        do_work();
    }
}

#define NS_IN_SECOND 1000000000

void interrupt_thread(thread_config_t *thread_config, volatile bench_counters_t *counters) {
    // epit2_interface_t epit_int;
    // int i;
    // int it;
    // /* arg1 is an empty slot
    //  * arg2 is async endpoint
    //  * arg3 is vaddr of timer */
    // set_ipc_buffer(thread_config);
    // epit_int = platsupport_get_epit2();
    // epit_int.init_timer( (epit2_config_data_t) {
    //     .cspace_root = seL4_CapInitThreadCNode,
    //     .irq_cap = thread_config->arg1,
    //     .aep_cap = thread_config->arg2,
    //     .vaddr = thread_config->arg3,
    //     .vspace_root = seL4_CapInitThreadPD,
    //     .irq_control = seL4_CapIRQControl,
    //     .ignore_objects = 0
    // });
    // epit_int.configure_timer_periodic(NS_IN_SECOND);
    // for (it = 0; it < ITERATIONS; it ++) {
    //     uint32_t last_value[NUM_THREADS];
    //     uint32_t current_value[NUM_THREADS];
    //     uint32_t total = 0;
    //     for (i = 0; i < WARMUPS; i++ ) {
    //         epit_int.wait_for_timer_interrupt();
    //         epit_int.ack_irq();
    //     }
    //     for (i = 0; i < NUM_THREADS; i++) {
    //         last_value[i] = counters->counters[i].val;
    //     }
    //     epit_int.wait_for_timer_interrupt();
    //     epit_int.ack_irq();
    //     for (i = 0; i < NUM_THREADS; i++) {
    //         current_value[i] = counters->counters[i].val;
    //     }
    //     printf("###");
    //     for (i = 0; i < NUM_THREADS; i++) {
    //         uint32_t diff;
    //         /* thanks to unsigned int arithmetic this will work even in the presence of overflow */
    //         diff = current_value[i] - last_value[i];
    //         printf("%d,", diff);
    //         last_value[i] = current_value[i];
    //         total += diff;
    //     }
    //     printf("%d\n", total);
    // }
    printf("All is well in the universe.\n");
    while(1);
}

#define MEM_POOL_SIZE (1*1024*1024)

static char initial_mem_pool[MEM_POOL_SIZE];

// static inline void *map_device(vspace_t *vspace, seL4_BootInfo *bootinfo, seL4_Word device_paddr) {
    // int i;
    // for (i = 0; i < bootinfo->numDeviceRegions; i++) {
    //     if (bootinfo->deviceRegions[i].basePaddr == device_paddr) {
    //         int num_frames = bootinfo->deviceRegions[i].frames.end - bootinfo->deviceRegions[i].frames.start;
    //         int j;
    //         seL4_CPtr frames[num_frames];
    //         for (j = 0; j < num_frames; j++ ){
    //             frames[j] = bootinfo->deviceRegions[i].frames.start + j;
    //         }
    //         // return vspace_map_pages(vspace, frames, seL4_AllRights, num_frames, bootinfo->deviceRegions[i].frameSizeBits, 0);
    //         // replaced with
    //         return vspace_map_pages(vspace, frames, NULL, seL4_AllRights, num_frames, bootinfo->deviceRegions[i].frameSizeBits, 0);
    //     }
    // }
//     return NULL;
// }

bench_counters_t *allocate_counters(void) {
    uint32_t result = (uint32_t)malloc(sizeof(bench_counters_t) + 64);
    if (!result) {
        return NULL;
    }
    /* make sure bench_counters is 64byte aligned */
    result = (result & ~0xF) + 0x10;
    return (bench_counters_t *)result;
}

int main(void) {
    seL4_BootInfo *bootinfo;
    vka_t *vka;
    allocman_t *allocman;
    vspace_t vspace;
    int error;
    int i;
    bench_counters_t *counters;

    sel4utils_thread_t int_thread;
    thread_config_t *int_thread_config;
    vka_object_t int_endpoint;

    sel4utils_thread_t ping_threads[NUM_THREADS];
    sel4utils_thread_t pong_threads[NUM_THREADS];

    bootinfo = seL4_GetBootInfo();
    allocman = bootstrap_use_bootinfo(bootinfo, sizeof(initial_mem_pool), initial_mem_pool);
    assert(allocman);
    vka = allocman_mspace_alloc(allocman, sizeof(*vka), &error);
    assert(!error);
    allocman_make_vka(vka, allocman);

    error = sel4utils_bootstrap_vspace_leaky(&vspace, seL4_CapInitThreadPD, vka, bootinfo);
    assert(!error);
    platsupport_serial_setup_bootinfo(&vspace, bootinfo);

    error = sel4utils_configure_thread(vka, &vspace, 0, INTERRUPT_PRIORITY, seL4_CapInitThreadCNode, seL4_CapData_Guard_new(0, 32 - bootinfo->initThreadCNodeSizeBits), &int_thread);
    assert(!error);
    for (i = 0; i < NUM_THREADS; i++) {
        error = sel4utils_configure_thread(vka, &vspace, 0, WORKER_PRIORITY, seL4_CapInitThreadCNode, seL4_CapData_Guard_new(0, 32 - bootinfo->initThreadCNodeSizeBits), &ping_threads[i]);
        assert(!error);
        error = sel4utils_configure_thread(vka, &vspace, 0, WORKER_PRIORITY, seL4_CapInitThreadCNode, seL4_CapData_Guard_new(0, 32 - bootinfo->initThreadCNodeSizeBits), &pong_threads[i]);
        assert(!error);
    }

    counters = allocate_counters();
    assert(counters);

    for (i = 0; i < NUM_THREADS; i++ ){
        vka_object_t endpoint;
        thread_config_t *ping_config = malloc(sizeof(thread_config_t));
        thread_config_t *pong_config = malloc(sizeof(thread_config_t));
        ping_config->ipc_buf = (void*)ping_threads[i].ipc_buffer_addr;
        pong_config->ipc_buf = (void*)pong_threads[i].ipc_buffer_addr;
        error = vka_alloc_endpoint(vka, &endpoint);
        assert(!error);
        ping_config->arg1 = endpoint.cptr;
        pong_config->arg1 = endpoint.cptr;
        ping_config->arg2 = i;
        pong_config->arg2 = i;
        error = sel4utils_start_thread(&ping_threads[i], (void*)ping_thread, ping_config, counters, 1);
        assert(!error);
        error = sel4utils_start_thread(&pong_threads[i], (void*)pong_thread, pong_config, counters, 1);
        assert(!error);
    }

    int_thread_config = malloc(sizeof(*int_thread_config));
    int_thread_config->ipc_buf = (void*)int_thread.ipc_buffer_addr;
    error = vka_cspace_alloc(vka, &int_thread_config->arg1);
    assert(!error);
    error = vka_alloc_async_endpoint(vka, &int_endpoint);
    assert(!error);
    int_thread_config->arg2 = int_endpoint.cptr;
    // int_thread_config->arg3 = (uint32_t)map_device(&vspace, bootinfo, EPIT2_DEVICE_PADDR);
    assert(int_thread_config->arg3);
    error = sel4utils_start_thread(&int_thread, (void*)interrupt_thread, int_thread_config, counters, 1);
    assert(!error);
    seL4_TCB_Suspend(seL4_CapInitThreadTCB);
    while(1);
    return 0;
}
