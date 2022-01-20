/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <autoconf.h>
#include <sel4/sel4.h>
#include "timer.h"
#include <utils/util.h>

struct sel4test_ack_data {
    env_t *env;
    int nth_timer;
};
typedef struct sel4test_ack_data sel4test_ack_data_t;

/* A pending timeout requests from tests */
static bool timeServer_timeoutPending = false;
static timeout_type_t timeServer_timeoutType;

static int timeout_cb(uintptr_t token)
{
    seL4_Signal((seL4_CPtr) token);

    if (timeServer_timeoutType != TIMEOUT_PERIODIC) {
        timeServer_timeoutPending = false;
    }
    return 0;
}

static int ack_timer_interrupts(void *ack_data)
{
    ZF_LOGF_IF(!ack_data, "ack_data is NULL");
    sel4test_ack_data_t *timer_ack_data = (sel4test_ack_data_t *) ack_data;

    env_t *env = timer_ack_data->env;
    int nth_timer = timer_ack_data->nth_timer;

    /* Acknowledge the interrupt handler */
    int error = seL4_IRQHandler_Ack(env->timer_irqs[nth_timer].handler_path.capPtr);
    ZF_LOGF_IF(error, "Failed to acknowledge timer IRQ handler");

    ps_free(&env->ops.malloc_ops, sizeof(sel4test_ack_data_t), ack_data);
    return error;
}

void handle_timer_interrupts(env_t *env, seL4_Word badge)
{
    int error = 0;
    while (badge) {
        seL4_Word badge_bit = CTZL(badge);
        sel4test_ack_data_t *ack_data = NULL;
        error = ps_calloc(&env->ops.malloc_ops, 1, sizeof(sel4test_ack_data_t), (void **) &ack_data);
        ZF_LOGF_IF(error, "Failed to allocate memory for ack token");
        ack_data->env = env;
        ack_data->nth_timer = (int) badge_bit;
        env->timer_cbs[badge_bit].callback(env->timer_cbs[badge_bit].callback_data,
                                           ack_timer_interrupts, ack_data);
        badge &= ~BIT(badge_bit);
    }
}

void wait_for_timer_interrupt(env_t *env)
{
    seL4_Word sender_badge;
    seL4_Wait(env->timer_notification.cptr, &sender_badge);
    if (sender_badge) {
        printf("Handling timer interrupgs\n");
        handle_timer_interrupts(env, sender_badge);
    }
}

void timeout(env_t *env, uint64_t ns, timeout_type_t timeout_type)
{
    ZF_LOGD_IF(timeServer_timeoutPending, "Overwriting a previous timeout request\n");
    timeServer_timeoutType = timeout_type;
    int error = tm_register_cb(&env->tm, timeout_type, ns, 0,
                            TIMER_ID, timeout_cb, env->timer_notify_test.cptr);
    printf("error = %d, ETIME = %d\n", error, ETIME);
    if (error == ETIME) {
        error = timeout_cb(env->timer_notify_test.cptr);
        printf("error = %d\n", error);
    } else {
        timeServer_timeoutPending = true;
    }
    ZF_LOGF_IF(error != 0, "register_cb failed");
}

void timer_reset(env_t *env)
{
    int error = tm_deregister_cb(&env->tm, TIMER_ID);
    ZF_LOGF_IF(error, "ltimer_rest failed");
    timeServer_timeoutPending = false;
}

uint64_t timestamp(env_t *env)
{
    uint64_t time = 0;
    int error = ltimer_get_time(&env->ltimer, &time);
    ZF_LOGF_IF(error, "failed to get time");
    return time;
}

void timer_cleanup(env_t *env)
{
    tm_free_id(&env->tm, TIMER_ID);
    timeServer_timeoutPending = false;
}
