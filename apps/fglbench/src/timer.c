#include <autoconf.h>
#include <sel4/sel4.h>
#include "timer.h"
#include <utils/util.h>

struct sel4test_ack_data {
    env_t *env;
    int nth_timer;
};
typedef struct sel4test_ack_data sel4test_ack_data_t;

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
        handle_timer_interrupts(env, sender_badge);
    }
}
