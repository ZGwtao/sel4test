/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "utils.h"

#define TIMER_ID 0

/* Timing related functions used only by in sel4test-driver */
void handle_timer_interrupts(env_t *env, seL4_Word badge);
void wait_for_timer_interrupt(env_t *env);
void timeout(env_t *env, uint64_t ns, timeout_type_t timeout);
uint64_t timestamp(env_t *env);
void timer_reset(env_t *env);
void timer_cleanup(env_t *env);
