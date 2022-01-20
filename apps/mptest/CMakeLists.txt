#
# Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)

project(mptest C)

# Use arm arch source files on RISCV
set(arch ${KernelArch})
if(KernelArchRiscV)
    set(arch arm)
endif()

file(
    GLOB
        deps
        src/main.c
)

add_executable(mptest EXCLUDE_FROM_ALL ${deps})

target_link_libraries(
    mptest
    PUBLIC
        # sel4_autoconf
        muslc
        # sel4
        # sel4runtime
        sel4allocman
        sel4vka
        sel4utils
        # sel4rpc
        # sel4test
        # sel4sync
        sel4muslcsys
        # sel4testsupport
        # sel4serialserver_tests
        sel4platsupport
        sel4vspace
    # PRIVATE sel4test-driver_Config
)
