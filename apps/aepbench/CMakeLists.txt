

# target_link_libraries(
#     aepbench
#     PUBLIC
#         # sel4_autoconf
#         muslc
#         # sel4
#         # sel4runtime
#         sel4allocman
#         sel4vka
#         sel4utils
#         # sel4rpc
#         # sel4test
#         # sel4sync
#         sel4muslcsys
#         # sel4testsupport
#         # sel4serialserver_tests
#         sel4platsupport
#         sel4vspace
#         sel4bench
#     # PRIVATE sel4test-driver_Config
# )
#
# Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)

project(aepbench C)

set(configure_string "")

config_option(Sel4testHaveTimer HAVE_TIMER "Enable tests that require a timer driver" DEFAULT ON)

config_option(Sel4testSimulation SIMULATION "Disable tests not suitable for simulation" DEFAULT OFF)

config_option(
    Sel4testHaveCache
    HAVE_CACHE
    "Enable tests that require a functioning cache"
    DEFAULT
    ON
)
if(Sel4testAllowSettingsOverride)
    mark_as_advanced(CLEAR Sel4testHaveTimer Sel4testHaveCache)
else()
    mark_as_advanced(FORCE Sel4testHaveTimer Sel4testHaveCache)
endif()
add_config_library(aepbench "${configure_string}")

find_package(musllibc REQUIRED)
find_package(util_libs REQUIRED)
find_package(seL4_libs REQUIRED)
find_package(sel4_projects_libs REQUIRED)

# Need to disable GC sections as it causes our tests to be stripped sometimes
# This option is tested in the following musllibc_setup_build_environment_with_sel4runtime call.
set(UserLinkerGCSections OFF CACHE BOOL "" FORCE)
# This sets up environment build flags and imports musllibc and runtime libraries.
musllibc_setup_build_environment_with_sel4runtime()
sel4_import_libsel4()
util_libs_import_libraries()
sel4_libs_import_libraries()

# Setting LibNanopb On causes its libraries to be created when sel4_projects_libs_import_libraries
# is called.
set(LibNanopb ON CACHE BOOL "" FORCE)
sel4_projects_libs_import_libraries()
add_subdirectory(../../libsel4testsupport libsel4testsupport)

add_executable(aepbench EXCLUDE_FROM_ALL src/main.c src/notifycost.c src/pingpong.c src/timer.c src/utils.c)
target_link_libraries(
    aepbench
    PUBLIC
        sel4_autoconf
        muslc
        sel4
        sel4runtime
        sel4allocman
        sel4vka
        sel4utils
        sel4rpc
        sel4test
        sel4platsupport
        sel4muslcsys
        sel4testsupport
        sel4bench
    PRIVATE aepbench_Config
)
target_compile_options(aepbench PRIVATE -Werror -g)

# Set this image as the rootserver
include(rootserver)
DeclareRootserver(aepbench)
