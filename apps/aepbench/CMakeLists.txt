cmake_minimum_required(VERSION 3.7.2)

project(aepbench C)

set(configure_string "")
config_option(PassiveServer PASSIVE_SERVER "Use a passive server for ping pong benchmark on MCS kernel" DEFAULT ON)
config_string(NumThreads NUM_THREADS "Number of hardware threads on which to run the ping pong benchmark" DEFAULT 4 UNQUOTE)
config_string(WorkUnits WORK_UNITS "Number of work units to do" DEFAULT -1 UNQUOTE)
config_option(PingPongOnly PINGPONG_ONLY "Run only the ping-pong benchmark" DEFAULT OFF)
config_option(SyscallOnly SYSCALL_ONLY "Run only the syscall benchmark" DEFAULT OFF)
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
target_compile_options(aepbench PRIVATE -Werror -O2)

# Set this image as the rootserver
include(rootserver)
DeclareRootserver(aepbench)
