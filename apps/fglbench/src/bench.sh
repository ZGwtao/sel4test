#!/usr/bin/env bash

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then
    set -o xtrace
fi

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
    echo 'Usage: ./bench.sh kernel-commit'
    exit
fi

run() {
    local out_file=$1
    local project_dir=$2
    local platform=$3
    local machine=$4
    local which_benchmark=$5
    local num_cores=$6

    cd $project_dir
    rm -rf CMakeCache.txt build
    mkdir build
    cd build
    echo wd = $(pwd)
    echo platform = $platform
    ../init-build.sh \
        -DPLATFORM=$platform \
        -DRELEASE=ON \
        -DKernelArmExportPMUUser=ON \
        -DMCS=ON \
        -DSMP=ON \
        -DKernelFastpath=ON \
        -DKernelSignalFastpath=ON \
        -DWhichBenchmark=$which_benchmark \
        -DNumCores=$num_cores
    ninja
    "$MQ_SCRIPT" run -s $machine -c "All is well in the universe" -f images/* -l mq.out
    ls -l mq.out
    sed 's/\r//' <mq.out | awk '/AEPBENCH \| Running benchmark [0-9]+ on [0-9]+ cores/ { printf "%d\t%d\t", $5, $7 } \
         /AEPBENCH \| ### Average across all runs/ { print $8 }' >>$out_file 
}

main() {
    local project_dir=$1
    local kernel_commit=$2
    local platform=$3
    local machine=$4
    local out_file=$5

    local cloned_project_dir=$(mktemp -d)
    rsync -a $project_dir/ $cloned_project_dir

    cd $cloned_project_dir/kernel
    git reset --hard HEAD
    git checkout $kernel_commit

    local temp_out_file=$(mktemp)
    for which_benchmark in {0..5}; do
        for num_cores in {1..4}; do
            run $temp_out_file $cloned_project_dir $platform $machine $which_benchmark $num_cores
        done
    done
    cat <$temp_out_file >$out_file
    rm $temp_out_file
    rm -rf $cloned_project_dir
}

main "$@"
