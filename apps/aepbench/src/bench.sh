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
    local num_cores=$5

    cd $project_dir
    rm -rf CMakeCache.txt build
    mkdir build
    cd build
    ../init-build.sh \
        -DPLATFORM=$platform \
        -DRELEASE=ON \
        -DKernelArmExportPMUUser=ON \
        -DMCS=ON \
        -DSMP=ON \
        -DKernelFastpath=ON \
        -DKernelSignalFastpath=ON \
        -DNumThreads=$num_cores
    ninja
    "$MQ_SCRIPT" run -s $machine -c "All is well in the universe" -f images/* -l mq.out
    ls -l mq.out
    awk '/Average across all runs/ { print $5 }' <mq.out >>$out_file
}

main() {
    local project_dir=$1
    local kernel_commit=$2
    local platform=$3
    local machine=$4

    local cloned_project_dir=$(mktemp -d)
    rsync -a $project_dir/ $cloned_project_dir

    cd $cloned_project_dir/kernel
    git reset --hard HEAD
    git checkout $kernel_commit

    local out_file=$(mktemp)
    for i in {1..4}; do
        run $out_file $cloned_project_dir $platform $machine $i
    done
    cat $out_file
    rm $out_file
    rm -rf $cloned_project_dir
}

main "$@"
