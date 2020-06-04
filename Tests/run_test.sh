#!/usr/bin/env bash

script_dir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
build_dir="$script_dir/build"
src_dir="$script_dir/.."
in_dir="$script_dir/Tests"
out_dir="$script_dir/Results"

rm -rf "$build_dir"
mkdir "$build_dir"
cd "$build_dir"
cmake -G "Unix Makefiles" "$src_dir" | tee build.log
make -j8 | tee -a build.log

export PYTHONPATH="$build_dir/Simulator/build/lib"
cd ..
python3 run_test.py

# for nosuid ecryptfs home dirs (allows renice capability)
#sudo mount -i -o remount,suid $HOME