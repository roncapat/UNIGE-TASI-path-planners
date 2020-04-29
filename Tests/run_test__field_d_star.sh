#!/usr/bin/env bash

script_dir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
build_dir="$script_dir/build__field_d_star"
src_dir="$script_dir/../FieldDStar"
in_dir="$script_dir/Tests"
out_dir="$script_dir/Results"
planner="$build_dir/src/field_d_planner"

rm -rf "$build_dir"
mkdir "$build_dir"
cd "$build_dir"
cmake -G "Unix Makefiles" "$src_dir" | tee build.log
make -j8 | tee -a build.log

rm -rf "$out_dir"
mkdir "$out_dir"
cd "$out_dir"

files="$in_dir/*.bmp"
export PYTHONPATH="$build_dir/python/build/lib"
for f in $files; do
  echo "Processing $f file..";
  IFS='_'
  read -ra ADDR <<< "$f"
  IFS=' '
  params="${ADDR[*]: -5:4}"
  IFS='/'
  read -ra ADDR <<< "${ADDR[*]: -6:1}"
  IFS=' '
  name="${ADDR[*]: -1}"
  echo "name: $name"
  echo "params: $params"
  mkdir "$name"
  cd "$name"
  type="c0_opt0"
  "$planner" "$f" $params 1 1 0 "$build_dir/pipe_1" "$build_dir/pipe_2" 0 "." | tee "planner_$type.log"
  type="c0_opt1"
  "$planner" "$f" $params 1 1 1 "$build_dir/pipe_1" "$build_dir/pipe_2" 0 "." | tee "planner_$type.log"
  cd ..
done