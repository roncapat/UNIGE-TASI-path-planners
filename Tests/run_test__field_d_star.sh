#!/usr/bin/env bash

script_dir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
build_dir="$script_dir/build__field_d_star"
src_dir="$script_dir/../FieldDStar"
in_dir="$script_dir/Tests"
out_dir="$script_dir/Results"
planner="$build_dir/field_d_planner"
postprocessor="$src_dir/plot_path.py"

rm -rf "$build_dir"
mkdir "$build_dir"
cd "$build_dir"
cmake -G "Unix Makefiles" --target field_d_planner "$src_dir" | tee build.log
make | tee -a build.log

rm -rf "$out_dir"
mkdir "$out_dir"
cd "$out_dir"

files="$in_dir/*.bmp"
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
  type="__lookahead_off"
  "$planner" $f $params 0 "logfile$type.json" "dbgfile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.bmp"
  type="__lookahead_on"
  "$planner" $f $params 1 "logfile$type.json" "dbgfile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.bmp"
  IFS=' '
  cd ..
done