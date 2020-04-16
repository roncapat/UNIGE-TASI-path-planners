#!/usr/bin/env bash

script_dir="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
build_dir="$script_dir/build__field_d_star"
build_opt_dir="$script_dir/build__field_d_star__optimized"
src_dir="$script_dir/../FieldDStar"
in_dir="$script_dir/Tests"
out_dir="$script_dir/Results"
planner="$build_dir/field_d_planner"
planner_opt="$build_opt_dir/field_d_planner"
postprocessor="$src_dir/plot_path.py"
tablebuilder="$src_dir/build_md_table.py"

rm -rf "$build_dir"
mkdir "$build_dir"
cd "$build_dir"
cmake -G "Unix Makefiles" --target field_d_planner "$src_dir" | tee build.log
make -j8 | tee -a build.log

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
  type="__l0_c0_opt0"
  "$planner" "$f" $params 0 0 0 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l0_c0_opt1"
  "$planner" "$f" $params 0 0 1 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l0_c1_opt0"
  "$planner" "$f" $params 0 1 0 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l0_c1_opt1"
  "$planner" "$f" $params 0 1 1 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l1_c0_opt0"
  "$planner" "$f" $params 1 0 0 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l1_c0_opt1"
  "$planner" "$f" $params 1 0 1 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l1_c1_opt0"
  "$planner" "$f" $params 1 1 0 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  type="__l1_c1_opt1"
  "$planner" "$f" $params 1 1 1 "logfile$type.json" "dbgfile$type.json" "infofile$type.json" &> "planner$type.log"
  python3 "$postprocessor" $f "logfile$type.json" "dbgfile$type.json" "result$type.jpg"
  python3 "$tablebuilder" info >> readme.md

  echo "
  # Lookahead OFF | C-space 0 | Basic D-Lite version
  ![000](result__l0_c0_opt0.jpg)
  # Lookahead OFF | C-space 0 | Initial optimized version
  ![001](result__l0_c0_opt1.jpg)
  # Lookahead OFF | C-space 1 | Basic D-Lite version
  ![010](result__l0_c1_opt0.jpg)
  # Lookahead OFF | C-space 1 | Initial optimized version
  ![011](result__l0_c1_opt1.jpg)
  # Lookahead ON | C-space 0 | Basic D-Lite version
  ![100](result__l1_c0_opt0.jpg)
  # Lookahead ON | C-space 0 | Initial optimized version
  ![101](result__l1_c0_opt1.jpg)
  # Lookahead ON | C-space 1 | Basic D-Lite version
  ![110](result__l1_c1_opt0.jpg)
  # Lookahead ON | C-space 1 | Initial optimized version
  ![111](result__l1_c1_opt1.jpg)

  " >> readme.md
  cd ..
done