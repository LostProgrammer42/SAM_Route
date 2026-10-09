#!/bin/bash
# Regression check over every test cell.
#   ./Scripts/Compare_All.sh save    -> run all cells, store results in Baseline/
#   ./Scripts/Compare_All.sh check   -> run all cells, compare with Baseline/
# Run from the folder containing SAM_Route.cpp. Needs `timeout` (Linux; on macOS use coreutils).
mode=${1:-check}
root=$(pwd)
out=$root/.regress_check
[ "$mode" = save ] && out=$root/Baseline
rm -rf "$out"; mkdir -p "$out"

g++ -std=c++17 -g SAM_Route.cpp -o Route || exit 1

run_one() {
  root=$1; out=$2; cell=$3
  name=$(basename "$cell" .rect)
  mkdir -p "$out/$name/SAM_Files"
  cd "$out/$name" || exit 1
  timeout 120 "$root/Route" "$cell" polysilicon,m1 > stdout.txt 2> stderr.txt
  echo $? > exit_code.txt
}
export -f run_one
ls "$root"/Test_Cells/*.rect | xargs -P 4 -I{} bash -c 'run_one "$1" "$2" "$3"' _ "$root" "$out" {}

if [ "$mode" = check ]; then
  if diff -rq "$root/Baseline" "$out" > /dev/null; then echo "ALL CELLS IDENTICAL"; rm -rf "$out"
  else diff -rq "$root/Baseline" "$out"; echo "DIFFERENCES FOUND (details in $out)"; exit 1; fi
fi