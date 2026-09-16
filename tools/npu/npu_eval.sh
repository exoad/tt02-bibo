#!/bin/sh
# On the board: every evaluation input through one network binary, one dump
# each, for evaluate.py npu. Inputs are the JPEGs as raw tensors in the format
# the binary wants (compare_npu.py make for int16; raw bytes for uint8; bytes
# less 128 as int8 for pcq), in /tmp/eval-<ext>/e*.<ext>.
#
#   sh npu_eval.sh tagnet_pcq_a733.nb /tmp/npu-pcq i8
NB=$1; OUT=$2; EXT=$3; mkdir -p "$OUT"; n=0
for f in /tmp/eval-$EXT/e*.$EXT; do
    b=$(basename "$f" .$EXT)
    ~/build-pilot-app/npu_probe "$NB" "$f" 1 "$OUT/$b" > /dev/null 2>&1 && n=$((n+1))
done
echo "dumped $n"
