#!/bin/bash
min=0
max=15

#cansend vcan0 00002001#0000.0000.0000.0000
for i in {1..1000}; do
  echo "Iteration $i"
  val=$(($RANDOM%($max-$min)+$min))
  echo $val
  hex=$(printf '%x\n' $val)
  echo $hex
  cansend vcan0 "00002000#0${hex}00"
  cansend vcan0 "00002001#0000.0000.0000.${hex}000"
  #echo $min
  #echo $max
done
