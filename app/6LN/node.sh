#!/bin/bash
for i in `seq 1 $1`;
        do
let ipei=11111+$i
gnome-terminal -e "./node_single.sh $ipei $i" --window-with-profile=TestLN
        done   


