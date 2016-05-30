#!/bin/bash
if [[ $# -ne 1 ]] ; then
    echo "Please add the username for your pc login"
    exit 1

fi
ip tuntap add tap0 mode tap user $1
ip link set tap0 up
ip link add br0 type bridge
ip link set tap0 master br0
ip link set dev eth0 down
ip addr flush dev eth0 
ip link set dev eth0 up
ip link set eth0 master br0
ip link set dev br0 up
