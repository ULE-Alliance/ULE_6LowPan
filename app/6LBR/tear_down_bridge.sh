#!/bin/bash

ip link set dev br0 down
ip link set eth0 nomaster
ip link set tap0 nomaster
ip link delete tap0
ip link delete br0
