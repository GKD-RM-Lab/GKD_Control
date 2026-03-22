#!/usr/bin/env bash
cleaned=0

cleanup() {
    if [ "$cleaned" -eq 0 ]; then 
        cleaned=1
        echo "reset motors"
        cansend can1 201#00.00.00.00.00.00.00.00
        cansend can1 202#00.00.00.00.00.00.00.00
        cansend can1 203#00.00.00.00.00.00.00.00
        cansend can1 204#00.00.00.00.00.00.00.00
        cansend can0 201#00.00.00.00.00.00.00.00
        cansend can0 202#00.00.00.00.00.00.00.00
        cansend can0 141#A1.00.00.00.00.00.00.00
        cansend can0 206#00.00.00.00.00.00.00.00
    fi
}

on_interrupt() {
    cleanup
    exit 130
}

trap on_interrupt INT TERM
trap cleanup EXIT

/home/gkd/.local/bin/xmake run 