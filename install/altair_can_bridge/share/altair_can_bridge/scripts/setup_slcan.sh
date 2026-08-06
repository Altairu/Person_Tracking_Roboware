#!/bin/bash
DEV=$1
BAUD=$2

if [ -z "$DEV" ]; then
    echo "Usage: $0 <device> [baudrate]"
    exit 1
fi

if [ -z "$BAUD" ]; then
    BAUD=1000000
fi

# Stop slcand if already running
killall slcand 2>/dev/null || true

# Bring down can0 if it exists
ip link set can0 down 2>/dev/null || true

# map baudrate to slcan speed code
case $BAUD in
    1000000) S_BAUD="-s8" ;;
    800000)  S_BAUD="-s7" ;;
    500000)  S_BAUD="-s6" ;;
    250000)  S_BAUD="-s5" ;;
    125000)  S_BAUD="-s4" ;;
    100000)  S_BAUD="-s3" ;;
    50000)   S_BAUD="-s2" ;;
    20000)   S_BAUD="-s1" ;;
    10000)   S_BAUD="-s0" ;;
    *)       S_BAUD="-s8" ;;
esac

slcand -o -c $S_BAUD $DEV can0
sleep 0.5
ip link set can0 up type can bitrate $BAUD
