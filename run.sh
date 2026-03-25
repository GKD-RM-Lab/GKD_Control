#!/usr/bin/env bash
cleaned=0

cleanup() {
    if [ "$cleaned" -eq 0 ]; then 
        cleaned=1
        echo "Resetting motors... sending CAN frames for 1 second"
        
        # 记录当前时间，计算 1 秒后的时间点
        local end_time=$(( SECONDS + 1 ))
        
        # 在 1 秒内循环发送
        while [ $SECONDS -lt $end_time ]; do
            cansend can1 201#00.00.00.00.00.00.00.00
            cansend can1 202#00.00.00.00.00.00.00.00
            cansend can1 203#00.00.00.00.00.00.00.00
            cansend can1 204#00.00.00.00.00.00.00.00
            
            cansend can0 201#00.00.00.00.00.00.00.00
            cansend can0 202#00.00.00.00.00.00.00.00
            cansend can0 203#00.00.00.00.00.00.00.00
            cansend can0 141#A1.00.00.00.00.00.00.00
            cansend can0 141#07.00.00.00.00.00.00.00
            cansend can0 206#00.00.00.00.00.00.00.00
            
            # 加上极短的休眠，防止 CPU 跑满和 SocketCAN 缓冲区溢出 (ENOBUFS)
            # 0.02 秒意味着大约每秒发送 50 轮
            sleep 0.02 
        done
        
        echo "Motors reset finished."
    fi
}

on_interrupt() {
    cleanup
    exit 130
}

trap on_interrupt INT TERM
trap cleanup EXIT

/home/gkd/.local/bin/xmake run