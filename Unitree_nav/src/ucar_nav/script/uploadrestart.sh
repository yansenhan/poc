#!/bin/bash

# 定义监控间隔时间（秒）
INTERVAL=1

# 定义上传流量阈值（KB）
THRESHOLD=400

# 定义socat进程的名称（可以根据实际情况修改）
SOCAT_PROCESS="socat"

while true; do
    # 获取当前上传流量（这里使用netstat命令，需要根据实际情况调整）
    # 注意：netstat命令可能在某些系统中不可用，可以考虑使用ss命令
    UPLOAD=$(sudo netstat -i | grep ^lo | awk '{print $2}')

    # 转换流量为KB
    UPLOAD_KB=$((UPLOAD / 1024))

    echo "Current upload traffic: ${UPLOAD_KB} KB"

    # 检查上传流量是否小于阈值
    if [ "$UPLOAD_KB" -lt "$THRESHOLD" ]; then
        # 杀掉所有socat进程
        echo "Upload traffic is below threshold, killing socat processes..."
        pkill -f $SOCAT_PROCESS
    fi

    # 等待INTERVAL秒后再次检查
    sleep $INTERVAL
done