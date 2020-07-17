#!/bin/bash

tool_path=../tools
sourceAddr=0x20000000
destAddr=0xA0C60000
size=$((3840*2160*4))

$tool_path/reg_rw /dev/xdma0_control 0x1004 w 0x00FFFFFFFF

$tool_path/reg_rw /dev/xdma0_user 8292 w 0
$tool_path/reg_rw /dev/xdma0_user 8296 w $sourceAddr
$tool_path/reg_rw /dev/xdma0_user 8300 w 0
$tool_path/reg_rw /dev/xdma0_user 8304 w $destAddr
$tool_path/reg_rw /dev/xdma0_user 8308 w $size
$tool_path/reg_rw /dev/xdma0_user 8312 w 3
$tool_path/reg_rw /dev/xdma0_user 8316 w 1

for i in {1..20000}
do
#$tool_path/reg_rw /dev/xdma0_control 0x1004 w 0x00FFFFFFFF
$tool_path/reg_rw /dev/xdma0_user 8316 w 1
done

