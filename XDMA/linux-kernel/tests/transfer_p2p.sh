#!/bin/bash

tool_path=../tools
sourceAddr=0x00000000
destAddr=0x72b20000
gpuSourceAddr=0x74ad0000
fpgaDestAddr=0x20000000

size=$((3840*2160*4))

#enable DMA channel 1&2 c2h
$tool_path/reg_rw /dev/xdma1_control 0x1004 w 0x00FFFFFF
$tool_path/reg_rw /dev/xdma1_control 0x1104 w 0x00FFFFFF
#enable DMA channel 1 h2c
$tool_path/reg_rw /dev/xdma1_control 0x0004 w 0x00FFFFFF

#enable FB reader and writer
$tool_path/reg_rw /dev/xdma1_user 0x0 w 3

# First channel FPGA to GPU - self enqueue
$tool_path/reg_rw /dev/xdma1_user 0x2C w 3
$tool_path/reg_rw /dev/xdma1_user 0x30 w $size
$tool_path/reg_rw /dev/xdma1_user 0x34 w $destAddr
$tool_path/reg_rw /dev/xdma1_user 0x38 w 0
$tool_path/reg_rw /dev/xdma1_user 0x3C w $sourceAddr
$tool_path/reg_rw /dev/xdma1_user 0x40 w 0
$tool_path/reg_rw /dev/xdma1_user 0x28 w 0x1

# First channel GPU to FPGA - self enqueue
$tool_path/reg_rw /dev/xdma1_user 0x48 w 3
$tool_path/reg_rw /dev/xdma1_user 0x4C w $size
$tool_path/reg_rw /dev/xdma1_user 0x50 w $fpgaDestAddr
$tool_path/reg_rw /dev/xdma1_user 0x54 w 0
$tool_path/reg_rw /dev/xdma1_user 0x58 w $gpuSourceAddr
$tool_path/reg_rw /dev/xdma1_user 0x5C w 0
$tool_path/reg_rw /dev/xdma1_user 0x44 w 0x1


# Second channel FPGA to GPU - polled enqueue
#$tool_path/reg_rw /dev/xdma1_user 0x204C w 0
#$tool_path/reg_rw /dev/xdma1_user 0x2050 w $sourceAddr
#$tool_path/reg_rw /dev/xdma1_user 0x2054 w 0
#$tool_path/reg_rw /dev/xdma1_user 0x2058 w $destAddr
#$tool_path/reg_rw /dev/xdma1_user 0x205C w $size
#$tool_path/reg_rw /dev/xdma1_user 0x2060 w 3

#for i in {1..20000}
#do
#$tool_path/reg_rw /dev/xdma1_user 0x207C w 4
#done

