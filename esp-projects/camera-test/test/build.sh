#!/bin/sh
# 本机没有 idf.py 也没有 make，用这个脚本做两件事：
#   1) 编译并运行几何层的合成回归测试
#   2) 用桩头文件对 ESP 侧源码做一次语法/类型检查
# 用法：sh build.sh            跑回归
#       sh build.sh <帧.bin>   回放 player.py --save-dir 存下的真实帧
set -e
CC=${CC:-gcc}
CFLAGS="-std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../main"
$CC $CFLAGS -o harness host_harness.c ../main/line_geometry.c ../main/line_mixer.c ../main/line_control.c
$CC -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
    -I../main -Iesp_stubs -Iesp_stubs/freertos ../main/camera_line_follow.c
# 校准模式那条分支也要编一遍，否则它会一直烂在 #if 里没人发现。
sed 's/^#define LINE_CALIB_MODE 0$/#define LINE_CALIB_MODE 1/'     ../main/camera_line_follow.c > .calib_variant.c
$CC -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter     -I../main -Iesp_stubs -Iesp_stubs/freertos .calib_variant.c
rm -f .calib_variant.c
echo "esp-side syntax check: ok (both LINE_CALIB_MODE branches)"
./harness "$@"
