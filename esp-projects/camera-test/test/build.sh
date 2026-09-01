#!/bin/sh
# 本机没有 idf.py 也没有 make，用这个脚本做两件事：
#   1) 编译并运行几何层的合成回归测试
#   2) 用桩头文件对 ESP 侧源码做一次语法/类型检查
# 用法：sh build.sh            跑回归
#       sh build.sh <帧.bin>   回放 player.py --save-dir 存下的真实帧
set -e
CC=${CC:-gcc}
CFLAGS="-std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../main"
$CC $CFLAGS -o harness host_harness.c ../main/line_geometry.c ../main/line_mixer.c
$CC -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
    -I../main -Iesp_stubs -Iesp_stubs/freertos ../main/camera_line_follow.c
echo "esp-side syntax check: ok"
./harness "$@"
