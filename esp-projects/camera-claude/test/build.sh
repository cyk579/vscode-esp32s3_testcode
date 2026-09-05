#!/usr/bin/env bash
# host 端验证。本机没有 idf.py，所以分两步：
#   1) 纯 C 的几何/控制/混控层编成可执行文件跑断言
#   2) 依赖 ESP-IDF 的文件用 esp_stubs 里的假头文件做语法检查
# 这两步都不能证明车在赛道上能跑，只能保证符号和编译是对的。
set -euo pipefail
cd "$(dirname "$0")"

CC=${CC:-gcc}
CFLAGS="-std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../main"

echo "[1/2] 编译并运行 host 回归测试"
$CC $CFLAGS -o harness harness.c \
    ../main/line_detect.c ../main/line_control.c ../main/line_mixer.c -lm
./harness

echo
echo "[2/2] ESP 侧文件语法检查"
for src in line_follow.c camera_display.c ultrasonic.c; do
    printf '  %-20s' "$src"
    $CC -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
        -I../main -Iesp_stubs -Iesp_stubs/freertos "../main/$src"
    echo "ok"
done

echo
echo "全部通过。注意：这只说明能编译、符号对，实车行为仍未验证。"
