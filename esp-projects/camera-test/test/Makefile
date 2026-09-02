CC ?= gcc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../main

harness: host_harness.c ../main/line_geometry.c ../main/line_mixer.c ../main/line_control.c
	$(CC) $(CFLAGS) -o harness host_harness.c ../main/line_geometry.c ../main/line_mixer.c ../main/line_control.c

.PHONY: run clean
run: harness
	./harness

clean:
	rm -f harness harness.exe

# ESP 侧源文件的语法/类型检查：用桩头文件让 host gcc 也能编译一遍。
# 本机没有 idf.py，这是提交前唯一能做的 ESP 端编译验证。
.PHONY: syntax
syntax:
	$(CC) -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
	    -I../main -Iesp_stubs -Iesp_stubs/freertos ../main/camera_line_follow.c
