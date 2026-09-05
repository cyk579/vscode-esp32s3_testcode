CC ?= gcc
CFLAGS ?= -std=c11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../main

harness: host_harness.c ../main/line_geometry.c ../main/line_mixer.c ../main/line_control.c
	$(CC) $(CFLAGS) -o harness host_harness.c ../main/line_geometry.c ../main/line_mixer.c ../main/line_control.c

.PHONY: run clean
run: harness obstacle_priority_harness lowres_corner_harness tft_status_harness
	./harness
	./obstacle_priority_harness
	./lowres_corner_harness
	./tft_status_harness

obstacle_priority_harness: obstacle_priority_harness.c ../main/camera_line_follow.c
	$(CC) $(CFLAGS) -Iesp_stubs -Iesp_stubs/freertos -ffunction-sections \
	    -fdata-sections -Wl,--gc-sections -o $@ obstacle_priority_harness.c

lowres_corner_harness: lowres_corner_harness.c ../main/line_geometry.c
	$(CC) $(CFLAGS) -o $@ lowres_corner_harness.c ../main/line_geometry.c

tft_status_harness: tft_status_harness.c ../main/tft_st7735.c
	$(CC) $(CFLAGS) -Iesp_stubs -Iesp_stubs/freertos -o $@ tft_status_harness.c

clean:
	rm -f harness harness.exe obstacle_priority_harness obstacle_priority_harness.exe \
	    lowres_corner_harness lowres_corner_harness.exe tft_status_harness tft_status_harness.exe

# ESP 侧源文件的语法/类型检查：用桩头文件让 host gcc 也能编译一遍。
# 本机没有 idf.py，这是提交前唯一能做的 ESP 端编译验证。
.PHONY: syntax
syntax:
	$(CC) -std=c11 -fsyntax-only -Wall -Wextra -Wno-unused-parameter \
	    -I../main -Iesp_stubs -Iesp_stubs/freertos ../main/camera_line_follow.c
