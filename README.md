# ESP32-S3 电子设计项目

本仓库集中管理 ESP32-S3 小车与摄像头测试程序，使用 ESP-IDF 5.4.4 开发。

## 工程目录

| 工程 | 功能 |
| --- | --- |
| `esp-projects/car-spin` | 三轮小车电机控制、四路红外检测与巡线 |
| `esp-projects/camera-test` | USB 摄像头枚举、RGB565 黑线识别、三轮车巡线、ST7735 本地预览与可选 Wi-Fi 图像传输 |
| `esp-projects/camera-claude` | 原组巡线/避障/推球移植，默认模式6，编码器与原组速度闭环启用 |
| `esp-projects/VScode ESP/VScode ESP` | 另一组摄像头巡线与超声波避障移植，仅启动巡线避障，找球代码停用 |
| `esp-projects/camera-other_group` | 另一组的原始仓库，只读参考，不要修改 |

每个工程的接线、参数和运行方法请查看对应目录中的 `README.md`。

2026-09-06：上述两份移植工程及 `引脚对应表2.xlsx` 已将 LCD CS 从 GPIO47 改为 GPIO0，
实物也需同步改线。GPIO0 的复位电平要求、完整映射和验证范围见 [引脚核查与复现说明](引脚核查与复现说明.md)。

白屏排查更新：**VScode ESP 暂把 LCD CS 改为 GPIO1，须先拔掉 GPIO1 原有的水平舵机信号线。** Excel 与 camera-claude 仍保留 CS=GPIO0，切换工程时须恢复对应接线。当前接线和 Monitor 排查步骤见 [VScode ESP 说明](esp-projects/VScode%20ESP/VScode%20ESP/README.md)。
`car-spin` / `camera-test` 保留原版本，切回它们时需另行核对其接线，不能直接套用此次接线。

## 开发环境

- ESP32-S3 DevKitC-1
- ESP-IDF 5.4.4
- Visual Studio Code + Espressif IDF 扩展
- Git + Git Graph 扩展

## 获取代码

```powershell
git clone <repository-url>
cd vscode-esp32s3_testcode
```

ESP-IDF 的 `build` 和 `managed_components` 目录是本地生成内容，不进入版本控制。克隆后进入具体工程并重新构建：

```powershell
cd esp-projects\car-spin
idf.py build
```

## 分支协作建议

稳定代码保存在 `main` 分支。每个人开发新功能时，从最新 `main` 创建独立分支：

```powershell
git switch main
git pull
git switch -c feature/功能名称
```

完成修改后提交并推送自己的分支：

```powershell
git add .
git commit -m "描述本次修改"
git push -u origin feature/功能名称
```

之后通过 GitHub Pull Request 审查并合并到 `main`。Git Graph 用于查看、创建、切换和合并分支，但远程同步仍基于相同的 Git 提交与分支。
