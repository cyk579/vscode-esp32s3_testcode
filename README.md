# ESP32-S3 电子设计项目

本仓库集中管理 ESP32-S3 小车与摄像头测试程序。原有工程使用 ESP-IDF 5.4.4，VScode ESP 按实际烧录电脑使用 ESP-IDF 5.5.4、COM6，各工程环境按各自说明选择。

**当前复现 VScode ESP：拉取代码后，在 VS Code 中使用“文件 -> 从文件打开工作区”，选择根目录 [VScode-ESP.code-workspace](VScode-ESP.code-workspace)。** 它只打开内层 VScode ESP 工程，ESP-IDF 扩展的构建/烧录入口不会再因多工程选择而落到 camera-test。具体步骤见 [VScode ESP 构建说明](esp-projects/VScode%20ESP/VScode%20ESP/README.md#构建和烧录)。

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
- VScode ESP：ESP-IDF 5.5.4、COM6；原有 car-spin / camera-test：ESP-IDF 5.4.4
- Visual Studio Code + Espressif IDF 扩展
- Git + Git Graph 扩展

## 获取代码

```powershell
git clone <repository-url>
cd vscode-esp32s3_testcode
```

VScode ESP 的 `.vscode` 设置、工作区入口、`sdkconfig`、默认配置和 `dependencies.lock` 都纳入版本控制。构建目录统一为 `build-local`，烧录与 Monitor 默认使用已确认的 COM6；SDK/Python 安装位置在每台电脑的 ESP-IDF 扩展中选择，工程设置使用相对路径。

ESP-IDF 的 `build` / `build-local` 是编译产物，`managed_components` 可由锁文件下载恢复；它们不是工程选择配置，仍不进入版本控制。编译器与 SDK 程序需要在烧录电脑按相应版本安装。若运行原有 car-spin，进入它自己的目录构建：

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
