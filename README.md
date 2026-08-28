# ESP32-S3 电子设计项目

本仓库集中管理 ESP32-S3 小车与摄像头测试程序，使用 ESP-IDF 5.4.4 开发。

## 工程目录

| 工程 | 功能 |
| --- | --- |
| `esp-projects/car-spin` | 三轮小车电机控制、四路红外检测与巡线 |
| `esp-projects/camera-test` | USB 摄像头枚举、ST7735 本地预览、Wi-Fi MJPEG 图像传输与抓拍测试 |

每个工程的接线、参数和运行方法请查看对应目录中的 `README.md`。

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
