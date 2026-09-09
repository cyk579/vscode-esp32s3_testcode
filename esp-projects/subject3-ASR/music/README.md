# 本地曲库

`library.json` 记录歌曲 ID、歌名、别名与本地 WAV 路径。默认只含 6 秒生成的“测试音”，用于核对扬声器；不会自动播放。

添加真实歌曲时，在 `tracks` 加入一个对象，例如：

```json
{"id": 2, "title": "你的歌名", "aliases": [], "file": "your-song.wav"}
```

歌曲文件放在本目录，或在 `file` 写本机绝对路径。请使用自己已准备的音频文件；本工程不会搜索、下载或接管其他音乐 APP。

转换示例（需要本机已有 FFmpeg）：

```powershell
ffmpeg -i input.mp3 -ac 1 -ar 16000 -c:a pcm_s16le your-song.wav
python esp-projects/subject3-ASR/tools/prepare_music.py
```

`prepare_music.py` 更新车端 `musicfs/*.wav`、`musicfs/catalog.id` 以及安卓 `assets/music_catalog.json`。生成后重建车端与 APP，使曲库编号及指纹一致。车端文件系统镜像会加入该工程的标准烧录参数；只更新 app.bin 不会更新歌曲分区。

只说完整歌名或说“播放 + 歌名”均可。名称必须与目录或别名匹配；如歌名恰好是“前进”“停止”等运动词，使用“播放 + 歌名”避免歧义。未知歌名保持当前播放，明确的“暂停音乐”“继续播放”“停止音乐”才控制播放器。

本版为 PCM16 单声道，默认 16kHz，音乐会损失高频，定位为功能实验。16kHz 下每分钟约 1.92MB；24MiB SPIFFS 按 70% 使用上限约容纳 9 分钟音频。提高采样率需要 USB Speaker 原生支持，并同步修改 `library.json` 与 `menuconfig → Subject3 ASR preparation → Speaker and prepared WAV sample rate`。未实现 MP3 解码、SD 卡或联网曲库。
