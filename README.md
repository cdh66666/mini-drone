# mini-drone

自制 ESP32-S3 1S 四轴小飞机：飞控固件 + 电脑端调试台。

## 启动
双击 `启动调试台.bat`（需要 Python 3.10+、`pip install pyserial numpy opencv-python`），浏览器打开 http://127.0.0.1:8771/ 。
页面自动通过 WiFi（或 USB）连上飞机。不接飞机看界面：`fly.html?mock`。

## 目录
- `fly.html` / `replay.html`：调试台、历史回放
- `drone_server.py`：本地服务（WiFi/UDP 控制、黑匣子、整次记录、无线刷固件、重心补偿自学）
- `cam_rec.py`：解锁时自动录像；`tools/video_track.py`：从录像算实际漂移
- `firmware/flight_vN/`：固件源码，由 `tools/make_v2.py` 从资料包里的 flight_v1 生成；`firmware/build/` 最新 .bin 用于无线更新
- `flights/`：飞行记录（黑匣子 250 Hz、`full/` 整次 25 Hz、`video/` 追踪结果）
- `backup/`：刷 v2 前的整片固件备份（8 MB）和原参数
- `pack/`：硬件资料包：电路板、原理图、物料清单、最早的固件
- `交接.md`：全部进展和踩过的坑（先看这个）

## 编译固件
```
python tools/make_v2.py <资料包>/flight_v1.ino firmware/flight_v11/flight_v11.ino
arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc,FlashSize=8M,PSRAM=enabled,PartitionScheme=min_spiffs,USBMode=hwcdc --output-dir firmware/build firmware/flight_v11
```
然后调试台“⬆ 更新固件”无线刷入。

## 安全
没确认拆桨不要转电机；空格 = 急停。
