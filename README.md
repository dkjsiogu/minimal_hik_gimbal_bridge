# minimal_hik_gimbal_bridge

`minimal_hik_gimbal_bridge` 是一个独立于 ROS 的机载端最小工程，用来把海康相机画面直接接到 RoboMaster 2026 官方 `0x0310` 自定义客户端链路上。

它的职责很明确：

- 打开第一台海康相机
- 打开图传发送端 USB-TTL 串口，默认 `/dev/ttyUSB0 @ 921600`
- 默认生成官方裁判系统 `0x0310` 帧并直接写入图传发送端
- 可选接收下位机回传的 `GimbalToVision`
- 保留扩展 `SP` relay 包用于旧调试路径

## 功能概览

- 海康相机自动枚举与取流
- 串口 `921600 8N1` 通讯
- `0x0308 / 0x0309 / 0x0310` relay 数据打包
- 默认 `0x0310` 视频模式
- `PV31` H264 分片
- OpenCV 静态简化 / 运动掩码 / 拖影预处理
- `--0310-telemetry` 调试模式

## 目录结构

```text
minimal_hik_gimbal_bridge/
├── CMakeLists.txt
├── scripts/
├── src/
│   ├── main.cpp
│   └── protocol.hpp
├── tools/
│   └── player_end_emulator.py
└── README.md
```

`tools/player_end_emulator.py` 只用于实验室：它把串口里的官方 `0x0310` 帧转成 MQTT `CustomByteBlock`，方便没有真实选手端时复现官方链路。

## 环境要求

- Ubuntu Linux
- CMake 3.16+
- C++17 编译器
- OpenCV（`core` + `imgproc`）
- 系统 `ffmpeg`
- 海康 MVS SDK

推荐安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev ffmpeg
```

## 海康 SDK

默认 CMake 使用：

- 头文件：`/home/dkjsiogu/文档/sp_vision_25/io/hikrobot/include`
- 库文件：`/home/dkjsiogu/文档/sp_vision_25/io/hikrobot/lib/amd64`

如果你的 SDK 放在别处，编译时指定：

```bash
cmake -S . -B build -DHIKROBOT_SDK_DIR=/your/hikrobot
```

## 编译

```bash
cmake -S . -B build
cmake --build build -j
```

编译产物：

```text
build/minimal_hik_gimbal_bridge
```

## 这条链路做的是什么

当前主路径是直连图传发送端串口：

海康相机 -> H264 -> `PV31` -> 官方 `0x0310` 裁判帧 -> `/dev/ttyUSB0` -> 图传发送端。

官方约束已经固化到默认值里：

- 串口：`921600 8N1`
- 命令号：`0x0310`
- 自定义数据段：固定 300 字节
- 发送频率：50Hz，也就是每 20ms 一包

旧的 `SP` relay 包仍保留给下位机调试路径；正常图传部署不需要再让下位机二次封帧。

## `0x0310` 视频模式

默认工作模式是 `0x0310` 视频：

海康帧 -> RGB24 -> OpenCV 预处理 -> ffmpeg/libx264 -> `PV31` -> `0x0310`

### 预处理内容

当前默认预处理包括：

- 中心正方形裁剪
- 固定边长缩放到 `video_size`
- 静态区域简化
- 运动区域保真
- 历史帧拖影叠加
- 中心保护区不过度模糊
- 后级轻量降噪与颜色压缩优化

### `PV31` 格式

`0x0310` 数据段固定 300 字节：

- 24 字节头：`PV31` / version / codec / flags / sequence / stream_ms / payload_bytes / payload_checksum
- 276 字节 H264 净荷

约束：

- 默认 H264
- 默认 50Hz（每 20ms 一包）
- 默认串口波特率 `921600`
- `flags & 1` 表示码流重启

## 调试 telemetry 模式

如果加上：

```bash
--0310-telemetry
```

则 `0x0310` 不再发送视频，而是发送 `VehicleTelemetryV1`，用于调试：

- 相机在线状态
- gimbal 在线状态
- 分辨率 / FPS / frame_seq
- 曝光 / 增益
- yaw / pitch / bullet_speed / bullet_count
- 状态文本

## 编码和串口建议

- `0x0310` 50Hz 建议使用 `921600 8N1`
- 不要用 `115200` 负担 333 字节的高频 relay 包
- 默认码率已经按官方小包链路收敛到约 `80 kbit/s`

## 运行

标准视频模式直接运行脚本即可：

```bash
./scripts/run-bridge.sh
```

这个脚本只做两件事：设置海康 MVS 运行时路径，然后启动 `build/minimal_hik_gimbal_bridge`。

等价命令：

```bash
LD_LIBRARY_PATH=/opt/MVS/bin:/opt/MVS/lib/64 ./build/minimal_hik_gimbal_bridge
```

兼容旧参数：

```bash
./build/minimal_hik_gimbal_bridge \
  --serial /dev/ttyUSB0 \
  --relay-target 3 \
  --payload-text HIK-GIMBAL
```

默认行为：

- 自动打开第一台海康相机
- 自动打开 `/dev/ttyUSB0` 图传发送端串口
- 每 20ms 生成并发送一包官方 `0x0310` 数据
- 默认在 `relay_data[300]` 中发送 `PV31` H264 分片
- 默认直接封成官方 `0x0310` 裁判帧写入图传发送端
- 默认关闭本地 PV31 UDP 调试输出
- 每秒打印抓帧、FPS、发送计数和 backlog

## 常用参数

- `--video-serial <path>`：直连图传发送端的 USB-TTL 串口路径，默认 `/dev/ttyUSB0`
- `--video-serial-baud <rate>`：图传发送端串口波特率，默认 `921600`
- `--serial <path>`：可选云台/下位机 SP 回传串口路径
- `--referee-cmd <hex>`：目标命令号，默认 `0x0310`
- `--relay-target <1..3>`：兼容旧映射参数
- `--ffmpeg <path>`：ffmpeg 路径
- `--video-size <n>`：输出边长，默认 `300`
- `--video-fps <n>`：编码帧率，默认 `30`
- `--video-bitrate-kbps <n>`：码率，默认 `80`
- `--video-gop <n>`：GOP，默认 `30`
- `--crop-size <n>`：中心裁剪边长
- `--no-static-simplify`：关闭静态简化和拖影
- `--motion-threshold <n>`：运动阈值
- `--motion-erode-px <n>`：掩码腐蚀
- `--motion-dilate-px <n>`：掩码膨胀
- `--motion-trail-frames <n>`：拖影历史帧数，默认 `30`
- `--trail-disable-motion-ratio <f>`：全局运动比例过高时禁用拖影
- `--bg-update-alpha <f>`：背景更新速度
- `--bg-blur-sigma <f>`：静态区域模糊强度
- `--center-clear-size <n>`：中心保护区边长
- `--force-monochrome`：强制灰度
- `--viewer-ip <ip>`：打开本地 PV31 UDP 调试输出，正常官方链路不需要

## 裸编码排障

如果你怀疑问题出在预处理阶段，可以临时关闭它：

```bash
./build/minimal_hik_gimbal_bridge \
  --no-static-simplify
```

## 下位机对接约定

下位机需要做的只有两步：

1. 读取 `SP` 包里的 `referee_cmd_id` 和 `relay_data`
2. 把 `relay_data[0:relay_data_length]` 原样封进对应裁判系统命令

不需要做的事情：

- 不需要重新编码视频
- 不需要理解 `PV31` 内层格式
- 不需要拆 telemetry 字段

## 最小验证流程

1. 先只接海康相机，确认 `frame_seq` 在增长。
2. 再接串口，确认 `sent` 在增长。
3. 如果下位机有 `GimbalToVision` 回传，终端会打印 `gimbal-rx`。
4. 最后在解码端订阅 `CustomByteBlock`，确认 `PV31` 和 H264 解码正常。

## 常见问题

### 1. 为什么现在可以直接往图传发送端串口发？

这台电脑已经通过 USB-TTL 接到图传发送端串口。按官方协议，图传发送端串口就是 `921600 8N1` 的裁判系统帧入口；这里写入的是完整 `0x0310` 官方帧，不是自定义私有包。

### 2. 为什么必须 `921600`？

`0x0310` 视频模式是 50Hz 高频小包链路，`115200` 的物理带宽通常不够稳。

### 3. backlog 偶尔升高是否正常？

少量抖动是正常的，只要 backlog 能持续回落、远端能正常解码即可。