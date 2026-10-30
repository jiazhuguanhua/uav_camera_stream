# ROS1 USB 摄像头 RTP/UDP 图传

Orin NX / L4T R35.6.5 / Ubuntu 20.04 / ROS Noetic / GStreamer 1.16.3，C++14、单一 `camera_stream_node`。直接链接 GStreamer C API：USB MJPEG → jpegdec → videorate/小容量丢旧帧队列 → CPU I420/textoverlay → nvvidconv/NVMM NV12 → nvv4l2h264enc → RTP PT96 → UDP。没有软件 H.264 回退、ROS 图像转发、录制或飞行控制。CPU 解码、转换和叠字意味着不是全程零拷贝。

## 构建与运行

```bash
cd /home/dlab/devices_ws
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1 -DCATKIN_WHITELIST_PACKAGES=uav_camera_stream -j4
source devel/setup.bash
roslaunch uav_camera_stream camera_stream.launch destination_ip:=192.168.3.185
```

上例地址是本次检查已在运行的用户图传目标；换地面电脑时必须替换为其真实 IPv4。没有配置地址会明确退出，不猜网卡或电台地址、不向 0.0.0.0 发送。单独构建包会缓存白名单；后续恢复全工作区构建时显式传 `-DCATKIN_WHITELIST_PACKAGES=''`，无需 clean。不会构建或修改 maxt_pkg。

QGC：Source=`UDP h.264 Video Stream`，UDP URL=`0.0.0.0:5600`，开启自动重连；低延迟和 jitter buffer 按实际链路调整。地面 0.0.0.0 是监听地址。UDP 本地发送不代表地面收到，断网未必报错；不实现重传、FEC/ARQ、动态码率，也不提供可靠文件传输。

## 叠字与配置

默认效果（示意值，不是电台带宽保证）：

```text
261019-193321+8
29fps-2.01Mbps
Battery:68%  Sat:0
```

固定白色字体、黑色描边，**无背景、无阴影**。使用 GStreamer textoverlay，不在 C++/OpenCV 中逐帧画字。时间和遥测文字由主循环更新，默认 5 Hz，秒级 wall clock，与 ROS 模拟时间无关；表示文字更新/处理时刻，不是曝光时间。`TZ=Asia/Shanghai` 仅影响本进程，不改系统时区。

`config/camera.yaml` 配置相机/网络/编码；**`config/overlay.yaml` 配置行号、启用字段、topic、标签、单位、小数位、超时和字体**。修改后重启生效，启动会清理节点私有参数后重新加载文件，避免上次配置残留。也可选择另一份叠字配置：

```bash
roslaunch uav_camera_stream camera_stream.launch \
  destination_ip:=192.168.3.185 overlay_config:=/绝对路径/my_overlay.yaml
```

`overlay.yaml` 的核心设置：

```yaml
overlay:
  time_line: 1
  stats_enabled: true
  stats_line: 2
  update_hz: 5.0
  telemetry_timeout_sec: 3.0
  font_desc: "Sans 20"
  xpad: 16
  ypad: 16
  fps_precision: 0
  mbps_precision: 2
  topics:
    velocity: /mavros/local_position/velocity_local
    attitude: /mavros/imu/data
    battery: /mavros/battery
    satellites: /mavros/global_position/raw/satellites
  fields:
    - {name: battery, enabled: true, line: 3, label: Battery, precision: 0, unit: "%"}
    - {name: satellites, enabled: true, line: 3, label: Sat, precision: 0, unit: ""}
    - {name: horizontal_speed, enabled: false, line: 4, label: HSpeed, precision: 1, unit: "m/s"}
    - {name: vertical_speed, enabled: false, line: 4, label: VSpeed, precision: 1, unit: "m/s"}
    - {name: pitch, enabled: false, line: 5, label: Pitch, precision: 1, unit: deg}
    - {name: yaw, enabled: false, line: 5, label: Yaw, precision: 1, unit: deg}
    - {name: roll, enabled: false, line: 5, label: Roll, precision: 1, unit: deg}
```

行号 1..12；同一行按 fields 数组顺序用 `separator` 连接，未使用的中间行保留空行。启用的遥测行不能占用时间行/统计行；重复字段、未知字段/键、非法参数类型/行号/精度或非绝对 topic 会明确退出。仅订阅已启用字段需要的消息源。`fields: []` 可不订阅任何遥测。字段标签会转义 Pango 标记，不能改变描边或注入额外样式。

`missing_text` 默认 `--`（未收到），`stale_text` 默认 `STALE`（超时），`invalid_text` 默认 `N/A`（NaN、未知或非法数据）。超时按 steady_clock 记录的**本机接收年龄**计算，不使用消息 header 的仿真时间。断开消息源后旧数值会失效，不会一直显示成有效遥测；恢复后自动更新。电量 0%、卫星数 0 等真实零值仍正常显示。

## 各数据含义

| 字段 | 固定消息类型 / 换算和约定 |
|---|---|
| horizontal_speed | geometry_msgs/TwistStamped，`hypot(linear.x, linear.y)`，m/s |
| vertical_speed | 同一速度消息的 `linear.z`，m/s，默认 MAVROS local ENU，向上为正 |
| pitch / yaw / roll | sensor_msgs/Imu，规范化 orientation 四元数后转 ROS ENU/FLU RPY，全部为 deg；零四元数、非有限分量或 `orientation_covariance[0]=-1` 为非法 |
| yaw | 范围约 -180..180，ENU 向东 0°、向北 +90°，**不是北向 0°的磁航向**；俯仰 ±90°时 roll/yaw 存在欧拉角奇异性 |
| battery | sensor_msgs/BatteryState，`percentage` 的 0..1 乘 100；NaN、负值、超过 1 或 present=false 为非法；不通过电压猜电量 |
| satellites | std_msgs/UInt32，使用本机 MAVROS 的 `/global_position/raw/satellites`；0 为真实值，255（MAVLink unknown）及更大值为非法；不是 GPS fix 状态 |

更换 topic 时必须保持上述消息类型、坐标系及单位一致，不做隐式 NED/ENU 猜测。MAVROS 未启动时默认 Battery/Sat 显示 `--`。本包只订阅遥测，不自动启动 MAVROS，不发布 setpoint，不调用解锁、模式、任务、参数写入或其他飞控服务。

`fps` 是上一统计窗口（约 1 秒）**编码器入口 NV12 原始帧**的计数 / steady_clock 秒数，不把编码 buffer 或 RTP 包当帧。`Mbps` 为同窗口 udpsink `bytes-served` 的差值 × 8 / 秒 / 1,000,000，包含 RTP 包头，不含 UDP/IP 包头；只表示本机 socket 提交的实际数据率，不是电台可用带宽、地面接收速率或端到端吞吐。默认 fps 四舍五入到整数，Mbps 保留 2 位，可在配置调整到 0..3 位。统计尚未可用时显示 `--fps---Mbps`。

默认时间基串 `time_format: "%y%m%d-%H%M%S"`；程序自动追加简短 UTC 偏移，上海 `+8`、UTC `+0`、印度 `+5:30`、尼泊尔 `+5:45`，保留非整小时偏移及 DST。不要在 time_format 重复放 `%z` / `%Z` 或换行；非法时区明确退出。`overlay_time=false` 仅隐藏时间行，统计和已启用遥测仍显示。

## 相机、其他私有参数与恢复

当前 AX USB2.0 的稳定采集路径：`/dev/v4l/by-id/usb-AX_USB2.0_AX_USB2.0-video-index0`。MJPEG 枚举 1280×720@31、1920×1080@30，**720p30 和 720p60 MJPEG 都未枚举**。31 是模式声明；场景短测真实输入约 29.8 fps，drop-only 不补帧，因此输出也可低于 30。YUYV 720p 仅 9 fps，本包不使用 YUYV；video1 是同设备另一个节点。

自动曝光（exposure_auto=3）、自动白平衡已开启，曝光优先也开启，暗处可能降帧。节点沿用当前 UVC 值；没有枚举可控对焦项，不能声称有自动对焦。所有默认码率/帧率/MTU 都只是起始调试值，不是性能或电台最优保证。

| 参数 | 默认 / 范围 |
|---|---|
| device | 上述稳定 by-id 绝对路径；启动严格检查设备枚举的离散 MJPEG 模式，不自动换模式 |
| destination_ip / destination_port | 必须配置数字单播 IPv4 / 5600（1..65535） |
| capture_width / height / fps | 1280 / 720 / 31；宽高 16..8192 偶数，fps 1..240 |
| output_fps | 30；1..capture_fps，仅丢帧 |
| bitrate_bps | 2000000 bit/s；10000..200000000，硬件 CBR |
| idr_interval_frames | 30；1..10000；iframeinterval/idrinterval 均设置，SPS/PPS 随 IDR 重发 |
| rtp_mtu | 1200；256..1500，RTP 包字节数，不含 UDP/IP |
| overlay_time / overlay_timezone / time_format | true / Asia/Shanghai / `%y%m%d-%H%M%S` |
| no_frame_timeout_sec / restart_delay_sec | 5 / 3，均 1..300 秒 |
| enabled | true；false 真正停止并释放相机，不自动重启 |

`~set_enabled` 标准 std_srvs/SetBool，重复操作幂等：

```bash
rosservice call /camera_stream/set_enabled 'data: false'
rosservice call /camera_stream/set_enabled 'data: true'
rostopic echo /camera_stream/diagnostics
```

单一主循环处理服务、遥测、bus ERROR/EOS、输入/编码 watchdog 和管线销毁/重建；pad probes 只做原子计数，不在 streaming thread 销毁管线。监测、重试、窗口和遥测年龄均使用 steady_clock，/use_sim_time=0 或校时不影响恢复。输入及编码前各队列最多 2 buffers、丢旧帧。SIGINT/ROS shutdown 完整释放相机、Gst 引用及任务线程。

`~diagnostics` 约 1 Hz：运行状态、三处活动年龄、真实原始帧 fps、编码字节率、RTP offered 字节率、本地 socket 提交字节率、重启尝试、最近错误、实际 caps；新增 `overlay_text`、`overlay_fps`、`overlay_udp_mbps` 和每个已启用字段的 `telemetry_<name>_state/value`。未收到不输出伪造数值。

## 依赖检查与常见故障

构建/运行 ROS 依赖：roscpp、diagnostic_msgs、std_srvs、geometry_msgs、sensor_msgs、std_msgs、roslaunch。开发依赖：pkg-config、libgstreamer1.0-dev、libgstreamer-plugins-base1.0-dev；运行需要 Base/Good 插件、tzdata 和 JetPack 已有的 NVIDIA 编码插件。不自动安装依赖、不升级 JetPack/GStreamer。

```bash
source /opt/ros/noetic/setup.bash
for p in roscpp diagnostic_msgs std_srvs geometry_msgs sensor_msgs std_msgs; do rospack find "$p"; done
pkg-config --modversion gstreamer-1.0 gstreamer-video-1.0
for e in v4l2src jpegdec queue videorate videoconvert textoverlay nvvidconv nvv4l2h264enc rtph264pay udpsink; do
  gst-inspect-1.0 "$e" >/dev/null || break
done
v4l2-ctl -d /dev/video0 --list-formats-ext
v4l2-ctl -d /dev/video0 --list-ctrls-menus
fuser -v /dev/video0 /dev/video1
```

- 相机忙：先检查占用，不 kill 其他业务。参数不支持时明确报错，不自动回退。
- 无遥测：检查 `rostopic type`、配置的消息契约、MAVROS 是否连接；无 GPS fix 不等于不存在卫星计数。
- 没有地面图像：先看本地 caps、fps、socket 字节率，再检查目标地址、防火墙、QGC；本地健康不代表地面在线。
- CRITICAL：保留 textoverlay 后**固定 CPU I420 caps**。旧基线已通过 gdb 定位到 overlay 的下游 caps 查询触发 nvvidconv 内空 caps 复制；固定 caps 后 fatal-criticals 真机复测通过，不屏蔽日志。无需 h264parse。
- 接收验证：RTP 与 RTCP 监听端口不要重叠；测试使用发送5620、解码5624/5625、单帧5626/5627。接收端中途加入需等下一组 SPS/PPS/IDR；可能先报缺 PPS，不能据此声称端到端失败或已无丢包。

## 验证与边界

C++ 单元测试：

```bash
cd /home/dlab/devices_ws
source /opt/ros/noetic/setup.bash
catkin_make run_tests_uav_camera_stream_gtest_overlay_test -j4
catkin_test_results build/test_results/uav_camera_stream
```

13 项测试覆盖水平/垂直速度、组合及单轴姿态/四元数归一化/奇异点、电量 0/73/100% 及非法值、卫星 0/17/254/未知255、NaN/Inf/无姿态、缺失/超时/恢复、精度/行号/主题/拼写/类型校验以及上海、UTC、半小时/45分钟时区和 DST。

另有 C++ `overlay_integration_test`（仅测试构建，不安装）向**独立 master 的 /overlay_test/** 发布合成消息，逐项对照：水平5 m/s、垂直−2.5 m/s、pitch20°、yaw−90°、roll30°、电量73%、卫星17；再验证全项非法、全项 stale、全部恢复真实0，并检查配置的独立行4..10与实际视频统计。绝不向真实 MAVROS topic 发布合成值。完整步骤和脚本保存在验证目录，不要在真实业务 master 上运行该测试。

2026-10-30 实测：真实 CubeOrange+、MAVROS 只加载 sys_status/imu/local_position/global_position 接收插件，已收到全部数据源；电量约68%、Sat=0，速度与姿态有更新，飞控报告 armed=false，未调用飞行服务、未执行飞行任务。最小插件组关闭 command 插件，因此 MAVROS 自动版本请求出现服务不可用警告；这不影响本次已收到的遥测，原始日志保留。临时 MAVROS 测后停止、释放串口。

真实 AX 摄像头 + NVENC + RTP/UDP 测试在 /use_sim_time=true、ROS 时间恒0下运行，逐项集成和450帧解码通过，另保存临时单帧检查无背景白字黑描边及全部字段。启停、重建和停止释放继续适用。视频只作短测，不录长视频；截图看完删除，仅持久保留文本/代码证据。

完整本轮证据：`/home/dlab/devices_ws/uav_camera_overlay_validation/`。旧相机/旧叠字验证见该目录 `README-before-overlay.md` 以及 `../uav_camera_validation/`、`../uav_camera_ax_validation/`；旧图片样式和测量值只作历史记录。

未验证：原版 QGC 当前屏幕实际显示、电台链路、真实拔插、断网恢复、长期稳定性、端到端延迟，以及动态飞行时遥测精度。不会通过飞行任务来验证。可扩展字段的换算与订阅集中于 overlay.hpp/overlay.cpp，测试集中于 test/，不引入通用大框架。
