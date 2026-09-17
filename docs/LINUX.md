# Linux 小主机部署（3.2.0）

适用：x86_64 Ubuntu 22.04 / 24.04。发布二进制在 Ubuntu 22.04 / GLIBC 2.35 环境构建。相机接在运行服务的小主机上，浏览器可在其他电脑上打开。

## 解压即可迁移的部分

将 `camera-workbench-3.2.0-linux-x86_64.tar.gz` 复制到小主机任意可写目录：

```bash
tar -xzf camera-workbench-3.2.0-linux-x86_64.tar.gz
cd camera-workbench
sha256sum -c SHA256SUMS
bash start-workbench.sh --doctor
bash start-workbench.sh --no-browser
```

浏览器打开终端显示的 `http://运行设备IP:8765`。程序每次启动自动识别当前默认路由对应的活动网卡地址；搬到小主机后无需修改固定 IP。未找到局域网地址时回退到 `127.0.0.1`，联网后重新启动即可更新。停止服务按 Ctrl+C。默认数据保存在本目录 `data/`，移动整个目录可同时保留视频、报告、标定和备份。不要同时在两个进程中操作同一相机或数据目录。

安装系统采集工具（首次需要网络和管理员权限）：

```bash
sudo apt-get update
sudo apt-get install ffmpeg v4l-utils libusb-1.0-0 libgl1 libglib2.0-0
```

网页版本不需要显示器、桌面环境或系统安装 Qt。包内二进制已含 Python、OpenCV、PyYAML、USB / DepthAI 运行库和兼容的可选桌面界面。FFmpeg、内核相机驱动和设备访问权限由小主机提供。`--doctor` 检查依赖、数据可写性、设备和 Sunplus 库；不执行 Flash 写入。

若自检显示视频节点无权限，给运行服务的用户加入 `video` 组后重新登录：`sudo usermod -aG video "$USER"`。DepthAI USB 权限规则按设备厂家安装。

## 在另一台电脑打开网页

默认启动后，同一局域网的其他电脑可直接访问终端显示的 `http://设备IP:8765`，无需账号密码。程序不再生成或读取登录密码。

多个网卡时可显式选定地址：`bash start-workbench.sh --host 192.168.1.20`，或设置 `CAMERA_WEB_HOST`。默认值为 `auto`；IP 改变后重新启动会自动选择新地址。

通过 SSH 转发访问时，先明确使用回环地址启动，再在访问端转发：

```bash
# 小主机上
bash start-workbench.sh --no-browser --host 127.0.0.1
# 访问端电脑上
ssh -L 8765:127.0.0.1:8765 用户名@小主机IP
```

访问端浏览器打开 http://127.0.0.1:8765。直接局域网访问使用 HTTP，跨网络使用 SSH 转发。

## 功能与设备范围

| 设备 / 输入 | 可用流程 |
| --- | --- |
| 普通 Linux V4L2 / UVC 单目 | 扫描、预览、实际尺寸、帧率/画面检查、MJPEG/H.264 原码或 YUYV→FFV1 无损录制、单目标定、固定参数复测 |
| 左右等宽拼接 UVC 双目 | 手选布局，导入真实 JSON / Kalibr 双目 YAML，双目重投影、极线、尺度与覆盖复测 |
| Ego-Std | 标定读取、通用复测；独立 H.264 / SEI → bag → 双目与相机—IMU 标定、报告导出；Linux 专用 Schema v2 备份写入回读 |
| Ego-Lite / DepthAI | EEPROM 读取、双目复测；独立同步采集、相机—IMU 联合标定、EEPROM 备份写入回读 |
| Dex / Sunplus 1bcf:28c4 | 通用单目流程，加专用 Flash 信息、备份、读取、显式确认写入和回读校验 |
| 离线单目 / 拼接双目视频 | 导入后标定或复测，可在没有相机时处理 |

不承诺自动支持所有厂家私有协议。GigE Vision、GenICam、RealSense、Orbbec、独立双 USB 同步及 RTSP 尚未增加专用适配器；若设备暴露标准 V4L2 彩色画面，可先使用通用入口。普通相机不会执行 Ego / Dex 的私有 Flash 指令。

新增 Ego-Lite / Ego-Std 的完整网页操作、输入要求和运行环境见 [各类相机标定流程](CALIBRATION_WORKFLOWS.md)。它们与 Dex 分页、分目录管理。

## 实际操作顺序

1. 扫描并选择设备。普通 UVC 明确选择单目或左右拼接布局。
2. 在单目页填写相机支持的采集尺寸、帧率和输入格式。基础检查使用完整输出尺寸；拼接双目应填两目合计宽度。可用 `v4l2-ctl -d /dev/videoN --list-formats-ext` 查看支持模式。
3. 点击“检查 10 秒”，观察实际尺寸、接收帧率、读帧错误和过暗/过曝提示。请求尺寸、真实画面尺寸、标定分辨率是不同字段，工具不会把请求值当作测量值。主机接收间隙不等同于硬件丢帧数。双目实时网页预览使用连续视频流，目标 30 FPS，分别显示“采集”和浏览器实际绘制的“网页预览”帧率。预览最多缩放到 1600 像素宽，原始图像仍用于几何检测；网络或主机较慢时跳过旧预览帧，实际帧率以页面为准。复测期间上方实时画面持续更新，方便调整标定板位置；下方“最近一次复测采样”独立显示角点与误差叠加图，默认每秒采样一次，并显示样本编号、时间和检测结果。离线视频的采样结果也在此处展示。停止复测或关闭实时画面后，最近一次采样图仍可查看。
4. 支持读取的设备点击“读取标定”，成功后自动进入“设备标定”页，直接展示内参、畸变和外参；也可点击“查看标定”返回已读结果。若已有文件，先导入并复测，无需先重新标定。双目在“设备标定”导入 JSON / YAML；单目在“单目 YAML 校验”导入 YAML。页面分别标明实际画面尺寸、标定记录尺寸和附带参考配置，参考配置不用于确认设备内参对应的尺寸。
5. 若没有标定或复测精度不足，采集标定板视频再求解。默认实物为已确认的 6×6 tag36h11，标签外黑边 55 mm、间隙 16.5 mm、ID 0–35；更换板后更新设置。录制过程中覆盖中央、四角和边缘，改变距离及倾角，每个姿态稍停。
6. 单目选择内置 OpenCV（pinhole-radtan / pinhole-equi）或 Kalibr（另支持 omni-radtan / ds-none）。内置分析最多按设置频率采样 600 帧，至少 12 个清晰且位置/尺寸不同的有效观测。求解结果导出 YAML 与 HTML / JSON / CSV；Kalibr 另输出 PDF、TXT、bag。
7. 使用独立录制的视频，点击“用已有 YAML 复测视频”。内置复测固定相机参数，用一半角点估计板位姿、另一半计算误差；报告包含样本、覆盖、倾角、距离变化及失败位姿。它支持 pinhole-radtan / equidistant / none。omni / ds 的精度须使用对应模型的验证程序。相同视频上的拟合误差不算独立验收，默认阈值仅为参考。
8. 导出报告，保留源视频、实物板尺寸与采集设置。Dex 写入仍要求核对当前设备、参数摘要和扇区；没有执行过写入的设备不能宣称持久化已验证。

通用双目 JSON 示例：`config/generic-stereo.example.json`。示例参数必须替换；`resolution` 为单目尺寸；外参为 `X_right = R * X_left + T_m`，平移单位米。工具不自动缩放内参以适配不匹配的视频。

## Kalibr 可选环境

内置 OpenCV 不需要 Conda、ROS 或 Kalibr。高级 Kalibr 求解需要小主机安装独立 ROS/Kalibr 环境，旧电脑的 Conda 环境不能直接复制后假设可用。

已有 Kalibr：在 `config/local.env` 指定 `CAMERA_KALIBR_SETUP`，对应脚本激活环境并加载 Kalibr 的 `setup.bash`；或者配置 `CONDA_BASE`、`EGO_KALIBR_CONDA_ENV`、`EGO_KALIBR_WS`。默认查找当前用户的 miniforge3/miniconda3/anaconda3 及 `~/kalibr_ws`。后端使用包内 Python 源码，无需旧 `camera_calibration` 目录。

新安装：现有 `scripts/install-kalibr-conda.sh` 提供 Conda / RoboStack 构建流程，需先安装 Conda，可设置 `CONDA_BIN` 和 `CATKIN_JOBS=1` 控制路径与小主机内存占用。该流程需要网络、编译工具与较大磁盘空间；不同 Conda 软件源的可用性由安装当时环境决定。先验证：

```bash
bash src/ego_calibration/backends/run.sh python -c 'import rosbag, rospkg, igraph; print(rospkg.RosPack().get_path("kalibr"))'
```

## 代码与旧数据

源码安装（Python 3.10+）：安装 `python3-venv` 后运行 `bash scripts/install-linux.sh`。网页基础依赖不含 Qt 和 DepthAI；需要时额外安装 `.[desktop,depthai]`。旧模块名 `ego_calibration` 与 `dex_*` HTTP 接口保留兼容，公共单目实现已整理至 `mono_service.py`，后端统一放在 `backends/`。

可选 `--dex-project /旧数据目录` 只读列出旧项目中的 `data/results/flash_backups`，不会加载它的代码。迁移时推荐连同旧数据复制后指定新路径；大视频默认不放进软件包。已有 2.x 的 `dex/` 数据目录仍兼容读取。

厂家库位于 `src/ego_calibration/backends/flash_tools/`；此私用迁移包保留现有 x86_64 Sunplus 库，使用相对 `$ORIGIN` 加载，不依赖旧电脑 SDK 目录。SDK 版权和再分发许可仍归厂家，公开发布前自行确认许可。其他架构需厂家对应 SDK，并用 `SUNPLUS_SDK_DIR=... bash .../flash_tools/build.sh` 重新构建；不要把 x86_64 库当成 ARM 库使用。

新相机适配从 `devices.py` 的识别和能力描述、`capture.py` 的采集、对应厂商模块三个位置扩展。共享单目/双目检测逻辑无需复制。

## 双目复测的判定与历史标定对比

复测页的“判定要求与已有标定对比”同时显示已有标定误差、本次误差、参考差值、阈值及单项结果。默认要求为左右重投影 RMS 各不超过 1 px、跨目预测 RMS 各不超过 1.5 px、极线偏差 P95 不超过 1 px。它们是软件工程参考值，并非厂家或通用标准承诺；实际验收应按相机型号、成像模式与用途设置。可在“质量筛选与判定阈值”中调整，并填写项目文件版本、条款等依据备注。软件保存备注，不自动核验标准文件。

总结果还要求有效样本、两目覆盖、距离变化、倾斜角度变化、标定分辨率确认与数据完整性均满足条件。没有有效观测时显示未完成，不用零误差代替；极线 P95 表示有效对应角点偏差的第 95 百分位，并非平均误差或所有角点的最大值。左右及跨目 RMS 按原始单目像素统计，极线偏差使用本次校正投影后的像素尺度。

比较使用本次读取 / 导入的标定记录，忽略附带的 common_calibration 参考配置。左右目历史 RMS 来自原标定拟合，与本次固定参数、另采图像的复测条件不同，数值差仅供参考；未确认标定分辨率时不计算差值。设备的双目联合 RMS 不等同于跨目预测 RMS，单列为历史记录。当前格式未记录同口径极线 P95 时显示未提供。固定 K、D、R、T 的复测没有求解新的内外参或基线，不能把相同参数当成无漂移的证明。

HTML 和 JSON 报告保存本次判定依据、阈值、比较结果及完整原始标定；JSON schema_version 为 2。旧报告保留原文件，可用“重新分析”根据保存的样本生成新版报告。

算法定义参考 [OpenCV 相机标定与三维重建](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。[ISO 10360-13:2021](https://committee.iso.org/standard/74957.html?browse=tc) 涉及光学三维坐标测量系统的长度测量性能验收与复验；不能据此把本工具的像素阈值当作 ISO 合格结论。
