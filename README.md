# Camera Workbench · 通用相机检测工作台

统一管理普通 UVC、拼接双目、Ego / DepthAI 与 Dex 的检测、采集、标定、复测和报告。网页运行于相机所在 Linux 主机，适配电脑及窄屏浏览器。

源码网页版启动（终端前台运行）：

```bash
cd /home/lskj/ego-calibartion
bash start-web.sh
```

启动后访问终端显示的 `http://运行设备IP:8765`；用完在同一终端按 **Ctrl+C**，停止服务并释放相机，已保存的数据保留。每次启动自动选择本机局域网地址，无需登录。`start-web.sh` 直接运行本目录的 Python 后端和前端源码，默认使用 `.venv` 和 `data/`；支持 `--doctor` 自检。没有源码环境时先运行 `bash scripts/install-linux.sh`。

前端位于 `src/ego_calibration/web/`，修改后刷新网页即可；后端修改后需停止并重新运行。若端口被旧服务占用，启动会明确报错，请先停止旧实例。旧 `start-workbench.sh` 优先使用打包的 `.bin`，源码修改请使用 `start-web.sh`。

设备列表分别显示 Ego-Std 的标定 SN 和串口协议返回的相机 SN；协议 SN 不可读时显示“未读取”，不使用 USB 序列号替代。拔插设备后重新扫描会刷新编号并清除旧标定；内置摄像头被自动选中后，新接入一台标定相机时会切换到新相机。

基础单目标定内置 OpenCV，无需 ROS；高级 Kalibr 模型使用可选环境。移动整个目录即可保留默认 `data/` 中的数据。发布包的二进制启动与部署方式见下文文档。

详见 [Linux 部署与完整操作流程](docs/LINUX.md)，包含局域网/SSH 访问、真实尺寸、相机支持范围、标定板参数、复测标准、Kalibr 部署和旧数据迁移。

3.2.0 网页入口：**相机复测**（普通单目 / 双目）、**Dex / 单目标定**、**Ego-Lite 标定**、**Ego-Std 标定**。Ego-Lite 提供同步图像与 IMU 采集、联合标定、EEPROM 备份写入回读；Ego-Std 集成指定 `customer_delivery` 的 H.264 / YCTC SEI → ROS bag → 双目 → 相机—IMU 求解、报告，以及专用标定备份、写入、激活和回读。各自数据和设备写入协议分开。操作见 [各类相机标定流程](docs/CALIBRATION_WORKFLOWS.md)。

代码结构：

- `devices.py`：识别标准视频节点并声明设备能力。
- `capture.py`、`camera_health.py`：无 Qt 采集与基础实测。
- `mono_service.py`、`mono_inspection.py`：共享单目工作流、内置求解与固定参数复测。
- `inspection.py`、`inspection_service.py`：双目几何复测和任务调度。
- `lite_service.py`、`std_service.py`：Ego-Lite 与 Ego-Std 独立标定流程。
- `backends/`：合入的 video_pipeline、Kalibr runner、Dex Flash 后端和厂商桥接库。
- `webapp.py`、`web/`：本地 HTTP 服务、中文响应式界面。
- 原 Ego Qt 工具保留为可选 `--desktop` 入口。

验证与打包：

```bash
.venv/bin/python -m unittest discover -s tests -v
python scripts/build.py
python scripts/package-linux.py
```

Linux 二进制须在 Ubuntu 22.04 / GLIBC 2.35 或更旧环境构建。发布包包含二进制、源代码、配置模板和自检，不包含原始大视频、用户密码或 Conda/ROS 环境。普通设备仅启用受支持的功能，专有工业接口需添加厂商适配器。
