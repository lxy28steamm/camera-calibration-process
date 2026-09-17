# Ego-Std 写入协议来源

来源：用户提供的 `/home/lskj/下载/Write/`（2026-09-17）。
仅集成两个 Python 程序；含凭据的文本与命令文件不进入源码。

- 原始 `yctc_kalibr_calibration_write.py` SHA-256：`7e1bc7e5fc5cb96712e4b2dd210d314a0e224ea52cea8672514afa1c11b689c5`
- 原始 `yctc_uvc_xu_procotol.py` SHA-256：`bec602cb2b140bc3eae60f6e155307d7e0d7a93e07f303daf3abcb378631d0ff`

本地改动仅调整 writer 的模块导入路径，使源码和 PyInstaller 均能加载。传输、解锁、提交、激活及回读协议保持原样。
工作台通过 `std_flash.py` 限定目标设备，增加参数校验、写前备份和一次性审核；不开放厂商 recovery 等其它写命令。
网页写入适配当前 Linux V4L2 设备；原脚本其它平台能力不代表网页已验证支持。
