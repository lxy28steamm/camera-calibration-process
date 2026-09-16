来自用户指定 customer_delivery/kalibr_h264_imu_demo。保留交付源码归档、补丁、提取器及许可证。未包含示例视频与预期报告。网页默认板为已确认的 55 mm / 16.5 mm；交付示例为 35.2 mm / 10.56 mm。封装及本机 Kalibr 适配位于上级 std_runner.py；原交付脚本保留。

本机报告绘图通过上级 std_kalibr_runner.py 使用 Kalibr 已有的 matplotlib，替代相机连接图的可选 Cairo 渲染，保留节点、共同角点权重和选中基线；为新版 matplotlib 的色条显式指定当前坐标轴。复用已有逐帧角点提取封装，避免整段图像缓存。不改变求解、误差筛选或时间偏移算法。
