# Open Source Manifest

## Included

- `rdk_transport_ws/src/rdk_transport_base`
  - 底盘串口桥接、手动命令、任务管理、分段导航、H30 IMU 里程计、目标位姿、Web 控制台。
- `rdk_transport_ws/src/rdk_transport_bringup`
  - 传感器、SLAM、Nav2、演示栈 launch/config/scripts/docs。
- `rdk_transport_ws/src/rdk_transport_description`
  - URDF、RViz 和参考模型。
- `rdk_transport_ws/src/rdk_transport_perception_cpp`
  - TROS YOLO 检测消息到 JSON 的桥接节点。
- `rdk_transport_ws/scripts`
  - 板端一键启动/状态脚本。
- `rdk_transport_ws/third_party/wheelbot_lslidar`
  - 雷达驱动源码快照。
- `frontend/qianrushi_qianduan`
  - 独立前端页面与本地 JS/CSS 依赖。
- `frontend/yolo_web_monitor`
  - 从 `yolo_web_monitor_node.py` 提取出的 Web 控制台 HTML 页面。

## Excluded

- `build/`, `install/`, `log/`, `logs/`
- Python `__pycache__` 和 `.pyc`
- 模型权重、运行地图、录包、截图、压缩包
- 本地办公文档与申报材料

## License Notes

本仓库根目录使用 MIT License。第三方目录中的代码可能有其原始许可证要求，正式公开前建议核对上游授权并保留对应版权声明。
