# RoboMaster Final Project

面向期末任务的低速巡线与多人协作基线。当前只包含稳定巡线、短时漏检容错、安全停车、最小公共接口和离线接入示例；具体期末任务尚未实现。

## 目录

- `main.py`：唯一实车入口和主循环。
- `camera_source.py`：后台最新帧入口。
- `line_detector.py`：HSV 路线检测。
- `controller.py`、`runtime.py`：低速控制与暂停/恢复状态。
- `motion_output.py`：唯一 SDK 运动出口。
- `models.py`：组员共同使用的最小数据类型。
- `examples/offline_takeover.py`：不连接机器人的任务接管示例。
- `tests/`：离线测试。
- `MODULE_GUIDE.md`、`TASKS.md`：接口与分工说明。

## 安装与离线验证

```powershell
cd F:\robomaster\final_project
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python -m unittest discover -s tests -v
python -m examples.offline_takeover
```

上述两个验证命令不连接机器人。不要把 `.venv` 提交到 Git。

## 实车入口

只有在车轮架空、周围无人且已连接 RoboMaster Wi-Fi 时才能人工执行：

```powershell
python main.py
```

- `SPACE`：在新鲜有效路线下启动；运行中再次按下立即暂停。
- `R`：清除丢线或视频故障锁定，仍保持停止。
- `Q` / `ESC` / `Ctrl+C`：退出并立即停车。

默认蓝线 HSV 沿用上一竞速版本的已验证范围。`0.32 m/s`、控制增益、ROI 和丢线时间均只是保守初值，尚未证明适合最终场地。

详细协作规则见 [MODULE_GUIDE.md](MODULE_GUIDE.md)，候选任务见 [TASKS.md](TASKS.md)。
