# RoboMaster Final Project

面向期末任务的低速巡线与多人协作基线。当前真正实现的是基础路线检测、低速运动请求、普通短时漏检容错、持续丢线/视频失效锁停、暂停与显式恢复、单一运动出口和离线接管示例。数字标识、红绿灯、绕障、长断线、岔路、截图证据及完整任务流程尚未实现。

公共接口版本为 **v0.1**；协作流程基线由标签 `v0.2-collaboration-baseline` 标识。离线测试通过不代表实车稳定。

## 主要文件

- `main.py`：唯一实车入口，目前只运行基础巡线。
- `camera_source.py`：产生共享最新帧 `FramePacket`。
- `line_detector.py`：HSV 路线检测。
- `controller.py`、`runtime.py`：低速控制、短时漏检、暂停/故障/恢复。
- `motion_output.py`：唯一正常底盘运动出口。
- `models.py`：v0.1 公共数据类型。
- `examples/offline_takeover.py`：不连接机器人的接管与恢复示例。
- `tests/`：安全离线测试。

## Windows 环境与依赖

建议使用 Python 3.8。第一次克隆后在项目根目录执行：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## 安全离线验证

激活虚拟环境后执行统一检查：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1
```

脚本只做语法检查、单元测试和离线接管示例，不运行 `main.py`，不连接机器人。单项命令见 `VERIFICATION.md`。

## 实车入口：仅限负责人安排

以下命令会连接机器人并启动视频，不属于自动化或代码助手验证：

```powershell
python main.py
```

只有在负责人确认提交、参数、安全区、架空车轮与急停人员后才能人工运行。按键：`SPACE` 在新鲜有效路线下恢复/运行中暂停，`R` 清除故障但仍保持停止，`Q`、`ESC` 或 `Ctrl+C` 退出停车。默认 `0.32 m/s`、HSV、ROI、控制增益和超时都是待实车验证的初值。

## 协作文档导航

- [PROJECT_HANDOVER.md](PROJECT_HANDOVER.md)：负责人先读，包含真实状态、审查结论、阶段目标和首次实车清单。
- [MODULE_GUIDE.md](MODULE_GUIDE.md)：冻结的 v0.1 类型、单位、调用顺序和模块示例。
- [TASKS.md](TASKS.md)：可领取工作包、优先级、边界、依赖、验收和降级方案。
- [COLLABORATION_GUIDE.md](COLLABORATION_GUIDE.md)：Windows 下从克隆到 PR、联调和标签的完整教程。
- [LEAD_AGENT_GUIDE.md](LEAD_AGENT_GUIDE.md)：负责人日常组织、合并和代码 Agent 使用指令。
- [DELIVERABLES.md](DELIVERABLES.md)：开发过程应保留的最终交付材料。
- [VERIFICATION.md](VERIFICATION.md)：实际执行过的离线验证及其边界。
- [.github/pull_request_template.md](.github/pull_request_template.md)：PR 必填检查项。

协作流固定为：`任务分支 → PR 到 integration → 离线/实车联调 → integration 合入 main → 稳定标签`。普通成员不得直接向 `main` 或 `integration` 推送业务修改。
