# RoboMaster Final Project

面向期末任务的低速巡线与多人协作基线。当前实现了基础路线检测、低速运动请求、普通短时漏检容错、视频失效锁停（**丢线不再锁停**：2026-09-18 起丢线后继续低速直行找线、线回来自动恢复）、暂停与显式恢复、集中式运动/云台出口和离线接管。长断线恢复第二版已有离线实现；它和其他正式任务仍未实车验证。

公共接口版本为 **v0.3**（2026-09-17：只扩展 `GimbalOutput` 的行为 —— 动作串行化、最新目标队列、可重试的巡线视角恢复；`GimbalCommand`/`TaskUpdate` 字段与调用顺序不变，v0.1/v0.2 写成的模块零改动）。**2026-09-18 补：接管裁判改为中央仲裁层** —— 模块只提交控制请求，`control_arbiter.py` 每帧只选一个赢家；优先级显式写在 `task_registry.TASK_PRIORITIES`；请求带租约（`ttl`）；控制权变更打 `[ARB] owner changed: A -> B` 日志。**熔断**：每个模块一次连续运行最长 **6 秒**（`control_arbiter.RUN_SECONDS`），跑完**冷却 3 秒**内不许再接管（`COOLDOWN_SECONDS`，**只锁刚跑完的那个模块**，其他模块照常可接管），冷却期连 `step()` 都不调它。**类型层不变**（`TaskStatus`/`TaskUpdate` 字段与 `step()` 签名都没动），默认抢占行为与之前一致（非安全级模块不能中途抢走别人的控制权，需要打开 `preempt_all`）。v0.2 由 Issue #15 批准增加可选云台请求，协作流程基线由标签 `v0.2-collaboration-baseline` 标识。离线测试通过不代表实车稳定。

## 主要文件

- `main.py`：唯一实车入口，装配基础巡线和 `task_registry.py` 中的任务；各任务仍须分别完成实车验收。
- `camera_source.py`：产生共享最新帧 `FramePacket`。
- `line_detector.py`：HSV 路线检测。
- `controller.py`、`runtime.py`：低速控制、短时漏检、暂停/故障/恢复。
- `motion_output.py`：唯一正常底盘运动出口。
- `gimbal_output.py`：唯一动态云台请求出口，逐帧请求去重且不阻塞。
- `route.py`：长断线走到物理线尾、抬头、有限跨越/扇扫、靠近对齐和稳定重获。
- `route_detector.py`：长断线专用的下方旧线与近全屏单端线段检测，不连接相机。
- `models.py`：v0.2 公共数据类型。
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
- [MODULE_GUIDE.md](MODULE_GUIDE.md)：冻结的 v0.2 类型、单位、调用顺序和模块示例。
- [ROUTE_RECOVERY.md](ROUTE_RECOVERY.md)：长断线第二版状态、边界、参数和待实车事项。
- [TASKS.md](TASKS.md)：可领取工作包、优先级、边界、依赖、验收和降级方案。
- [COLLABORATION_GUIDE.md](COLLABORATION_GUIDE.md)：Windows 下从克隆到 PR、联调和标签的完整教程。
- [LEAD_AGENT_GUIDE.md](LEAD_AGENT_GUIDE.md)：负责人日常组织、合并和代码 Agent 使用指令。
- [DELIVERABLES.md](DELIVERABLES.md)：开发过程应保留的最终交付材料。
- [VERIFICATION.md](VERIFICATION.md)：实际执行过的离线验证及其边界。
- [.github/pull_request_template.md](.github/pull_request_template.md)：PR 必填检查项。

协作流固定为：`任务分支 → PR 到 integration → 离线/实车联调 → integration 合入 main → 稳定标签`。普通成员不得直接向 `main` 或 `integration` 推送业务修改。
