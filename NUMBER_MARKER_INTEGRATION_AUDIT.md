# Number Marker Integration Audit

审计日期：2026-09-12

负责人：孙宇鹏

工作分支：`feat/number-marker-sun-yupeng`

基线：`integration` @ `2287dc110d82c24ca457e22f5828269a6efc1e79`

## 结论

当前仓库已经把 `NumberMarkerTask` 注册到真实主循环，但只提供共享相机帧和底盘
`MotionCommand`。仓库没有 marker 回调缓存、物理距离关联、任务截图请求通道或运行中
云台速度命令。因此本模块可以完整实现 ID 筛选、重复 ID、目标选择、严格尺寸门槛、
停车后瞄准、稳定锁定、目标丢失和证据请求状态机；实际 marker 观测接线、云台俯仰执行、
证据落盘确认仍需整合负责人通过现有公共层接入。不能用离线测试把这些缺口写成已完成实车
验收。

## 当前架构

1. `main.py` 创建唯一 `LatestFrameSource`，每次取得一个 `FramePacket` 后调用
   `TaskCoordinator.step(packet, now)`。
2. `TaskCoordinator` 先调用所有 observer，再更新基础巡线，然后按
   `task_registry.MOTION_TASK_CLASSES` 顺序询问任务。第一个返回 `RUNNING` 的任务接管。
3. 接管时 coordinator 先暂停巡线，再由 `MotionOutput.claim("external")` 硬停车并切换
   唯一 owner。任务返回的运动请求会被限幅，NaN/Inf 会被归零。
4. 接管后任务必须一直返回 `RUNNING`，直到 `COMPLETED` 或 `FAILED`。释放时 coordinator
   硬停车、归还 `line` owner、清理巡线历史，并等待一张新鲜有效路线后恢复。
5. `main.build_coordinator()` 从 `task_registry` 构建全部任务。因此已注册的
   `NumberMarkerTask.step()` 确实在主流程中逐帧调用，无需修改公共文件。

## 已冻结接口

`NumberMarkerTask` 必须保持：

```python
class NumberMarkerTask:
    name = "number_marker"

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        ...
```

输出使用现有 v0.1 类型：

- 检测：`VisualDetection(kind="number_marker", target_id, center, box, confidence)`，坐标均为
  整幅 BGR 图像像素坐标。
- 停车：`TaskUpdate(RUNNING, motion=None)` 或零 `MotionCommand`；coordinator 执行统一硬停车。
- 水平瞄准：返回限幅后的 `MotionCommand(yaw=...)`，由 coordinator 和 `MotionOutput` 发送。
- 完成/失败：返回 `COMPLETED` / `FAILED`；coordinator 统一停车和释放。

## 观测与检测数据

当前生产框架只传入 `FramePacket(image, sequence, captured_at)`，没有 marker observation
类型，也没有调用 `Vision.sub_detect_info(name="marker", ...)`。DJI 官方示例所示 marker
回调元组为归一化的 `(x, y, w, h, info)`；本模块可独立把该格式转换为全帧候选，但不得
自行初始化 Robot、相机或 vision subscription。整合负责人需要在唯一 SDK adapter 中订阅、
缓存带时间戳的 marker 观测，并注入 `NumberMarkerTask`。

当前也没有相机候选与 ToF 的关联。若候选全部含可靠正距离，目标选择可按该距离；否则只能
使用名为 `NEAREST_PROXY` 的最大表观宽度代理。该代理不等于真实最近距离，属于
**HARDWARE / REQUIREMENT CLARIFICATION REQUIRED**。

## 证据接口

`EvidenceRecorder.observe()` 当前只定期保存原始帧和 CSV，并不接收任务检测、标注内容、
一次性截图请求或保存成功回执。`TaskUpdate` 也没有 evidence 字段。直接在本模块另建写盘
系统会与整合负责人边界冲突。

本模块因此暴露 `EvidenceRequest` 和明确的成功/失败确认方法，保留真实 `FramePacket.image`、
marker 矩形、ID、队号标注文本和原场景。只有收到成功回执才写入 `saved_ids`；瞄准完成立即
写入 `aimed_ids`，两者不会混淆。整合负责人后续需把 request 交给 evidence 层并回传实际
保存结果。当前 evidence 模块不能据此宣称 Final scoring snapshot 已完成。

## 瞄准能力边界

现有 `MotionCommand` 只有底盘 `forward/lateral/yaw`。`main._align_camera()` 只在启动时调用
云台 `moveto()`；运行中没有云台命令出口。模块可用底盘 yaw 表达水平朝向，并计算、保存
独立的 vertical/pitch intent，但不能绕过公共运动层直接驱动云台。因此：

- 水平底盘 yaw 可接入当前框架，方向符号仍需架空实测。
- 垂直误差会阻止虚假的 `AIM_LOCKED`，并输出可供后续 gimbal adapter 使用的 pitch intent。
- “face” 是底盘正对还是云台正对，Final 文件存在歧义；当前实现不把画面居中等同于已证明
  完整 face requirement。

## 允许依赖与禁止拥有

本模块可调用：

- `models.FramePacket`、`VisualDetection`、`TaskUpdate`、`TaskStatus`、`MotionCommand`；
- 由整合层注入的无硬件观测 provider；
- 标准库中的不可变数据结构、有限数值检查和锁。

本模块不得拥有或直接调用：

- Robot、camera、vision subscription、chassis 或 gimbal SDK 对象；
- `LatestFrameSource`、`MotionOutput`；
- `main.py`、`coordinator.py`、`task_registry.py` 的生命周期；
- 独立截图目录或第二套证据 writer；
- 其他成员的 traffic light、obstacle、route、junction 状态。

## 集成风险与待办

| 风险 | 当前处理 | 整合前必须解决 |
|---|---|---|
| marker 观测未接线 | 默认 provider 返回空，不误接管 | 在唯一 SDK adapter 中提供有时间戳的官方 marker 回调数据 |
| “nearest” 无真实定义/距离 | 隔离策略；明确标为 `NEAREST_PROXY` | 教师澄清，或建立已标定距离/ToF 关联 |
| 垂直瞄准无公共执行接口 | 保留 pitch intent；不误判锁定 | 由整合负责人决定最小 gimbal command 接口 |
| `face` 语义不清 | 分离画面居中、底盘 yaw、云台 pitch 信息 | 向教师确认 chassis/gimbal 验收口径 |
| 中心容差不清 | `is_marker_centered()` 独立且配置化 | 按教师解释或真实标定调整一个策略 |
| evidence 无事件接口 | request/ack 分离，保存失败不进 `saved_ids` | evidence 层接收请求、保留原场景、返回真实写盘结果 |
| 队号未知 | 配置项不伪造默认队号 | 运行前填入真实 team number |
| 视觉/运动符号未实测 | 所有方向符号和速率可配置 | 架空车轮及安全场地逐项验证 |

审计仅证明接口与离线设计边界；没有连接机器人，没有 hardware verification。
