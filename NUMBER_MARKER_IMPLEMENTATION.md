# Number Marker Implementation

负责人：孙宇鹏

模块：`number_marker.py`

本次 handoff closure 基线：`integration` @ `66e526edb1e4c365a5cc1031bcfeb6d91e8c45fc`

## Requirement mapping

| Final requirement | 实现位置 | 当前证据与边界 |
|---|---|---|
| 仅处理 ID 1–5 | `VALID_MARKER_IDS`、`evaluate_number_markers()` | 离线正负样例通过 |
| 忽略无关 ID | `evaluate_number_markers()` | 无关 ID 单独/与有效 ID 混合均不接管错误目标 |
| 忽略已瞄准重复 ID | `aimed_ids` 过滤 | `aimed_ids` 与 `saved_ids` 分开维护 |
| 多目标选最近 | `select_target_marker()` | 有完整可靠距离时取最小距离；否则明确标为 `NEAREST_PROXY`，不冒充物理距离 |
| 宽度大于画面 1/5 | `is_marker_eligible()` | 严格执行 `width / frame_width > 0.20`，等于 0.20 不合格 |
| 先停再瞄准 | `STOPPING` 首帧返回 `RUNNING, motion=None` | coordinator 先暂停巡线并硬停车，再允许下一帧 yaw 请求 |
| face / center | `compute_aim_intent()`、`_integrate_pitch()`、`is_marker_centered()` | 水平误差继续输出底盘 yaw；垂直误差经受限时间积分后输出既有 `GimbalCommand` 绝对 pitch；居中帧不发送多余云台指令 |
| 连续稳定后锁定 | `aim_stable_frames`、frame sequence 去重 | 同一帧重复调用不增加计数 |
| marker 丢失/观测过期 | `LOST`、freshness 校验 | 立即零运动；超过模块超时后 `FAILED` |
| scoring snapshot | `EvidenceRequest`、保存回执 | 本模块只构造请求：真实全帧副本、全帧坐标框、ID、默认 Team 03 文字与中心文字锚点；正式图片只由 `evidence.py` 渲染并写盘 |
| 个人函数由主流程调用 | `NumberMarkerTask.step()` | 已在 `task_registry.py` 注册，`main.build_coordinator()` 逐帧调用；marker source 与 evidence service 均已接入主循环 |

## Current integration interface

主流程提供 `FramePacket(image, sequence, captured_at)`。模块保持冻结签名：

```python
NumberMarkerTask.step(frame: FramePacket, now: float) -> TaskUpdate
```

`NOT_TRIGGERED` 不接管；`RUNNING` 由 coordinator 切换为 external owner；
`COMPLETED` / `FAILED` 由 coordinator 统一硬停车、恢复巡线云台视角、释放并等待稳定时间后恢复
新鲜路线。模块不调用 SDK、不新建相机、不直接发底盘或云台命令。

当前 `marker_source.py` 已把唯一 SDK adapter 收到的、带接收时间的候选转换为
`MarkerCandidate`，并在 coordinator step 前通过 `update_candidates()` 提供给同一个任务实例。
`marker_candidates_from_normalized()` 只做已核实的
`(x, y, w, h, info)` 归一化坐标转整幅像素坐标，不订阅硬件。

## Target selection algorithm

处理顺序为：

1. 拒绝非 1–5、已在 `aimed_ids`、非有限数值、错误帧号和过期候选。
2. 若所有剩余候选都有正的可靠 `estimated_distance_m`，按最小距离选择。
3. 否则用最大表观宽度代理，并将策略记录为
   `NEAREST_PROXY_LARGEST_APPARENT_WIDTH`。
4. 对选中的最近候选再检查尺寸门槛。目标过小返回 `TARGET_TOO_SMALL`，不判成功。

Final 文件未定义 nearest 的距离来源，也未定义“最近但太小”与其他可瞄准候选的优先顺序。
当前顺序保留最近目标语义并明确报告过小；需要教师/实测确认。

## Marker eligibility and aiming

尺寸使用原始整幅图像宽度，不裁剪、不放大。第一次发现合格目标只请求停车。后续帧用归一化
画面误差计算有限 yaw 和 pitch-rate intent：水平方向仍由现有 `MotionCommand.yaw` 控制底盘，
垂直方向通过 `TaskUpdate.gimbal` 返回现有 `GimbalCommand(pitch, yaw)` 绝对视角请求。两轴可在
同一更新中同时输出。

`_target_pitch` 从 `RuntimeConfig.gimbal_pitch` 初始化，`_last_step_at` 用于计算积分时间；单步
时间由模块工程参数 `max_pitch_integration_dt=0.20` 限制。最终 pitch 使用现有
`RuntimeConfig.gimbal_pitch_min` / `gimbal_pitch_max` 限幅，不复制另一套角度范围。垂直已居中、
重复时间戳、目标丢失、陈旧帧、异常和 evidence pending 都不会继续累积 pitch。终态、丢失超时、
新任务与 reset 会清除 pitch 瞬态。

`PITCH_FOLLOW_SIGN=-1.0` 是可单点翻转的工程方向约定。当前约定把画面上方 marker 映射到更正的
绝对 pitch；真实 RoboMaster 上的方向仍为 **HARDWARE SIGN NOT YET VERIFIED**。

中心容差由 `is_marker_centered()` 独立封装。默认把 PDF 中有歧义的“marker 宽高 1/10”解释
为总中心区域宽高，故半边误差阈值为 marker 宽高的 0.05。这是
**ENGINEERING INTERPRETATION**，可在一个配置处替换，不能写成教师已确认规则。

## Duplicate and evidence flow

达到连续 N 个新鲜居中帧时，ID 进入 `aimed_ids` 并生成一次 `EvidenceRequest`。请求保存：

- 真实 `FramePacket.image` 的全帧副本；
- marker box、ID、frame sequence 和采集时间；
- `Team <number> detects a marker with ID of <id>`；
- 画面中心文字锚点。

`NumberMarkerConfig.team_number` 的正式默认值为 `"03"`，仍可显式覆盖。annotation 由
`self.settings.team_number` 与当前 marker ID 动态组成；没有把整句写成固定字符串。首次请求 ID 是
`marker:<id>:frame:<sequence>:attempt:1`，有界重试在旧 ID 后追加 `:retryN`，每次不同；旧 ID
回执无效。

主循环的 evidence service 调用 `take_evidence_request()` 取走请求，由 `evidence.py` 的
`render_task_evidence()` 在全帧副本上画框文字并保存，再用
`acknowledge_evidence(request_id, saved)` 回传实际写盘结果。只有 `saved=True` 才加入
`saved_ids` 并完成；失败不会伪装成功。重试次数可配置且有限，重试不重复执行瞄准。
`EVIDENCE_PENDING` 未回执时保持 `RUNNING` 与零运动，不继续 yaw/pitch 漂移；终态立即清除本模块
的排队请求和 pitch 瞬态。若调用方显式把队号设为空，锁定后仍安全失败，避免伪造队号证据。

## Offline verification

单模块命令：

```powershell
C:\Users\15836\anaconda3\envs\robomaster38\python.exe -m unittest tests.test_number_marker -v
C:\Users\15836\anaconda3\envs\robomaster38\python.exe scripts\check_module.py number_marker
```

本次模块测试共 64 项：原有 56 项中仅把旧模块 renderer 测试改为正式 evidence renderer 测试，
另新增 8 项 handoff/失败清理测试。此前测试覆盖居中无多余 pitch、上下方向、符号
单点翻转、0.20 秒积分上限、重复时间戳、上下角度限幅、水平和垂直同时输出、校正后稳定锁定、
目标短时丢失、陈旧帧、evidence pending 无漂移、完成/失败后新目标从 entry pitch 重新开始，
以及 `TaskUpdate.gimbal` 经真实 coordinator 到既有 `GimbalOutput` 的离线贯通。本次额外验证默认
Team 03、动态 ID、完整请求字段/全帧副本、有界唯一重试、旧回执拒绝、pending 零运动、重复 ID
不再计分和失败即时清理。
结果只可标为 **OFFLINE VERIFIED**。

## Unresolved requirement ambiguities

- nearest 是物理距离还是视觉尺寸代理；如何关联 ToF 与具体 marker。
- 最近 marker 太小但另一个 marker 合格时的选择顺序。
- “face” 要求底盘朝向、云台朝向或两者都要。
- 中心区域的精确几何定义。
- 已瞄准但截图写盘失败时，允许几次补存，以及如何计分。
- marker 样式、真实尺寸、颜色、视距与光照范围。

## Hardware tuning checklist

1. 在不落地驱动的安全测试中确认 `marker_source.stats()["callback_hz"] >= 3.5`，并记录
   `coordinate_mode` 的实际解析结果。
2. 在架空车轮和急停人员就位时，分别把 marker 放在画面上方与下方，核对 pitch 物理方向；
   若相反，只翻转 `PITCH_FOLLOW_SIGN`，不改算法或角度限幅。
3. 核对 chassis yaw 正负号、云台速度、上下限、居中后保持和重复帧无漂移。
4. 进行完整流程：发现 marker、先停车、双轴校正、稳定锁定、真实截图回执、恢复巡线视角与路线。
5. 收集每个 ID 在不同距离、角度、亮度和运动模糊下的真实帧，统计误检/漏检。
6. 标定 marker 表观尺寸与距离；若接入 ToF，证明它和目标框的关联有效。
7. 与教师确认 nearest、face、中心区域、过小目标和保存失败重试规则。
8. 分别验证目标短时丢失、观测过期、video gap、人工急停和任务超时。

尚未连接机器人；`HARDWARE VERIFIED` 状态为 **NOT YET PERFORMED**。
