# Number Marker Implementation

负责人：孙宇鹏

模块：`number_marker.py`

基线：`integration` @ `2287dc110d82c24ca457e22f5828269a6efc1e79`

## Requirement mapping

| Final requirement | 实现位置 | 当前证据与边界 |
|---|---|---|
| 仅处理 ID 1–5 | `VALID_MARKER_IDS`、`evaluate_number_markers()` | 离线正负样例通过 |
| 忽略无关 ID | `evaluate_number_markers()` | 无关 ID 单独/与有效 ID 混合均不接管错误目标 |
| 忽略已瞄准重复 ID | `aimed_ids` 过滤 | `aimed_ids` 与 `saved_ids` 分开维护 |
| 多目标选最近 | `select_target_marker()` | 有完整可靠距离时取最小距离；否则明确标为 `NEAREST_PROXY`，不冒充物理距离 |
| 宽度大于画面 1/5 | `is_marker_eligible()` | 严格执行 `width / frame_width > 0.20`，等于 0.20 不合格 |
| 先停再瞄准 | `STOPPING` 首帧返回 `RUNNING, motion=None` | coordinator 先暂停巡线并硬停车，再允许下一帧 yaw 请求 |
| face / center | `compute_aim_intent()`、`is_marker_centered()` | 当前可输出底盘 yaw；vertical pitch 仅保留 intent，等待公共云台命令接口 |
| 连续稳定后锁定 | `aim_stable_frames`、frame sequence 去重 | 同一帧重复调用不增加计数 |
| marker 丢失/观测过期 | `LOST`、freshness 校验 | 立即零运动；超过模块超时后 `FAILED` |
| scoring snapshot | `EvidenceRequest`、`render_evidence_image()`、保存回执 | 请求含真实全帧、框、ID、队号文字和中心文字锚点；当前 evidence 层尚未接线 |
| 个人函数由主流程调用 | `NumberMarkerTask.step()` | 已在 `task_registry.py` 注册，`main.build_coordinator()` 逐帧调用 |

## Current integration interface

主流程提供 `FramePacket(image, sequence, captured_at)`。模块保持冻结签名：

```python
NumberMarkerTask.step(frame: FramePacket, now: float) -> TaskUpdate
```

`NOT_TRIGGERED` 不接管；`RUNNING` 由 coordinator 切换为 external owner；
`COMPLETED` / `FAILED` 由 coordinator 统一硬停车、释放并等待新鲜路线恢复。模块不调用 SDK、
不新建相机、不直接发底盘或云台命令。

当前仓库没有 marker 订阅缓存。整合负责人应把唯一 SDK adapter 收到的、带接收时间的候选
转换为 `MarkerCandidate`，通过 `observation_provider` 或 `update_candidates()` 提供给同一个
`NumberMarkerTask` 实例。`marker_candidates_from_normalized()` 只做已核实的
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
画面误差计算有限 yaw 和 pitch intent：方向符号、增益、速率上限都在
`NumberMarkerConfig` 集中管理。

当前 `MotionCommand` 没有 gimbal pitch 字段。水平误差映射到底盘 yaw；垂直误差保存在
`last_aim_intent.pitch_rate`。只要垂直方向未居中，模块不会虚假进入 `AIM_LOCKED`，并返回
`GIMBAL_PITCH_INTEGRATION_REQUIRED`。是否以底盘或云台解释 “face” 仍需澄清。

中心容差由 `is_marker_centered()` 独立封装。默认把 PDF 中有歧义的“marker 宽高 1/10”解释
为总中心区域宽高，故半边误差阈值为 marker 宽高的 0.05。这是
**ENGINEERING INTERPRETATION**，可在一个配置处替换，不能写成教师已确认规则。

## Duplicate and evidence flow

达到连续 N 个新鲜居中帧时，ID 进入 `aimed_ids` 并生成一次 `EvidenceRequest`。请求保存：

- 真实 `FramePacket.image` 的全帧副本；
- marker box、ID、frame sequence 和采集时间；
- `Team <number> detects a marker with ID of <id>`；
- 画面中心文字锚点。

整合层调用 `take_evidence_request()` 取走请求，保存 `render_evidence_image()` 的结果，再用
`acknowledge_evidence(request_id, saved)` 回传实际写盘结果。只有 `saved=True` 才加入
`saved_ids` 并完成；失败不会伪装成功。重试次数可配置，重试不重复执行瞄准。未配置真实队号
会在锁定后失败，避免生成伪造队号证据。

仓库现有 `EvidenceRecorder` 没有任务请求或回执接口，所以上述接线仍由整合负责人完成。

## Offline verification

单模块命令：

```powershell
C:\Users\15836\anaconda3\envs\robomaster38\python.exe -m unittest tests.test_number_marker -v
C:\Users\15836\anaconda3\envs\robomaster38\python.exe scripts\check_module.py number_marker
```

覆盖无 marker、ID 1/5、无关 ID、多 marker、已瞄准重复、尺寸边界、左右 yaw、上下 pitch
intent、稳定帧、同帧去重、目标丢失、陈旧帧/观测、NaN、provider 异常、证据成功/失败/
重试、标注保留全场景，以及真实 coordinator harness 的接管/释放。结果只可标为
**OFFLINE VERIFIED**。

## Unresolved requirement ambiguities

- nearest 是物理距离还是视觉尺寸代理；如何关联 ToF 与具体 marker。
- 最近 marker 太小但另一个 marker 合格时的选择顺序。
- “face” 要求底盘朝向、云台朝向或两者都要。
- 中心区域的精确几何定义。
- 已瞄准但截图写盘失败时，允许几次补存，以及如何计分。
- 正式 team number、marker 样式、真实尺寸、颜色、视距与光照范围。

## Hardware tuning checklist

1. 在架空车轮和急停人员就位时核对 chassis yaw 正负号与速率上限。
2. 由整合层接入唯一 marker subscription，记录 callback 时间、字段和丢帧行为。
3. 确认云台 pitch 命令的公共出口、正负号、速度上限及与底盘模式的关系。
4. 收集每个 ID 在不同距离、角度、亮度和运动模糊下的真实帧，统计误检/漏检。
5. 标定 marker 表观尺寸与距离；若接入 ToF，证明它和目标框的关联有效。
6. 与教师确认 nearest、face、中心区域、过小目标和保存失败重试规则。
7. 填入真实 team number，接通 evidence request/回执，核对每张原图、标注图和保存返回值。
8. 分别验证停止、目标短时丢失、观测过期、video gap、人工急停和任务超时。

尚未连接机器人；`HARDWARE VERIFIED` 状态为 **NOT YET PERFORMED**。
