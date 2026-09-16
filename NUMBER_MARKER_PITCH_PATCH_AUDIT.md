# Number Marker Pitch Patch Audit

审计日期：2026-09-14

负责人：孙宇鹏

分支：`feat/number-marker-pitch-sun-yupeng`

基线：`integration` @ `de781590a9fdbd1b4d41eb3fa11f92f65a334272`

## 已核实接口

- `models.GimbalCommand` 的字段是 `pitch: float` 和 `yaw: float = 0.0`，单位为度，语义是绝对云台视角。
- `models.TaskUpdate` 在 v0.2 末尾提供 `gimbal: Optional[GimbalCommand] = None`。
- `GimbalOutput.send()` 调用唯一云台出口并去重相同请求；它使用
  `RuntimeConfig.gimbal_pitch_min` / `gimbal_pitch_max` 与对应 yaw 范围限幅。
- 当前配置的巡线视角来自 `RuntimeConfig.gimbal_pitch` / `gimbal_yaw`。默认值分别为
  `-25°` 与 `0°`，合法 pitch 范围为 `[-25°, 10°]`；这些是当前工程配置，不是 Final 教师规定。
- coordinator 对活动任务的 `TaskUpdate.gimbal` 调用 `GimbalOutput.send()`。任务结束、失败、
  超时、人工停止或视频失效后，coordinator 调用 `restore_line_view()`，随后等待
  `gimbal_settle_seconds` 才尝试恢复巡线。
- `route.py` 在每个 RUNNING 更新中附带绝对 `GimbalCommand`，未在任务内调用 SDK；本补丁只参考
  这一公共接口用法，不复制 route 的检测、搜索、状态或业务逻辑。
- `marker_source.py` 已由主循环在 coordinator step 前向 `NumberMarkerTask.update_candidates()`
  注入带回调接收时间的候选。本补丁无需修改观测链。

## 最小改动方案

1. 在 `number_marker.py` 导入当前 `CONFIG` 和 `GimbalCommand`，从实际 RuntimeConfig 读取巡线
   entry pitch、固定 yaw 及 pitch 合法范围。
2. 增加本任务瞬态 `_target_pitch` 和 `_last_step_at`。新 marker 接管时以配置的巡线 pitch
   初始化，不硬编码角度。
3. 在新鲜且垂直未居中的 AIMING step 中把已有 vertical pitch-rate intent 按受限 `dt` 积分成
   绝对 pitch，再按当前 RuntimeConfig 范围限幅；通过 `TaskUpdate.gimbal` 与已有 chassis yaw
   同时返回。
4. marker 已垂直居中时保持当前绝对目标、不继续积分，也不发送重复云台请求；evidence pending、
   目标丢失、陈旧或异常路径不再使用旧 pitch-rate。
5. 完成、失败、丢失超时、新任务开始和 `_reset_transient()` 清理 pitch 瞬态，防止跨 marker 泄漏。
6. 只扩展 `tests/test_number_marker.py`，并更新本模块实现说明与 provenance。

## 配置与验收边界

- `MAX_PITCH_INTEGRATION_DT` 是防循环卡顿造成大跳变的工程参数，不是教师规定。
- `PITCH_FOLLOW_SIGN` 集中控制图像 vertical error 到物理 pitch 的方向，现场只翻转该符号。
- 当前经验不能证明真实机器上正负号正确：**HARDWARE SIGN NOT YET VERIFIED**。
- 保持现有中心区域解释，即 marker 完整宽高的 1/10、误差半范围为 ±5%；不放宽容差。
- 本审计没有运行 `main.py`、连接机器人或发送真实云台命令。

## 离线补丁结果

- 改动限定在 `number_marker.py`、`tests/test_number_marker.py`、本审计、实现说明和 provenance。
- 原有 40 项 number-marker 测试全部保留，新增 16 项 pitch 专项/贯通测试，合计 56 项通过。
- `scripts/check_module.py number_marker` 通过静态契约、模块测试和 synthetic coordinator smoke。
- 上述结果仅证明离线逻辑与当前公共接口兼容；硬件方向、实际闭环效果和完整赛道流程均为
  **NOT YET PERFORMED**。
