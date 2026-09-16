# Number Marker Handoff Closure Audit

负责人：孙宇鹏。基线：`origin/integration` @ `66e526edb1e4c365a5cc1031bcfeb6d91e8c45fc`。分支：`feat/number-marker-handoff-closure-sun-yupeng`，独立干净工作树。本审计在业务代码编辑前建立。

## Interface A 现状

| 核对点 | 当前真实状态 |
|---|---|
| 生产队号 | `NumberMarkerConfig.team_number: Optional[str] = None`；`task_registry.build_motion_tasks()` 在正式路径中调用 `NumberMarkerTask()`，没有注入队号。锁定后会 `FAILED:TEAM_NUMBER_REQUIRED_FOR_EVIDENCE`。本补丁按已确认接口把模块默认值设为 `"03"`，保留调用方自定义配置能力。 |
| Request 数据 | `EvidenceRequest` 是模块内冻结 dataclass：`request_id`、`marker_id`、`frame_sequence`、`captured_at`、`detection`、`annotation`、`text_anchor`、全帧 `image` 副本，以及 `attempt`。`MarkerCandidate.to_detection()` 的 center/box 是整幅图像像素坐标。 |
| 请求与回执 | `take_evidence_request()` 原子地取出并清空排队请求；`acknowledge_evidence()` 只接受当前 `_active_evidence.request_id`，成功才写 `saved_ids`。`main.service_task_evidence()` 把 `evidence.py` 的真实写盘布尔结果回传。 |
| ID 与重试 | 首次 ID 为 `marker:<id>:frame:<sequence>:attempt:1`。当前重试按 `old_request_id + ":retry" + next_attempt` 生成，故每次不同；超过 `max_evidence_attempts` 把 outcome 设为 False。旧 ID 不匹配当前活动请求时被拒绝。 |
| Pending 与失败 | 锁定后把 ID 加进 `aimed_ids`，进入 `EVIDENCE_PENDING`；未回执时继续 `RUNNING`、零运动、无新的 pitch 请求。成功回执下一步 `COMPLETED`；失败回执在次数用尽后下一步 `FAILED:EVIDENCE_WRITE_FAILED`。终态时 `_queued_evidence` 当前仅在下次 `_reset_transient()` 清理；本补丁会在终态立即清理。 |
| 渲染所有权 | 生产 `EvidenceRecorder.save_task_evidence()` 调用 `evidence.py::render_task_evidence()`，读取 `request.image`、`request.detection.box`、`request.annotation` 和 `request.text_anchor`，在全帧副本上画框文字并编码写盘。仓库搜索显示 `number_marker.py::render_evidence_image()` 只被 `tests/test_number_marker.py` 与旧模块文档提及，没有生产调用或其他模块公共契约依赖。可删除模块内 helper、专用 `cv2` import，改测试直接核对集成层 renderer。 |
| 职责边界 | number-marker 负责 request 内容、Team 03、ID/去重、触发与验收；integration 负责图像渲染、存盘、文件名、报告、回执和 coordinator 安全护栏。本补丁不改公共文件。 |

## 不变项与风险

- 保持现有 `center_zone_width_fraction=0.10`、`center_zone_height_fraction=0.10`、`aim_stable_frames=3` 的工程解释；不添加未居中存图、机械极限 fallback 或放宽阈值。
- `aimed_ids` 仍表示已完成瞄准锁定，`saved_ids` 仅在成功保存回执后加入。当前重复 ID 策略不在此补丁改变。
- `gimbal_output.py` 仍可能发生 `gimbal.moveto()` action collision；这次不解决，也不声称硬件已验收。
- 验收只用离线测试。旧自测分支的未提交 `NUMBER_MARKER_SELF_TEST_AUDIT.md` 与离线日志保留在原工作树，没有带入本分支。

## 补丁后离线验收

- 模块测试：`python -m unittest tests.test_number_marker -v`，64 项通过（原 56 项，新增 8 项；旧 renderer 测试改为正式 evidence renderer 测试）。
- 模块检查：`python scripts/check_module.py number_marker`，`MODULE_CHECK_OK`，含 64 项模块测试与 smoke。
- 全仓回归：`python -m unittest discover -s tests -v`，423 项通过。
- 静态检查：`python scripts/check_module.py --all --quiet`，`MODULE_CHECK_OK`；`py_compile` 覆盖仓库 48 个 Python 文件，通过。
- 本补丁仅修改本模块、模块测试、两份模块文档和本审计文件；未运行真实机器人，不能据此声明机械极限、云台方向或 `gimbal.moveto()` 并发风险已被硬件验证。
