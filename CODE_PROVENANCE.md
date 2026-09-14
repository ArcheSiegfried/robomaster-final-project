# Code Provenance — Number Marker Module

审计日期：2026-09-14。范围为 `number_marker.py`、对应测试和模块文档。

| 状态 | Source / version | Original file / function | License | Reuse record | Final destination and requirement fit |
|---|---|---|---|---|---|
| REFERENCE ONLY + INDEPENDENTLY REWRITTEN | DJI RoboMaster SDK `ff6646e115ab125af3207a4ed3df42cc76c795b2` | `examples/05_vision/01_marker.py` 的 callback tuple 与归一化坐标展示 | Apache-2.0 | 参考已核实的 `(x, y, w, h, info)` 字段含义；没有复制示例的类、回调、循环、固定 1280×720、订阅或硬件生命周期代码。像素转换、校验、时间/帧绑定和测试均独立编写。 | `number_marker.py::marker_candidates_from_normalized()`；为 ID 1–5 候选提供全帧像素坐标，同时遵守共享相机/订阅边界。 |
| CURRENT TEAM INTERFACE | 当前仓库 `integration` @ `de781590a9fdbd1b4d41eb3fa11f92f65a334272` | `models.py::GimbalCommand`、`TaskUpdate.gimbal`、`coordinator.py`、`gimbal_output.py`、`task_registry.py` 的 v0.2 contract | 团队仓库现有代码；本分支不改写共享文件 | 使用已有绝对角度 `GimbalCommand(pitch, yaw)`，通过 `TaskUpdate.gimbal` 交给 coordinator 和唯一 `GimbalOutput`；保留注册类名和 `step()` 签名。 | `NumberMarkerTask.step()` 由主流程实际调用；底盘与云台输出都经过现有公共出口，终态由 coordinator 恢复巡线视角。 |
| REFERENCE ONLY | 当前仓库 `integration` @ `de781590a9fdbd1b4d41eb3fa11f92f65a334272` | `route.py::_gimbal_request()` | 团队仓库现有代码；未复制业务逻辑 | 仅核对公共绝对 `GimbalCommand` 的构造与 `TaskUpdate.gimbal` 用法；没有复制 route 的检测、搜索、状态机或参数。 | 证明 number-marker 应复用已有云台输出契约，无需新增 SDK 调用或第二出口。 |
| INDEPENDENT IMPLEMENTATION | 本分支，作者职责：孙宇鹏 | 无上游代码函数 | 项目内个人实现 | ID 过滤、`aimed_ids`/`saved_ids`、严格尺寸门槛、距离/代理选择、双轴 intent、稳定帧、丢失/陈旧安全状态、证据请求/回执和测试均为本模块独立实现。此次垂直环新增 `_target_pitch`、受限 `dt` 积分、现有配置限幅、单点方向符号和瞬态重置，未复制其他任务业务逻辑。 | `evaluate_number_markers()`、`select_target_marker()`、`is_marker_eligible()`、`is_marker_centered()`、`compute_aim_intent()`、`NumberMarkerTask.step()`；对应 Final R03–R10、E01、I01/I02。云台方向和完整流程尚未硬件验证。 |
| LIBRARY API USE | OpenCV / NumPy（仓库既有依赖） | `cv2.rectangle`、`cv2.getTextSize`、`cv2.putText`、数组 copy | 依各库许可证 | 只调用公开 API 生成待保存标注图，没有复制库实现。 | `render_evidence_image()`；保留完整真实场景并添加框和中心文字，不负责写盘。 |

未复制或改写 Practice 5、Practice 6、race-v4、社区仓库或其他成员模块代码。没有把外部工作声明为个人原创，也没有修改 Final Requirements。
