# 考场优先级顺序与"第一岔路被红灯钉死"修复

状态日期：**2026-09-18**（决赛当天）。分支：`fix/exam-priority-order`，从 `integration @ a1a0d7c` 新建。

本文只写仓库能证明的事实；所有数字都有可复现命令，**没有任何实车验证**。

## 1. 做了什么

用户要求"把整车程序优先级直接排序好"。落地为两件事。

**顺序的依据需要说准确**：新顺序**不是**要求 PDF 里任务列表的编号顺序。决赛要求 PDF
（`16-Final_Project_Requirements v8`）第 41 页是按 **1 标识 → 2 断线 → 3 绕障 →
4 岔路选绿灯 → 5 岔路避拥堵 → 6 红灯停** 逐条描述任务的，它**没有**规定接管优先级。
本顺序来自用户给出的考场实际遭遇顺序（第一个岔路口是"左红右绿"、第二个要避开拥堵、
第三个随便走），再叠加两条判断：断线恢复是"线消失"才触发的独立场景、优先级高于"选路"；
标识 1~5 是拍照计分、不动作，所以排最后。

### 1.1 注册表接管顺序重排（`task_registry.py`）

```python
MOTION_TASK_CLASSES = (
    GreenJunctionTask,  # 1 第一岔路：左红右绿，往绿灯那侧走（赛题 4）
    ObstacleTask,       # 2 绕障（赛题 3）
    FreeJunctionTask,   # 3 第二岔路：避开拥堵（赛题 5）
    RouteTask,          # 4 断线恢复（赛题 2）
    TrafficLightTask,   # 5 自选地点红灯停绿灯行（赛题 6）
    NumberMarkerTask,   # 6 标识 1~5 拍照（赛题 1）
)
```

`green_junction` 排第 1 的理由：考场第一岔路口是**左红右绿**，而 `traffic_light` 只认颜色
不认位置，排第 1 时会把车按停。

### 1.2 修复：`traffic_light` 不得在"红绿同框"时接管（`traffic_light.py`）

**只改顺序不够 —— 这是本次最重要的发现。**

`traffic_light` 只要 **2 帧**红灯就接管（`red_confirm_frames=2`），而 `green_junction`
要 **3 帧**才确认岔路，所以灯模块**永远赢下这场赛跑**，排到第 5 位也一样。
它接管之后，`coordinator.py` 的红灯否决权（`_step_active` 先问红灯、否决时**不调用**
当前任务的 `step()`）又把岔路模块按停 —— 车被钉死在岔路口。

修复：`TrafficLightConfig.fork_light_competition = True`（默认）。红绿同框时
`TrafficLightDetector.detect()` 返回**无结果**，`TrafficLightTask` 从 IDLE 状态
返回 `NOT_TRIGGERED`，把岔路口让给 `green_junction`（它的 `LampSpotter` 会同时报出
两盏灯各自在画面哪一边）。

- **自选地点的"红灯停绿灯行"（赛题 6）按定义只有一盏灯，不受影响**，红灯停车能力保留。
- 关掉这个开关就退回旧的"红优先"行为，留一个现场临时回退的口子。

## 2. 实测证据（合成帧 + 假底盘，全程离线，未连车/相机）

复现脚本见第 5 节。用真模块（真 `GreenJunctionTask` + 真 `TrafficLightTask` +
真 `TaskCoordinator`），合成 Y 形岔路 + 左红右绿，30fps × 20s = 600 帧：

| 场景 | `green_junction` 被调用 | 它 RUNNING | 零运动帧 | 结果 |
|---|---|---|---|---|
| 修复前：岔路 + 灯模块 | **3** | 1 | **599/600** | 钉死在岔路口 |
| 修复后：岔路 + 灯模块 | **599** | 154 | 4/600 | `take right branch (green light on the right branch)` |
| 对照：只有岔路模块 | 599 | 154 | 4/600 | 同上 |

修复后的"岔路+灯模块"与对照组**逐项一致** → 灯模块不再干扰岔路。

`LampSpotter` 在同一个岔路帧上照旧给出 `[('green','right'),('red','left')]`，
选右边是正确的。

用仓库外层 `_scratch_green/synth3/` 里**已有的**岔路帧跑真检测器：

| 帧 | `traffic_light` 修复前 | `traffic_light` 修复后 | `LampSpotter` |
|---|---|---|---|
| `05_green_lamp_junction` | green | green | `[('green','right')]` |
| `06_two_lamps_green_left` | **red** | **无结果** | `[('green','left'),('red','right')]` |
| `07_two_lamps_green_right` | **red** | **无结果** | `[('green','right'),('red','left')]` |

## 3. 测试

新增 `tests/test_fork_light_competition.py`（11 个用例）—— 这是原本**完全缺失**的覆盖：
整套测试里没有任何一条让真的 `traffic_light` 和岔路模块同时挂在协调器上跑。

**这套测试确实能抓住这个 bug**：把源码的两个修复点用 monkeypatch 还原成旧行为后，
再跑这套测试 → **4 个用例变红**（`test_a_fork_with_two_lamps_never_takes_over`、
`test_the_mirrored_fork_is_also_left_alone`、`test_the_fork_module_keeps_control_at_a_two_lamp_fork`、
`test_the_car_is_not_pinned_for_the_whole_run`）。

`tests/test_traffic_light.py` 里原来钉住旧行为的那条
（`test_red_wins_when_both_colours_visible`）已按新语义改写为
`test_both_colours_visible_reports_no_result`，并新增
`test_red_priority_still_available_when_the_fork_gate_is_off` 覆盖回退开关。

### 命令与结果

解释器固定：`D:\programme\Python\py_project_vscode\robomaster\.venv\Scripts\python.exe`（Python 3.8，cv2 5.0.0 / numpy 1.24.4）

```powershell
python -m unittest tests.test_fork_light_competition      # Ran 11 tests ... OK
python -m unittest tests.test_task_registry_order         # Ran 8 tests  ... OK
python -m unittest discover -s tests                      # Ran 658 tests ... FAILED (failures=7)
```

基线（`integration @ a1a0d7c`，改动前）：`Ran 645 tests ... FAILED (failures=7)`。
失败集合**逐条完全一致**，全部是既有失败，与本次改动无关：

```
test_alignment_survives_one_missing_candidate_frame
test_lowered_view_without_a_valid_base_line_fails_stopped
test_new_route_requires_approach_alignment_and_fresh_handoff_frames
test_oblique_right_angle_candidate_rotates_before_translation
test_perpendicular_candidate_is_valid_for_known_right_angle_gap
test_perpendicular_fragment_confirms_then_aligns_before_translation
test_saved_old_tangent_rejects_old_line_and_accepts_perpendicular
```

645 → 658（+13 个用例），失败 7 → 7 → **无新增失败，也未去修既有失败**。

## 4. 改动范围与未改动清单

`git diff --name-only integration`：

```
task_registry.py
traffic_light.py
coordinator.py                     # 仅注释更正（见下），无逻辑改动
evidence.py                        # 运行记录：console.log + scoring/ + 关键帧封顶（第 7 节）
main.py                            # 加 _TeeStream，把终端状态行同时写进 console.log
tests/test_task_registry_order.py
tests/test_traffic_light.py
tests/test_fork_light_competition.py
tests/test_evidence.py
tests/test_evidence_photos.py
tests/test_main_startup.py
```

`coordinator.py` 只有一处**注释**改动：原来写"`traffic_light` 位置在注册表第 1 位"，
本次重排后已不成立，改为说明"按 name 查找、与位次无关，且正因为如此否决权会把正在
接管的岔路模块按停"。**代码逻辑一行未改**，可用
`git diff integration -- coordinator.py` 核对只有 `#` 开头的行发生变化。

以下文件经 git blob 哈希比对，与 `integration` **逐字节相同**：

`models.py`、`runtime.py`、`config.py`、
`green_junction.py`、`obstacle.py`、`free_junction.py`、`route.py`、
`number_marker.py`

即：六个功能模块里只有 `traffic_light.py` 改了逻辑（它正是出问题的那个）；
公共文件里 `coordinator.py` 只动注释、`evidence.py` 与 `main.py` 为运行记录功能
（见第 7 节），其余模块逐字节未变。

## 5. 未验证事项（必须如实说明）

1. **没有实车验证。** 全部结论来自合成帧 + 假底盘；红灯 HSV 阈值、现场光照、
   相机角度、灯的实际大小与位置都未在真车上确认。
2. 复现脚本放在系统临时目录，**不在仓库里**，需要时按下面参数重建：
   - 岔路帧：蓝色 Y（stem `(330,355)-(330,250)` 粗 22；左右分支到 `(150,120)`/`(520,120)`
     粗 20）+ 左 `(150,80)` 红、右 `(520,80)` 绿，半径 26；背景 210 灰；640×360。
   - 用真 `TaskCoordinator`（假底盘）+ `motion_tasks=(GreenJunctionTask(), TrafficLightTask())`
     跑 600 帧，统计 `green_junction.step` 调用次数与零运动帧数。
3. 红色只看"面积/形状合格的红斑"。若自选红灯停靠点**旁边恰好有一盏绿灯同框**，
   `traffic_light` 会误判为岔路口而**不停车**——这个反向风险未在实车验证。
   现场如果发现"该停不停"，把 `TrafficLightConfig.fork_light_competition` 设为 `False`
   即可退回旧行为（代价是岔路口重新被钉死，二选一）。
4. 顺序重排**不改变** `route` 的既有 7 个离线失败（长断线恢复），它们仍未修。

## 6. 怎么用

```powershell
# 全部离线回归
python -m unittest discover -s tests

# 只跑本次新增的岔路覆盖
python -m unittest tests.test_fork_light_competition -v
```

现场临时回退岔路闸门（改 `traffic_light.py` 的 `TrafficLightConfig`）：

```python
fork_light_competition: bool = False   # 退回旧的"红优先"
```

## 7. 运行记录：终端日志 + 截图分家（2026-09-18）

按用户要求"把终端输出记录到日志里、关键帧截图不要太多（一次限发 20 张）、
得分截图单独放一个文件夹"。改动前 `captures/` 里两类图混在一起，终端状态行
只存在于窗口里、关掉就没了。

现在的目录结构（`captures/` 已被 `.gitignore` 排除）：

```text
captures/run_YYYYmmdd_HHMMSS/
├── console.log      终端上打过的状态行副本（[  12.3s] …），main 用 tee 同时写终端和这里
├── log.csv          每帧一行：帧号/采集时刻/循环时刻/已运行秒数/尺寸/亮度
├── frame_*.jpg      关键帧，**上限 20 张**（DEFAULT_MAX_KEYFRAMES），调试用
├── scoring/         **得分截图专用目录**：交作业只交它
│   └── task_<标签>_<帧号>_<秒>s.jpg
├── summary.json     含 console_log / scoring_directory / max_keyframes 字段
└── report.md        结束时自动生成，得分截图清单写的是 `scoring/task_…` 真实相对路径
```

三条边界（都有测试钉住）：

1. **关键帧封顶 20 张**（`DEFAULT_MAX_KEYFRAMES`，构造参数 `max_keyframes`，
   `<=0` 表示不限制）。超了就不再存，`snapshots` 停在 20。
2. **得分截图不封顶**：老师按 `scoring/` 里的张数算分，封顶就是丢分。
   上限只作用于关键帧，两者走不同代码路径。
3. **关键帧不许进 `scoring/`**，得分截图也不会掉到根目录（目录建不出来时才退回根，
   宁可路径难看也不能"存不上分")。

为什么关键帧留在根目录而不是另开子目录：`tests/test_evidence.py` 等用的是
**非递归** `glob("frame_*.jpg")`，挪走会连带打断既有测试；而得分截图那边
测试本来就用 `rglob`，挪进 `scoring/` 代价最小。

关键帧数量怎么调（`main.build_coordinator` → `build_observers` →
`EvidenceRecorder(directory=...)`）：想多留证据就调大 `snapshot_interval`
（默认 2.0 秒一张）或调大 `max_keyframes`。**注意关键帧是调试用的，
不是算分材料**，所以默认封顶偏保守。
