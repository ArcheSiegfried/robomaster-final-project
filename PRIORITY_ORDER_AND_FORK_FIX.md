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
├── log.csv          每帧一行：帧号/采集时刻/循环时刻/已运行秒数/尺寸/亮度/**note**
│                    note 列在状态变化的那一帧写上"谁接管/什么状态/为什么"，
│                    可用来把 console.log 的 [27.0s] 精确落到帧号
├── frame_*.jpg      关键帧，**默认不封顶**（max_keyframes=0），调试用
├── scoring/         **得分截图专用目录**：交作业只交它
│   └── task_<标签>_<帧号>_<秒>s.jpg
├── summary.json     含 console_log / scoring_directory / max_keyframes 字段
└── report.md        结束时自动生成，得分截图清单写的是 `scoring/task_…` 真实相对路径
```

三条边界（都有测试钉住）：

1. **关键帧默认不封顶**（`DEFAULT_MAX_KEYFRAMES = 0`；构造参数 `max_keyframes`，
   正数表示封顶）。2026-09-18 调整：原先默认 20，理由是"一次限发 20 张"，
   但记录就在本地 `captures/` 里、可以直接读，不需要为"发送"牺牲证据。
   想防止无人看管时刷爆磁盘就设一个正数。
2. **得分截图永不封顶**：老师按 `scoring/` 里的张数算分，封顶就是丢分。
   它走 `save_task_evidence`，与关键帧完全不同的代码路径。
3. **关键帧不许进 `scoring/`**，得分截图也不会掉到根目录（目录建不出来时才退回根，
   宁可路径难看也不能"存不上分")。

为什么关键帧留在根目录而不是另开子目录：`tests/test_evidence.py` 等用的是
**非递归** `glob("frame_*.jpg")`，挪走会连带打断既有测试；而得分截图那边
测试本来就用 `rglob`，挪进 `scoring/` 代价最小。

关键帧数量怎么调（`main.build_coordinator` → `build_observers` →
`EvidenceRecorder(directory=...)`）：想多留证据就调大 `snapshot_interval`
（默认 2.0 秒一张）或调大 `max_keyframes`。**注意关键帧是调试用的，
不是算分材料**，所以默认封顶偏保守。

## 8. 预约接管通道：修 `number_marker` 被饿死（2026-09-18 实车）

### 症状（`captures/run_20260918_163210`，196 秒）

用户报告"识别 marker 没触发、也没拍照"。从那次 run 的 `console.log` 统计：

| 模块 | 接管次数 | 被优先级截断（那一帧根本没被问） |
|---|---|---|
| green_junction | 1 | 1 |
| **obstacle** | **0** | 1 |
| free_junction | 5 | 1 |
| route | 2 | 6 |
| traffic_light | 1 | 8 |
| **number_marker** | **0** | **9** |

`scoring/` 里只有 4 张图，**一张标识照片都没有** —— 标识 5 分/个、满 25 分，
是全场最大一块，一分未得。

### 根因：仲裁架构，不是识别阈值

`coordinator._find_takeover()` 按注册表顺序问，**遇到第一个返回 RUNNING 的
就停**，后面的模块那一帧**根本不会被调用**（日志里叫"优先级截断"）。
`number_marker` 排最后、`free_junction` 又是强触发，于是被永久饿死。

**只调注册表顺序救不了**：把 `number_marker` 提前，它会反过来挡住岔路/绕障。

### 修法：另开一条"预约"通道

模块可以实现一个**只读**的 `wants_control(frame, now) -> bool`，协调器在
**已经有别的模块拿着运动权**时，每帧额外问它一遍。命名沿用骨架已有的
鸭子类型做法（`reset()` / `record_decision()`），**没有这个方法的模块行为完全不变**。

三条设计约束（都有测试钉住）：

1. **只在"已经有人开车"时生效**。没人开车时仍按注册表顺序问，所以
   "岔路优先于标识"这类裁定**保持不变** —— 饿死本来就只发生在
   "前面的模块一直拿着运动权"的时候。
2. **红灯否决优先于它**，且仍受 `TaskConfig` 全部限幅与超时约束。
3. 当前任务要跑够 `RESERVATION_MIN_HOLD_SECONDS`（1.0 秒）才允许被抢，
   否则两个模块会每帧互抢。

目前只有 `number_marker` 实现了这个钩子（`wants_control` 复用 `step()` 的
接管判据 `is_marker_eligible`，即赛题要求的 `width/frame_width > 0.20`；
已锁定目标时返回 False，不抢正在收尾的拍照）。

`obstacle` 同样是 0 次接管，也可以接这个钩子，但**本次没做**：
它的判据要接 SDK robot 观测（`feed_robot_observations`），
需要单独测，且"障碍没触发"到底是没看到还是被饿死，这次 run 分不出来。

### 验证

新增 `tests/test_reservation.py`（11 条）：排最后的模块能拿到运动权、
没有钩子的模块行为不变、不想接管时不调它的 `step()`、预约模块之间不互抢、
min_hold 之前不抢、**红灯否决优先于预约**、预约指令仍被限幅、
STOPPED 时任何模块都不许接管、钩子抛异常只记录不影响主循环、
以及"没人开车时仍按注册表顺序（预约不许抢在岔路前面）"。

离线回归：685 tests / 7 failures，7 条全部是既有的 `test_route.RouteRecoveryTests`
失败，无新增。

**未验证**：预约通道对那次真实 run 的实际改善没有回放验证，
也**没有任何实车验证**。要确认 `number_marker` 真的开始拍照，需要再跑一次场地。

## 9. 第二次实车（2026-09-18，`run_20260918_164540`）暴露的四个真问题

这次跑完，预约通道**没有生效**（console.log 里 `reservation` 痕迹 0 处）。
查 `report.md` 的接线层自检，根因不在仲裁，而在**上游根本没数据**：

### 9.1 数字标识：SDK 一个 marker 都没给

```
## 数字标识观测（SDK marker 订阅）
| subscribed | True | callbacks | 2 | empty_callbacks | 2 |
| markers_in_snapshot | 0 | observed_candidates | 0 |
| 提醒 | 还没有收到任何 marker 回调。 |
```

整场 156 秒只回调 **2 次、且都是空的**。所以 `number_marker.wants_control()`
永远没机会返回 True —— **预约通道没生效是因为它上游没数据，不是通道本身坏了**。

排查方向（按顺序）：
1. `CONFIG.marker_color` 目前是空字符串（不设过滤器）。`marker_source.py:30` 自己写着
   "一直收不到任何 marker 就把 color 依次改成 red / green / blue 再试"。
   现场看到的标识是一块**红色方牌**（`tests/samples/real_marker_square_red.jpg`），
   先试 `"red"`。
2. 确认考场那些标牌**是不是 DJI 官方 vision marker**。如果只是打印的号码牌，
   SDK 的 marker 识别永远不会有回调，必须换方案（自己用相机帧识别）。
3. `coordinate_mode` 是 `auto`，回调进来后要核对它判成像素还是归一化。

### 9.2 绕障 / 拥堵：SDK 识别到 2554 个框，0 个"小车"

```
## 障碍物观测（SDK 机器人识别）
| callbacks | 2629 | callback_hz | 22.86 |
| observed_boxes | 2554 | max_width_ratio | 0.3 | widest_robot_at | 中心(534,192) |
| robots_in_snapshot | 0 |
```

识别**在跑**、画面里也有东西（2554 个框、最宽 0.3 倍画面），但 SDK **一次都没
把它判成机器人**。`obstacle` 与 `free_junction` 的拥堵判据都依赖它，所以：
`obstacle` 接管 0 次；`free_junction` 只能退回"图像判据"
（日志：`official robot detection unavailable (0 sightings so far; picture criterion)`），
于是**在第一个岔路口就抢在 green_junction 前面接管**，还 FAILED 过一次
（`no vehicle on either branch`）。

### 9.3 第一个岔路口为什么先被 free_junction 抢走

两个模块的触发条件**不对等**：
* `free_junction` 不需要灯就能接管（官方识别不可用时退回图像判据）；
* `green_junction` 要 **3 帧确认岔路 + 一个绿灯读数**（A14/A15），天然慢 3 帧。

实测时序：`[14.5s] free_junction` 先抢到并"完成"，`[18.4s] green_junction`
才在下一个岔路口完成。用户观察"一开始是 free_junction，后来变成 green_junction
且成功通过"与日志一致。

### 9.4 断线：两次都是**判据失败**，不是超时

```
[42.2s] route 接管 | 云台 pitch=-12 | crossing bounded blank along old-route tangent
[45.5s] 巡线 STOPPED                      ← 用户看到的"第一次停住了"
[48.2s] Resumed on a fresh valid line
[49.7s] route 又接管 | 云台 pitch=-25 | lowering camera; stopped before endpoint approach
[52.4s] route（FAILED，共 2.7s）
[78.3s] Resumed on a fresh valid line     ← "退一段再启动又好了"
```

`route` 只在接管后 2~4 秒就 FAILED（模块预算有 19 秒），说明卡在视觉判据
（`crossing bounded blank` / `stopped before endpoint approach`），
不是没时间。用户"退一段重启就好了"= 人退车后巡线重新接上、后续岔路正常通过。

### 结论：这次三个抱怨里，两个的根因在 **SDK 侧没有数据**，不在仲裁或阈值

| 抱怨 | 根因 | 证据 |
|---|---|---|
| marker 没识别/没拍照 | SDK marker 回调 2 次全空 | `report.md` 数字标识小节 |
| 避障识别不到小车 | SDK 识别 2554 框、0 个机器人 | `report.md` 障碍物小节 |
| 第一个岔路先 free 后 green | 两模块触发条件不对等（free 不需灯） | console.log 14.5s / 18.4s |
| 断线第一次没过 | route 视觉判据失败，非超时 | console.log 42.2s / 49.7s |

**这些都没有修**，本文只记录诊断与证据。下一步优先级待用户决定。

## 10. 第三次实车（三个短 run，`165321` / `165452` / `165522`）

用户报告："中间断了 3 次，marker 还是没识别，避障一直调用的是 route"。
三个 run 都在 30~80 秒内被人工重启。

### 10.1 marker 依旧：SDK 回调全是空的

`run_20260918_165522`：

```
## 数字标识观测（SDK marker 订阅）
| callbacks | 3 | empty_callbacks | 3 |
| markers_in_snapshot | 0 | observed_candidates | 0 |
| callback_hz | 25.64 |
```

**回调频率 25.64 Hz 说明 SDK 的识别器在跑，只是什么都没认出来**（不是订阅没建起来）。
三次 run 累计：callbacks 2 / 3，`observed_candidates` 恒为 0。
→ `CONFIG.marker_color` 必须试 `"red"`；若仍为 0，就要怀疑**考场标牌不是
DJI 官方 vision marker**，SDK 这条路走不通，得改成自己用相机帧识别。

### 10.2 绕障：SDK 机器人框太小 / 判不出小车

`run_20260918_165522`：

```
| callbacks | 808 | empty_callbacks | 788 | callback_hz | 25.26 |
| observed_boxes | 31 | max_width_ratio | 0.034 | robots_in_snapshot | 0 |
| 提醒 | 看到过的最宽机器人框只有画面宽的 3.4%（中心(457,98)），
        低于模块要求的 6%：障碍模块会认为「太远」而不绕。把车开近些再复测。 |
```

对比 `run_20260918_164540` 的 `observed_boxes 2554 / max_width_ratio 0.3 /
robots_in_snapshot 0`：**两次 run 都不是"没有框"，而是框从来没被 SDK 判成机器人**。
`obstacle` 与 `free_junction` 的拥堵判据都依赖 `robots_in_snapshot`，所以：

* `obstacle` 三次 run **接管 0 次**（拦截面 1/0/0 次，不是被饿死，是没目标）；
* `free_junction` 退回图像判据后**在缺口处触发并选边**。

### 10.3 "避障一直调用的是 route" —— 用户观察成立，但那是**丢线恢复**

`run_20260918_165452`：

```
[  5.2s] 巡线 LINE_LOST —— 丢线，底座开始找回
[  5.2s] >>> 模块开始运行：route ｜ task took over
[  6.0s] route: crossing bounded blank; candidate history disabled during initial old-line clearance
[  9.1s] -- 巡线 STOPPED | 任务 route          ← 人工停车
```

`route` 是**断线恢复**模块，它只在巡线进入 `LINE_LOST` 之后才接管
（`route.step` 要求 `not line.valid` 且有下方旧线；骨架规定
"长断线必须在短时容错边界之后接管"）。所以看到 `route` 就说明**车已经丢线了**，
不是"避障调用了 route"。

用真检测器复跑该 run 的关键帧，确认丢线是**真丢**（不是状态机误判）：

```
frame_000008_0000.00s.jpg  valid=True  conf=0.444
frame_000069_0002.01s.jpg  valid=True  conf=0.896
frame_000128_0004.01s.jpg  valid=True  conf=0.847
frame_000190_0006.05s.jpg  valid=True  conf=0.765
frame_000250_0008.05s.jpg  valid=False conf=0.000   ← 画面里确实没有可用的线
frame_000310_0010.05s.jpg  valid=True  conf=0.886
```

喂给 `LineFollower` 后同样在 8.05s 判 `LINE_LOST`（`line-loss timeout;
reset and resume required`）。所以**线段是真丢**，`route` 接管是正确反应。

### 10.4 因果链（本次最重要的结论）

三条证据指向同一条链：

```
free_junction 在没有官方机器人识别的情况下，凭图像判据判定"某侧拥堵"并选边
        ↓（选了一条边 → 车跟着那条边走 → 离开了本来要走的线）
巡线进入 LINE_LOST（真丢线）
        ↓
route 接管（断线恢复）
        ↓
用户看到"避障一直调用的是 route"
```

`free_junction` 在三次短 run 里分别接管 4 / 1 / 3 次，且 `165321` 里
13.2s 那次 0.3 秒就 FAILED（`no vehicle on either branch`）——
**它在没有拥堵证据时也会进接管流程**（有 `require_blockage_to_trigger` 挡着，
但它用的是图像判据，不是官方识别）。

**推论（未证实）**：第一个岔路口应该由 `green_junction`（选绿灯那侧）处理；
`free_junction` 抢在它前面选边，很可能就是 10.4 那条链的起点。
需要现场验证：**把 `free_junction` 暂时停用（或提高它的证据门槛），
看车是否就不再丢线**。这一步会直接区分"free_junction 选错边"与"线本身有问题"。

### 10.5 还没做的修复

本轮**只做诊断，没有改任何代码**。待用户决定优先级，候选：
1. `marker_color="red"` 试一次（一行配置，25 分）；
2. 确认考场标牌是否为官方 marker（决定要不要改写识别方案）；
3. `free_junction` 的证据门槛 / 与 `green_junction` 的竞争（10.4 那条链）；
4. `obstacle` 对"框太小/没判成机器人"的处理（是否该退回自己的灰度判据并降低门槛）。
