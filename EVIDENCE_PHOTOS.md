# 证据照片对照表（老师后来明确的 5 组要求）

> 老师原话："The count of the saved images will be the final task score." —— **照片张数就是分数**。
> 所以这张表是"分数清单"：每一行都要真的存出一张带**标注**与**指定文字**的照片。

负责方式（2026-09-17 定的边界）：
* **画框/画圆/写字/去重/落盘/写进 `report.md`** 全部由**集成侧**的 `evidence.py` 负责（一个人写一次，5 个模块口径一致）；
* **"什么时候算完成、往左还是往右"** 由各模块自己决定，模块只在那**一刻**调用
  `evidence.make_evidence_photo(kind, frame, ...)` 并把请求交给证据层；
* 文案模板集中在 `evidence.ANNOTATION_TEMPLATES`，队号 `evidence.TEAM_NUMBER = "10"`
  （老师样例里的 `Team 10` 是样例队号）。

| # | 场景 | 标注形状 | 图上文字（我们队号 03） | 触发时刻 | 触发者 | 分值 |
|---|---|---|---|---|---|---|
| 4.3-a | 红绿灯·红灯 | **圆圈**圈住红灯 | `Team 10 detects a red light and the robot stops` | 红灯连续确认、开始停车保持那一帧 | `traffic_light.py` | 计分事件① |
| 4.3-b | 红绿灯·绿灯 | **圆圈**圈住绿灯 | `Team 10 detects a green light and continues` | 绿灯连续确认、放行那一帧 | `traffic_light.py` | 计分事件②（与①**分别**保存） |
| 5.3 | 岔路选择 | **圆圈**圈住识别到的灯 | `Team 10 detects a green light » left way and left`（只放红灯时写 `a red light … and right`） | 灯的哪一侧确认 + 分支选定 | `green_junction.py` | 必须（且**必须真的转进正确道路**） |
| 6.2 | 拥堵岔路 | **矩形**框住堵路的车 | `Team 10 detects traffic jam » left way and right` | 判定出拥堵侧 + 选定分支 | `free_junction.py` | **15 分** |
| 7.1 | 障碍绕行 | 障碍要看得清（矩形框） | `Team 10 detects obstacle » the left side` | 确认绕行、下第一脚侧移那一帧 | `obstacle.py` | **15 分** |
| 8.1 | 断线恢复 | 标出重新找到的路线（矩形） | `Team 10 finds correct to follow` | 确认重新找到正确路线、准备交回巡线 | `route.py` | **10 分** |
| （既有） | 数字标识 1~5 | 矩形框住标识 | `Team 10 detects a marker with ID of N` | 瞄准锁定、存图那一帧 | `number_marker.py` | 每张计分 |

## 两条容易丢分的地方
1. **红和绿是两件独立的事**：只存红灯那张，绿灯那张就没了 —— 现在两个时刻各存一张
   （`traffic_light.py` 里按事件去重，`traffic_light:red` / `traffic_light:green`）。
2. **"只拍照不算完成"**：老师对 5.3 明确写了"机器人随后必须实际进入正确道路"。
   照片只是证据，**行为**（真的转对、真的绕过、真的恢复路线）是各模块自己的验收标准。

## 去重规则（防重复刷分）
`evidence.EvidencePhoto.event_key` 默认 = 照片标签（例如 `obstacle`、`traffic_light_green`）。
`EvidenceRecorder.save_task_evidence()` 对**同一个事件键只存一张**：第二次收到同键请求时
直接返回 `True`（= "这件事已经有照片了"）而不重复写盘，任务照常完成。
`number_marker` 例外：它按**标识 ID** 分事件（5 个 ID → 最多 5 张），这本来就是不同事件。

## 交付包里也能对上
每次运行结束时 `report.md` 的"**得分截图（Final 按这些图算分）**"小节会逐张列出
墙钟时间、相对时间、标签、**图上那句话**、文件名 —— 交 WORD 时直接照它整理即可。
