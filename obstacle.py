"""障碍检测与绕行（WP5 / Issue #5）。

状态：**已实现**（离线合成帧 + 真实场地画面的离线测量验证；未再次上真车）。

职责边界（违反会被 tests/test_task_contract.py 直接拦下）：
  * 只接收主流程给的 FramePacket，绝不自己开相机或视频流。
  * 只返回 VisualDetection / TaskUpdate / MotionCommand，绝不直接调用 SDK，
    也不接触底盘的唯一运动出口。
  * step() 必须立刻返回：绕行动作被拆成"每次一小步"的状态机。

必须实现的接口（签名已冻结，不要改）：
    class ObstacleTask:
        name = "obstacle"
        def step(self, frame: FramePacket, now: float) -> TaskUpdate

返回约定：
  * 没有障碍   -> TaskUpdate(TaskStatus.NOT_TRIGGERED)
  * 绕行中     -> TaskUpdate(TaskStatus.RUNNING, motion=MotionCommand(...))
  * 绕行完成   -> TaskUpdate(TaskStatus.COMPLETED)
  * 失败/超时  -> TaskUpdate(TaskStatus.FAILED)
  一旦返回 RUNNING，就必须持续 RUNNING，直到 COMPLETED 或 FAILED。

交给骨架处理、本文件**不重复实现**的事：
  * 命令限幅、nan/inf 置零、20 秒硬超时、视频中断、人工 SPACE/R 打断。

==================== v3：实车测试报告（2026-09-14）的三条根因 ====================

现象：车沿蓝线走，碰到障碍**直接停住，完全没有绕行动作**。
报告用 426 张现场画面做了像素级测量，根因不是"没检测到"，而是下面三条叠加：

【P0-1】相机自己的橙色件常驻在检测区里
    ROI 原来是 y 0.45~0.95（画面下方），而镜头前方正下方中间恰好是机器人
    自己的橙色件：每帧在 ROI 里贡献 900~1700 个橙色像素，最大色块 73×19，
    area_ratio ≈ 0.0125 —— 离 0.03 的阈值只差 2.4 倍，随姿态/光照浮动。
    更糟的是人的皮肤 H 也在 5~25，和橙色撞在一起：426 帧里被判为障碍的 4 帧
    全部是人的手/手臂（confidence 最高 0.86）。
    -> 修法：**把 ROI 下沿从 0.95 抬到 0.75**，车体自己直接被关在 ROI 外面；
       同时默认**不用颜色判据**（改用不依赖颜色的结构判据），并加一道
       "皮肤色剔除"，把 H 5~25 且低饱和的候选直接扔掉。

【P0-2】从"看到"到"第一脚侧移"要 0.35 秒，中间还有一段主动零速
    原来：连续 3 帧确认 ≈0.15s + HOLD_BEFORE_GO=0.20s 主动下发三轴全零。
    以巡线 0.32 m/s 计，这 0.35 秒车又前冲约 11 cm；再叠上障碍挡住蓝线后
    巡线 0.28 秒的降速滑行，一共约 15 cm 是在"第一脚侧移之前"走掉的。
    操作者看到的就是"车停住了，没绕"。
    -> 修法：**HOLD_BEFORE_GO 改成 0，确认完成的当帧立刻下发侧移**；
       CONFIRM_FRAMES 3 -> 2。有测试钉死："确认那一帧必须已经在下发非零 lateral"。

【P1-3】检测区下沿离车头太近，提前量不够
    原来 ROI 到 0.95，和巡线 ROI（0.54~0.96）压在同一块地面上，障碍进 ROI
    时已经贴到车头；而且越近 area_ratio 越大，越容易被 MAX_OBSTACLE_AREA 拒掉
    ——"越该动作的时候越可能不动作"。
    -> 修法：ROI 改成**靠上、靠窄**的 (0.25, 0.30, 0.75, 0.75)，用更远的提前量
       换更晚的确认；并把"太近"交给 ROI 下沿处理，面积上限只当兜底。

======================= 官方信息与由此推出的假设 =======================

【官方信息】障碍是**一辆静止的小车**（另一台 RoboMaster 同型车），不会移动。
【官方信息】没有人给过赛道宽度、道具精确尺寸；下面按"同型车"估算。

  假设 1：障碍车与我们车同型，长约 30~32 cm、宽约 24 cm、高约 27 cm。
          它是立着的立体物 -> 轮廓有硬边缘 -> **结构判据能抓到，不需要知道颜色**。
  假设 2：它停在蓝线上 -> 会把蓝线挡住 -> 绕过去之后线会重新出现（SEEK 段靠这个收尾）。
  假设 3：赛道够宽，往一侧让开约 41 cm 能完全错开两辆车。
  假设 4：绕完允许本模块主动找线（SEEK）；SEEK 失败就停车报 FAILED。

  由尺寸推出的三个距离（写死在参数区，改参数时要一起重算）：
      OUT  ≈ 41 cm = (24 + 24) / 2 + 余量 10   完全错开车宽
      PASS ≈ 44 cm = (30 + 32) / 2 + 余量 10   完全越过车长
      BACK ≈ 24 cm，余下的交给 SEEK 边挪边找线

  障碍是静止的，所以：它不会自己让开，也不会追上来；但"绕完还看到同一个障碍"
  就说明**没绕过去**，这时连绕上限（MAX_CONSECUTIVE_DODGES）会停下来交给人看。

仍未验证（不得当作已通过）：
  * 上面三个距离是照"同型车"**算**出来的，**没有实车量过**（道具宽度、场地宽度都没量）；
  * 障碍车如果是**浅色**、和浅色地面差别很小，结构判据的边缘阈值要现场调
    （用 `python obstacle.py 真车照片.png` 看掩膜）；
  * `MotionCommand.lateral` 的正负号必须在**架空**状态确认（MODULE_GUIDE 明确要求）；
  * 报告的 426 张现场画面不在仓库里，本文件**没有**对它们跑过回归；
  * v4 的三处修改（断档检测、分段推进、PASS 加长）**一次真车都没试过**。

离线调参（不用连车、不用连相机）：
    python obstacle.py 样图.png [输出掩膜.png]
"""

from typing import Optional

import time

import cv2
import numpy as np

from config import CONFIG
from models import (
    FramePacket,
    MotionCommand,
    TaskStatus,
    TaskUpdate,
    VisualDetection,
)

KIND = "obstacle"


# =====================================================================
# 1. 可调参数（假设全在这里；改这一节就够了）
# =====================================================================

# 【v10】触发条件：**视野里有车 且 蓝线消失**（用户要求）。
#
# 为什么要这样：原来"一看到车就开始横移"，车会在离障碍还很远的地方就早早开始绕，
# 结果每次都**正好停在障碍机器人前面**，然后才手忙脚乱地躲。
#
# 现在等到**蓝线被障碍挡住**才动手 —— 蓝线在近处消失 = 车已经开到障碍跟前了，
# 这本身就是最可靠的"距离检测"（比任何像素估距都稳），
# 而且必须**同时**看到车（SDK 机器人识别），避免把"线断了"误当成"有障碍"。
TRIGGER_NEEDS_LINE_LOST = True

# 触发之后的动作是**开环**的：OUT -> PASS -> BACK -> 回到线上，
# 过程中**不再看蓝线、也不再看障碍框**（用户要求），只管绕过去。
# 只有最后 SEEK 段才重新看线，用来确认"线回来了"。

# --- 绕行方向 ---
DODGE_SIDE = "left"          # 默认往左绕（障碍基本在正中间时用这个）
# 【v8】PDF item 3 要求 "observe carefully to choose go around it from left or right"，
# 快照里还要写清是从哪边绕的 —— 所以不能永远往左。
# 规则：障碍明显偏左就从右边绕、明显偏右就从左边绕（离它远一点更安全）；
# 偏离不到 SIDE_DEADBAND 就按 DODGE_SIDE 的默认值。
SIDE_DEADBAND = 0.25         # 障碍横向偏移小于这个比例就算"在正中间"

# --- 尺寸（米）—— 全部来自 DJI 官方技术参数，不是估的 ---
#   https://www.dji.com/cn/robomaster-ep/specs   「机器人 - 尺寸」：
#       步兵机器人：320×240×270 mm（长 × 宽 × 高）
#       工程机器人：410×240×330 mm
#   障碍是**同一型号**的小车，所以两车长宽相同。
OUR_LENGTH_M = 0.32          # 我们车长（步兵）
OUR_WIDTH_M = 0.24           # 我们车宽
OBSTACLE_LENGTH_M = 0.32     # 障碍车长（同型号）
OBSTACLE_WIDTH_M = 0.24      # 障碍车宽
CLEARANCE_M = 0.10           # 每个方向留的余量

# --- 速度（m/s）。骨架还会再裁一次（横移<=0.25、前进<=0.30），这里留一点余量 ---
SIDE_SPEED = 0.24            # 横移让开的速度
FWD_SPEED = 0.25             # 越过障碍时的前进速度（要跑 0.74 m，太慢会拖长时间）
BACK_SPEED = 0.20            # 绕完往回收的速度
SEEK_SPEED = 0.16            # 找线时往回挪的速度

# --- 每一段的最长时间（秒）---
# HOLD_BEFORE_GO = 0 是 P0-2 的修法：确认完立刻侧移，不再先停车看清。
# 实测过：那 0.20 秒的零速会让车多前冲约 6 cm，正是"看起来像停住了"的来源。
HOLD_BEFORE_GO = 0.00
#
# 【v11 关键修正】前进距离必须按"**我们车尾也过去**"算，不能只算车头。
#   车头刚过障碍时车尾还在它旁边，一收回来就蹭上 —— 用户实测就是这么蹭的。
#
#     需要前进的距离 = 障碍车长 + 我们车长 + 余量
#                    = 0.32 + 0.32 + 0.10 = 0.74 m
#       -> T_PASS_TIME = 0.74 / 0.25 = 2.96 s，取 3.00 s（≈0.75 m）
#     （v10 及以前用的是 (32+32)/2 + 余量 ≈ 0.44 m —— **短了 30 cm**，车尾过不去）
#
#   横移同理，要完全错开两车的**宽度**：
#     需要横移的距离 = (我们车宽 + 障碍车宽) / 2 + 余量
#                    = (0.24 + 0.24) / 2 + 0.10 = 0.34 m
#       -> OUT 段 0.24 × 1.70 ≈ 0.41 m  ✓（留了额外余量，更稳）
#
#   回程不用一次收完：BACK 收回 0.20 × 1.20 ≈ 24 cm，剩下的交给 SEEK 边挪边找线。
T_OUT_TIME = 1.70            # 第 1 段：往侧面让开（0.24 × 1.70 ≈ 0.41 m ≥ 0.34）
T_PASS_TIME = 3.00           # 第 2 段：往前越过（0.25 × 3.00 ≈ 0.75 m ≥ 0.64）
T_BACK_TIME = 1.20           # 第 3 段：往回收（0.20 × 1.20 ≈ 0.24 m）
SEEK_TIME = 2.50             # 第 4 段：主动找线，最多找这么久
# 【v8】原来 1.5s —— 实车记录里"绕完 1.5s 内没找回线"是现在最贵的失败模式
# （2026-09-16 16:59 那次 5 次绕行里有 1 次要人按 SPACE 救）。
# 放宽到 2.5s：0.16 m/s × 2.5s ≈ 40 cm 的搜索范围。
#
# 四段合计 1.70 + 3.00 + 1.20 + 2.50 = 8.40 s，所以总上限要放在它之上。

# 【官方信息】障碍是**一辆静止的小车**（另一台 RoboMaster 同型车）。
# 尺寸全部来自 DJI 官方技术参数页（步兵机器人 320×240×270 mm），
# 上面 OUR_/OBSTACLE_ 那六个常量就是它；三个动作距离都由它们算出来，不是拍脑袋。
#
# 障碍是**静止的**，所以它不会自己让开，也不会追上来；
# 但也意味着"绕完还看到同一个障碍"就说明**没绕过去**，
# 这时候连绕上限（MAX_CONSECUTIVE_DODGES）会把它停下来交给人看。

# --- 硬性保护 ---
# 四段合计 8.40 s（1.70+3.00+1.20+2.50），总上限放在它之上，留出帧率波动的余量。
MAX_TOTAL_TIME = 11.00       # 整段（含找线）最长 11 秒，超了立刻停车报 FAILED
CONFIRM_FRAMES = 2           # 连续 2 帧确认（原来 3 帧，见 P0-2）
LINE_CONFIRM_FRAMES = 2      # 找线时连续 2 帧看到蓝线，才算找回来了
REARM_SECONDS = 5.00         # 一次绕行结束后，这段时间内不再重新接管
MAX_CONSECUTIVE_DODGES = 2   # 同一段路最多连绕 2 次，第 3 次直接停车要人来看
CLEAR_SECONDS = 3.00         # 画面里连续这么久没有障碍，就认为换了段路，连绕计数归零

# 【v4 · 最重要的一条】两次 step() 之间隔这么久，就认为"我们被从外面踢掉了"。
#
# 实车测试报告（2026-09-15）的时间线里出现过这样的记录：
#     00:04.9  TASK_ACTIVE  obstacle  RUNNING    task took over
#     00:04.9  RELEASING    obstacle  COMPLETED  task completed
# 整个绕行动作最短也要 4.5 秒（1.7 + 1.6 + 1.2 + 找线），**0.1 秒完成在物理上不可能**。
#
# 根因：协调器在视频中断 / 人工按键 / 硬超时时会把控制权拿走，但**不会通知模块**。
# 模块的 stage 还停在 OUT/PASS 上；等车被人工恢复、模块再次被问到时，
# 每一段的"已用时间"早就超过各自时限，于是**一帧跳一段**（OUT→PASS→BACK→SEEK），
# 看到线就报 COMPLETED —— 造出"绕过去了"的假象，实际上一动没动。
#
# 修法：自己发现这次断档；隔得太久就作废重来，绝不从半路接着走。
# 1.0 秒的依据：被踢掉之后巡线是 STOPPED，需要人工按 SPACE 才恢复，
# 实测那一次断档是 2.6 秒；而正常绕行时每帧都会调用一次 step()（约 0.03 秒一次）。
STALE_STEP_GAP = 1.00

# 断档之后"先停一脚"的时长：这段时间内保持控制权、原地不动，然后再重新判断。
# 为什么不一帧就接着绕：被从外面拿走控制权的瞬间，车可能已经不在原来的位置/姿态，
# 先停稳、看清画面再动更安全（审阅人 2026-09-16 的建议）。
RECONFIRM_HOLD = 0.10

# --- 检测 ROI（画面比例 x1, y1, x2, y2）---
# 【P0-1 / P1-3 的核心修改】原来是 (0.20, 0.45, 0.80, 0.95)：
#   * 下沿 0.95 会把镜头正下方、机器人自己的橙色件收进来（实测每帧 900~1700 px）；
#   * 下沿太低 = 障碍贴到车头才进 ROI，没有侧移的余地。
# 现在改成靠上、靠窄：车体自己落在 ROI 外面，障碍也能更早被发现。
OBSTACLE_ROI = (0.25, 0.30, 0.75, 0.75)

# 形状判据（比例都相对 ROI）。量的是**外接框**，不是填充面积 ——
# 障碍常常被画面下边缘切掉一块，轮廓是开口的，填不出面积；外接框不受影响。
MIN_OBSTACLE_AREA = 0.03     # 外接框至少占 ROI 面积的 3%，更小的当噪声
MIN_OBSTACLE_WIDTH = 0.10    # 宽度至少占 ROI 宽度的 10%
MIN_OBSTACLE_HEIGHT = 0.22   # 高度至少占 ROI 高度的 22%：立着挡路的东西才有这个高度
MIN_DENSITY = 0.05           # 框内"有东西"的像素占比 >= 5%：排除稀疏散点
# 上限只当兜底。【v10 放宽到 0.95】：贴到车头的障碍会占满大半个 ROI，
# 原来 0.60 的上限会把最该绕的那一台车拒掉。
MAX_OBSTACLE_AREA = 0.95
MAX_OBSTACLE_WIDTH = 0.95

# ---------------------------------------------------------------------
# 【v5】实车误触发（2026-09-15 16:15 那次运行）之后加的三道闸
# ---------------------------------------------------------------------
# 实测：26 张现场截图里有 11 张被判成"有障碍"（42%），车就在正常巡线中反复
# 左移-回正。报告给了三条关键线索，下面三道闸一一对应。
#
# 闸一：**贴边淘汰**（只判左沿 / 右沿）。
#   11 个误判框里有 9 个压在 ROI 的侧沿或下沿上。几何理由：真障碍是"挡在路中间"的，
#   被 ROI 侧边切开的，多半是背景、场地边界或者车体自身的边缘。
#
#   【v10 改动】**下沿不再淘汰**：触发条件已经变成"蓝线消失"，
#   而蓝线消失恰恰意味着障碍已经贴到车头 —— 那时候障碍**必然**压住 ROI 下沿。
#   原来那条"下沿也淘汰"会把最该绕的那一台车拒掉。
REJECT_EDGE_TOUCHING = True
EDGE_TOUCH_MARGIN = 0.02     # 距边界 2% 以内就算"贴着"

# 闸四：上沿留白。【v10 默认关掉（=0）】。
#   v6 加它是因为当时"一启动就避障"，那些误判框全贴在 ROI 顶边（背景）。
#   现在触发条件多了"蓝线必须消失"这一条，背景结构再也过不来；
#   而**贴到车头的障碍顶边本来就会冲出 ROI 上沿**，留着它反而拒掉真障碍。
TOP_INSET_RATIO = 0.00       # 0 = 不判上沿

# 闸五（v6 新增）：**位置判据**。真障碍挡在线上、就在车正前方，所以
#   纵向应该出现在走廊下半部（近处），横向不该偏出 ROI 中间一半。
#   实测那三帧：两帧在 ROI 上半部（中心 0.15 / 0.16），第三帧横向偏了 65%。
MIN_LOWER_RATIO = 0.40       # 框中心必须在 ROI 高度 40% 以下
MAX_CENTER_OFFSET_RATIO = 0.50   # 横向偏移不能超过 ROI 半宽的 50%

# 闸二：**抬高置信度门槛**。实测误判的 confidence 是 0.38~0.85，最低 0.38，
#   而原来门槛 0.35 —— 形同虚设。取 0.50 的依据：
#   那批误判里"没贴边"的只有两帧，分别是 0.39 和 0.61；
#   门槛放在 0.50 就能吃掉 0.39 那一帧，又不像 0.60 那样会误伤
#   "真实但稍远、稍小"的障碍（那种的打分本来就在 0.6 附近）。
MIN_CONFIDENCE = 0.50

# 启动预热（审阅人建议 3；**默认关**，理由见文件末尾"v6"一节）：
#   启动后这么久之内不接管。实测那 3 次误触发都在 t≈2.0s，
#   把它设成 3.0 就能全部压掉。默认关是因为闸四/闸五已经把那三帧挡掉了，
#   而预热期会顺带压掉"启动时真有一台车挡在前面"这种正当情况。
STARTUP_GRACE_SECONDS = 0.00

# 闸三：**框里必须真的有"东西"**：既不像地面、也不像蓝线。
#   关掉颜色判据之后只剩"硬边 + 连成块"，对**高对比背景**没有区分力；
#   但"一块和地面同色的浅色结构"是可以识别的——它的内部几乎全是地面色。
#   浅色地面 = 低饱和 + 高亮度；蓝线 = 巡线那套 HSV 区间。
#   门槛取得很低（5%），因为它的作用只是"框里不能**全是**地面色"；
#   取高了会误伤浅色障碍（浅色车只有轮子和底盘阴影是深的）。
MIN_OBJECT_RATIO = 0.05      # 框内"非地面非蓝线"的像素占比要 >= 5%
FLOOR_S_MAX = 60             # 饱和度 <= 60 且
FLOOR_V_MIN = 140            # 亮度 >= 140 -> 算"浅色地面"

# 闸六（v7 新增）：**平坦的中性色区域不是障碍**。
#   审阅人给了 29 次运行、589 张截图的逐帧记录：一共 74 帧被判成"有障碍"
#   （那一批里没有真障碍）。闸四干掉 63 帧（贴在 ROI 上沿的背景），闸五干掉 5 帧，
#   剩下 6 帧的共同点是：**全是平坦的中性色区域** ——
#     145456 12.12s  中灰 90.6%  meanV=104
#     123656 32.14s  亮白 59.4% + 中灰 40.6%  meanV=153
#     131607 0.0/2.03/4.06s 与 131842 4.02s  亮白为主  meanV=211~226
#   真障碍不一样：**要么有结构**（轮子、云台、接缝、明暗面 —— 轮廓里面还有硬边缘），
#   **要么是彩色道具**（饱和度高）。一块平的灰/白区域两样都没有。
#
#   判据：框内部（躲开它自己那圈轮廓）还有硬边缘，**或者**框内平均饱和度够高。
#   两者都不满足 -> 拒。
MIN_INTERNAL_EDGE_RATIO = 0.01   # 框内部硬边缘像素占比的下限
INTERNAL_INSET_PX = 8            # 往里收这么多像素，躲开物体自己的轮廓
MIN_MEAN_SATURATION = 70         # 或者框内平均饱和度 >= 70（彩色道具）

# 去噪用的形态学核（函数内部会自动取奇数）
OPEN_KERNEL = 3
CLOSE_KERNEL = 5

# 多个候选一起出现时，谁更像"该绕的那一个"（打分权重）
SCORE_AREA_W = 1.0           # 大块优先
SCORE_LOWER_W = 0.8          # 位置越靠下（离车越近）优先
SCORE_CENTER_W = 0.6         # 越在正前方优先
SCORE_WIDTH_W = 0.6          # 越宽（越挡路）优先
SCORE_TOTAL_W = SCORE_AREA_W + SCORE_LOWER_W + SCORE_CENTER_W + SCORE_WIDTH_W

# ---------------------------------------------------------------------
# 检测模式一：结构检测（**默认开启**，不依赖障碍颜色）
# ---------------------------------------------------------------------
# 思路：障碍（另一台车、箱子、挡板）立在浅色地面上，轮廓一定是很"硬"的边缘；
#       而影子、反光是渐变的，边缘很"软"。所以找硬边缘 -> 连成块 -> 套形状判据。
#       什么颜色都能抓，不依赖"猜道具颜色"。
#
# 三个踩过的坑（都会让另一台车漏检或让地板误触发，别再改回去）：
#   * Canny 出来是 1 像素宽的曲线，闭运算不会把它变粗，必须先 dilate 加粗；
#   * 蓝线是竖着穿过障碍的，如果把蓝线像素从边缘图里"挖掉"，会把障碍的轮廓
#     切成两半、连不成块。正确做法是**先把蓝线在灰度图上抹平成地面色再找边缘**；
#   * 【v8】这个场地的地面是**带斑点的地胶**，斑纹在 blur=5/low=40 下 ROI 里
#     边缘密度有 4.94%/3.38%/4.28%/4.49%（四处取样），而真车才 14.28%
#     —— **只差 1.8 倍，分不开**，于是"没障碍的时候车会莫名开始避障"。
#
# 【v8】参数是扫出来的（真实画面 + 合成道具一起端到端测）：
#     blur=5 /40/110  -> 真实地板两处都**误判**，合成道具都能认
#     blur=9 /60/160  -> 真实地板两处**都干净**，车形障碍仍能认（conf 0.84）★选它
#     blur=11/80/200  -> 地板干净，但合成车形障碍被滤掉（太狠）
# 依据：审阅人留的两张真实框图 + `tests/samples/` 里那两个真实裁片
# （`real_floor_roi.png` 负样本、`real_obstacle_car.png` 正样本）。
#
# ⚠️ 代价（必须知道）：模糊变大之后，**对比度不足的扁平浅色道具抓不到**
# （纯色亮橙块 vs 浅色地面只差 37 灰阶，模糊后边缘消失）。
# 所以**这一路只当兜底**：主路径是下面第 0 节 —— 直接用 DJI SDK 的机器人识别。
USE_STRUCTURE = True
EDGE_BLUR = 9                # Canny 之前的模糊核（奇数）
EDGE_LOW = 60                # Canny 低阈值（原来 40）
EDGE_HIGH = 160              # Canny 高阈值（原来 110）
EDGE_DILATE = 3              # 把 1 像素宽的边缘加粗，后面才连得成块
STRUCTURE_CLOSE = 17         # 把边缘连成整块的核大小

# ---------------------------------------------------------------------
# 【v9】检测模式零（**主路径**）：DJI SDK 的机器人识别
# ---------------------------------------------------------------------
# 用户点破了要害：**颜色判据根本答不了"车在哪里"** ——
# 车身在画面里颜色千变万化（光照、角度、遮挡），靠颜色既框不准位置、也框不全车身。
#
# 而 DJI 官方 SDK 本来就有"机器人识别"（已对 `robomaster/vision.py` 核实）：
#
#     ep_robot.vision.sub_detect_info(name="robot", callback=cb)
#     回调 rect_info = [(x, y, w, h), ...]   x/y 是**中心点**，w/h 是宽高
#
# 也就是 SDK 直接给出"画面里那个位置有一台同型机器人，框是这么大" ——
# 这才是真正能框住车、还能顺便估算远近的东西，而且**完全不看颜色**。
#
# 架构上：任务模块**不得自己订阅 SDK**（契约），所以订阅由集成层做
# （照 `marker_source.py` 的样子），每帧调 `task.update_robot_observations(...)`
# 把观测推进来。模块这边只用、不订。
#
# 下面这两个门槛是"这一帧要不要因为这台车开始绕"的判据：
ROBOT_MIN_WIDTH_RATIO = 0.06   # SDK 框宽度 < 画面宽 6% 就当太远，先不绕
ROBOT_OBSERVATION_MAX_AGE = 0.35  # 观测超过这么久没更新就当没有（SDK 回调会掉）
ROBOT_CONFIDENCE = 0.95        # SDK 给的框，置信度给高（它是官方识别，不是猜的）

# ---------------------------------------------------------------------
# 皮肤色剔除（**默认开启**）
# ---------------------------------------------------------------------
# 实测：手掌/手臂的 H 落在 5~25，和橙色道具几乎重叠；几何上也没有任何特征
# 能把"手"和"障碍"分开。所以直接把"皮肤色占比过半"的候选整个扔掉。
# 区别在于饱和度：皮肤通常 S < 190，鲜艳的橙色道具 S 接近 255。
# 真道具万一就是这个颜色，把 REJECT_SKIN_LIKE 关掉或把区间调窄。
REJECT_SKIN_LIKE = True
SKIN_HSV_RANGES = (
    ((0, 25, 60), (25, 190, 255)),
)
SKIN_REJECT_RATIO = 0.50     # 候选里皮肤色像素超过一半 -> 不是障碍

# ---------------------------------------------------------------------
# 找线（SEEK 段用）：和框架巡线用同一套蓝线颜色，避免两套标准打架
# ---------------------------------------------------------------------
# 只看车前方这一段：再往下就是车体自己挡住的区域了。
LINE_CHECK_ROI = (0.10, 0.45, 0.90, 0.80)
MIN_LINE_PIXELS = 40         # 蓝线像素少于这个数就算没看到线


def _clamp(value, low, high):
    return max(low, min(high, value))


def _odd_kernel(size):
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def _pixel_box(shape, roi):
    """把比例 ROI 换算成像素框 (left, top, right, bottom)，并夹在画面范围内。"""
    height, width = shape[:2]
    left = max(0, min(int(width * roi[0]), width - 1))
    top = max(0, min(int(height * roi[1]), height - 1))
    right = max(left + 1, min(int(width * roi[2]), width))
    bottom = max(top + 1, min(int(height * roi[3]), height))
    return left, top, right, bottom


def line_is_visible(image) -> bool:
    """车前方看得到蓝线吗（SEEK 段用）。用的是框架巡线同一套颜色。"""
    if image is None:
        return False
    left, top, right, bottom = _pixel_box(image.shape, LINE_CHECK_ROI)
    roi = image[top:bottom, left:right]
    if roi.size == 0:
        return False
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower, upper = CONFIG.vision.hsv_lower, CONFIG.vision.hsv_upper
    mask = cv2.inRange(
        hsv,
        np.array(lower, dtype=np.uint8),
        np.array(upper, dtype=np.uint8),
    )
    return int(cv2.countNonZero(mask)) >= MIN_LINE_PIXELS


class ObstacleDetector:
    """只回答一个问题：这一帧里有没有障碍、在哪儿。

    两条证据合起来用（默认只开第一条）：

      * **结构**（默认开）：走廊里边缘很硬、连得成块、又宽又高、立在地面上的东西。
        不依赖颜色 —— 另一台车、纸箱、挡板都能抓。软影子、反光抓不到。
      * **颜色**（默认关）：命中 OBSTACLE_HSV_RANGES 的色块。见 P0-1，皮肤会撞车。

    候选出来以后统一套形状判据，再剔掉皮肤色，最后从多个候选里挑最好的。
    全程只在自己那一小块 ROI 里算。
    """

    def __init__(self) -> None:
        self.last_candidates = 0
        self.last_skin_rejected = 0
        self.last_edge_rejected = 0
        self.last_position_rejected = 0
        self.last_flat_rejected = 0
        self.last_mask = None
        self.last_roi = None      # 最近一次用的 ROI 像素框，选边时要用
        self.last_source = "none"  # "sdk_robot" / "structure" / "none"，给运行记录看
        # SDK 机器人识别的观测（由集成层每帧推进来；模块自己不订阅 SDK）
        self._robot_rows = ()
        self._robot_seen_at = None

    def update_robot_observations(self, rows, observed_at=None) -> None:
        """集成层每帧调：把 SDK 机器人识别的结果推进来。

        `rows` 里每一项是 `(x, y, w, h)`：x/y 是**中心点**、w/h 是宽高（整幅画面像素）。
        契约上模块不得自己订阅 SDK，所以这条通道必须由主循环统一喂
        （和 `number_marker` 的 `update_candidates` 一个道理）。
        空列表表示"这一帧没有识别到机器人"，会把旧观测清掉，免得拿旧框当新鲜。
        """
        try:
            converted = tuple(tuple(float(v) for v in row[:4]) for row in (rows or ()))
        except (TypeError, ValueError):
            converted = ()
        # 丢掉 nan / inf，它们会让后面的比较静默出错
        converted = tuple(
            row for row in converted if all(value == value and abs(value) != float("inf")
                                            for value in row)
        )
        self._robot_rows = converted
        self._robot_seen_at = (
            (time.monotonic() if observed_at is None else float(observed_at))
            if converted else None
        )

    def _robot_box(self, image, now):
        """SDK 机器人识别给出的框（新鲜的话）。**完全不看颜色。**

        这是主路径：官方识别直接告诉我们"画面里那个位置有一台同型机器人，框这么大"，
        比任何颜色/边缘判据都可靠，也顺带能估算远近。
        """
        rows = self._robot_rows
        seen_at = self._robot_seen_at
        if not rows or seen_at is None or now is None:
            return None
        if now - seen_at > ROBOT_OBSERVATION_MAX_AGE:
            return None

        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return None
        roi_left, roi_top, roi_right, roi_bottom = _pixel_box(image.shape, OBSTACLE_ROI)

        best = None
        for x, y, w, h in rows:
            if w <= 0 or h <= 0:
                continue
            if w / float(width) < ROBOT_MIN_WIDTH_RATIO:
                # 太远：还没到要绕的距离（这一条替代了原来那套位置判据）
                continue
            if not (roi_left <= x <= roi_right):
                # 不在正前方（偏到旁边去了），不是挡路的那台
                continue
            if best is None or w > best[0]:
                best = (w, x, y, w, h)

        if best is None:
            return None

        _w, x, y, w, h = best
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(x)), int(round(y))),
            confidence=ROBOT_CONFIDENCE,
            box=(
                int(round(x - w / 2.0)),
                int(round(y - h / 2.0)),
                int(round(x + w / 2.0)),
                int(round(y + h / 2.0)),
            ),
        )

    # ---------------- 两条证据 ----------------

    def _line_mask(self, hsv):
        """蓝线掩膜，和框架巡线同一套颜色。抹线、判"框里是不是线"都用它。"""
        lower, upper = CONFIG.vision.hsv_lower, CONFIG.vision.hsv_upper
        return cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )

    def _structure_mask(self, roi, line):
        """硬边缘 -> 连成块。不依赖障碍颜色。

        返回 (连成块的掩膜, 加粗后的原始边缘图)。边缘图给闸六用：
        "框里面除了自己那圈轮廓，还有没有别的东西"。
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        if cv2.countNonZero(line):
            # 把蓝线像素抹成"周围地面的灰度"：线本身不再产生边缘，
            # 也不会把横穿它的障碍轮廓切断
            outside = gray[line == 0]
            if outside.size:
                gray = gray.copy()
                gray[line > 0] = int(np.median(outside))

        gray = cv2.GaussianBlur(gray, (EDGE_BLUR, EDGE_BLUR), 0)
        edges = cv2.Canny(gray, EDGE_LOW, EDGE_HIGH)
        edges = cv2.dilate(edges, _odd_kernel(EDGE_DILATE))
        blobs = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, _odd_kernel(STRUCTURE_CLOSE))
        return blobs, edges

    @staticmethod
    def _looks_like_an_object(edges, box):
        """闸六：框里除了自己那圈轮廓，**里面还有硬边缘**吗。

        v9 起只看灰度边缘，不再看颜色（用户要求：判据里不要有颜色）。
        """
        if edges is None:
            return False

        box_left, box_top, box_width, box_height = box
        inset = INTERNAL_INSET_PX
        inner_left = box_left + inset
        inner_top = box_top + inset
        inner_right = box_left + box_width - inset
        inner_bottom = box_top + box_height - inset
        if inner_right <= inner_left or inner_bottom <= inner_top:
            return False
        inner = edges[inner_top:inner_bottom, inner_left:inner_right]
        total = float(inner.shape[0] * inner.shape[1])
        if total <= 0:
            return False
        return (int(cv2.countNonZero(inner)) / total) >= MIN_INTERNAL_EDGE_RATIO

    @staticmethod
    def _is_skin_like(patch, inside):
        """候选里皮肤色像素占了一半以上吗。"""
        if not REJECT_SKIN_LIKE:
            return False
        skin = np.zeros(patch.shape[:2], dtype=np.uint8)
        for lower, upper in SKIN_HSV_RANGES:
            skin |= cv2.inRange(
                patch,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
        skin = cv2.bitwise_and(skin, inside)
        total = int(cv2.countNonZero(inside))
        if total <= 0:
            return False
        return (int(cv2.countNonZero(skin)) / float(total)) >= SKIN_REJECT_RATIO

    @staticmethod
    def _object_ratio(patch, line_patch):
        """框里"既不像浅色地面、也不像蓝线"的像素占多少。

        闸三用它：一块和地面同色的浅色结构（背景、场地边界、阴影边）内部几乎
        全是地面色，这个比例会很低，于是被拒；真障碍（车、箱子）内部是别的东西。
        """
        floor_like = cv2.inRange(
            patch,
            np.array((0, 0, FLOOR_V_MIN), dtype=np.uint8),
            np.array((180, FLOOR_S_MAX, 255), dtype=np.uint8),
        )
        background = cv2.bitwise_or(floor_like, line_patch)
        total = int(patch.shape[0] * patch.shape[1])
        if total <= 0:
            return 0.0
        return 1.0 - (int(cv2.countNonZero(background)) / float(total))

    # ---------------- 对外 ----------------

    def detect(self, image, now=None) -> VisualDetection:
        """返回整幅图像坐标的 VisualDetection；没有障碍返回 no_result。

        **主路径是 SDK 的机器人识别**（`_robot_box`）：集成层每帧把官方识别的框
        推进来，这里直接用，完全不看颜色。没有新鲜观测时才退回灰度结构判据。
        """
        self.last_candidates = 0
        self.last_skin_rejected = 0
        self.last_edge_rejected = 0
        self.last_position_rejected = 0
        self.last_flat_rejected = 0
        self.last_mask = None
        self.last_source = "none"
        if image is None:
            return VisualDetection.no_result(KIND)

        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return VisualDetection.no_result(KIND)

        left, top, right, bottom = _pixel_box(image.shape, OBSTACLE_ROI)
        self.last_roi = (left, top, right, bottom)

        # 主路径：SDK 机器人识别（不看颜色、直接给框）
        from_robot = self._robot_box(image, now)
        if from_robot is not None:
            self.last_source = "sdk_robot"
            return from_robot

        roi = image[top:bottom, left:right]
        roi_height, roi_width = roi.shape[:2]
        roi_area = float(roi_height * roi_width)
        if roi_area <= 0:
            return VisualDetection.no_result(KIND)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        line = self._line_mask(hsv)

        mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
        edges = None
        if USE_STRUCTURE:
            structure, edges = self._structure_mask(roi, line)
            mask |= structure
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _odd_kernel(OPEN_KERNEL))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _odd_kernel(CLOSE_KERNEL))
        self.last_mask = mask
        self.last_source = "structure"

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_score = -1.0
        for contour in contours:
            box_left, box_top, box_width, box_height = cv2.boundingRect(contour)
            box_area = float(box_width * box_height)
            if box_area <= 0:
                continue
            box_ratio = box_area / roi_area
            if box_ratio < MIN_OBSTACLE_AREA or box_ratio > MAX_OBSTACLE_AREA:
                continue

            width_ratio = box_width / float(roi_width)
            height_ratio = box_height / float(roi_height)
            if width_ratio < MIN_OBSTACLE_WIDTH or width_ratio > MAX_OBSTACLE_WIDTH:
                continue
            if height_ratio < MIN_OBSTACLE_HEIGHT:
                # 立着挡路的东西才有这个高度；贴地的细条、扁平色斑到这里被刷掉
                continue

            # 闸一：贴边淘汰（**只判左沿 / 右沿**）
            # v10：下沿不再淘汰 —— 障碍贴到车头时必然压住 ROI 下沿，
            # 那正是该绕的那一台。上沿由 TOP_INSET_RATIO 控制（现在 =0，不判）。
            if REJECT_EDGE_TOUCHING:
                margin_x = roi_width * EDGE_TOUCH_MARGIN
                if (
                    box_left <= margin_x
                    or box_left + box_width >= roi_width - margin_x
                ):
                    self.last_edge_rejected += 1
                    continue

            # 闸四 + 闸五：位置判据（v6）
            #   闸四：上沿不再豁免——实测两个误判框的 y 正好等于 ROI 顶边，
            #         框内是墙 / 白板 / 暗处。真障碍不该从 ROI 最顶上开始。
            #   闸五：纵向要在下半部（近处）、横向不能偏出中间一半。
            center_x = box_left + box_width / 2.0
            center_y = box_top + box_height / 2.0
            if box_top < roi_height * TOP_INSET_RATIO:
                self.last_position_rejected += 1
                continue
            if center_y < roi_height * MIN_LOWER_RATIO:
                self.last_position_rejected += 1
                continue
            if abs(center_x - roi_width / 2.0) > (roi_width / 2.0) * MAX_CENTER_OFFSET_RATIO:
                self.last_position_rejected += 1
                continue

            inside = mask[box_top:box_top + box_height, box_left:box_left + box_width]
            density = cv2.countNonZero(inside) / box_area
            if density < MIN_DENSITY:
                # 框里几乎是空的 = 稀疏散点，不是一坨挡路的东西
                continue
            patch = hsv[box_top:box_top + box_height, box_left:box_left + box_width]

            # 闸三：框里得真的有东西（不能全是浅色地面或蓝线）
            line_patch = line[box_top:box_top + box_height, box_left:box_left + box_width]
            if self._object_ratio(patch, line_patch) < MIN_OBJECT_RATIO:
                self.last_flat_rejected += 1
                continue

            if self._is_skin_like(patch, inside):
                # 皮肤色的东西不当障碍（见 P0-1）
                self.last_skin_rejected += 1
                continue

            # 闸六：一块"平的"区域不是障碍（框里除了轮廓，里面还得有硬边缘）
            if not self._looks_like_an_object(
                edges, (box_left, box_top, box_width, box_height)
            ):
                self.last_flat_rejected += 1
                continue

            self.last_candidates += 1
            area_score = _clamp(box_ratio / 0.15, 0.0, 1.0)
            lower_score = _clamp(center_y / float(roi_height), 0.0, 1.0)
            center_score = 1.0 - _clamp(
                abs(center_x - roi_width / 2.0) / (roi_width / 2.0), 0.0, 1.0
            )
            width_score = _clamp(width_ratio / 0.50, 0.0, 1.0)
            score = (
                area_score * SCORE_AREA_W
                + lower_score * SCORE_LOWER_W
                + center_score * SCORE_CENTER_W
                + width_score * SCORE_WIDTH_W
            )
            if score > best_score:
                best_score = score
                best = (box_left, box_top, box_width, box_height, center_x, center_y)

        if best is None:
            return VisualDetection.no_result(KIND)

        confidence = _clamp(best_score / SCORE_TOTAL_W, 0.0, 1.0)
        if confidence < MIN_CONFIDENCE:
            return VisualDetection.no_result(KIND)

        box_left, box_top, box_width, box_height, center_x, center_y = best
        return VisualDetection(
            valid=True,
            kind=KIND,
            center=(int(round(left + center_x)), int(round(top + center_y))),
            confidence=confidence,
            box=(
                int(left + box_left),
                int(top + box_top),
                int(left + box_left + box_width),
                int(top + box_top + box_height),
            ),
        )


class ObstacleTask:
    """看到障碍就绕过去；绕行被拆成"每帧只走一小步"的状态机。

    阶段：
        IDLE -> OUT（立刻侧移让开） -> PASS（前进越过）
             -> BACK（侧移回中线） -> SEEK（主动把线找回来） -> COMPLETED

    HOLD 段只在 HOLD_BEFORE_GO > 0 时才存在；默认是 0，也就是确认完当帧就侧移。
    """

    name = "obstacle"

    def __init__(self, settings: Optional[object] = None) -> None:
        # settings 保留是为了和骨架的约定一致；本模块的阈值全部是文件顶部的常量。
        self.settings = settings
        self.detector = ObstacleDetector()
        self.last_detection = VisualDetection.no_result(KIND)
        self.started_at = None
        self.stage = "IDLE"
        self.segment_started_at = None
        self.hit_frames = 0
        self.line_frames = 0
        self.finished_at = None
        # 同一段路上连续绕了几次；连绕太多说明"障碍"很可能根本绕不过去，
        # 或者那压根不是障碍，这时候停车让人来看，比一直绕安全。
        self.dodge_count = 0
        self.last_seen_at = None
        # 到达连绕上限后置位：不再接管，免得变成"每几秒停一下"的走走停停。
        self.locked = False
        # 这一轮往哪边绕（v8：不再固定往左，触发时按障碍偏哪边决定）
        self._active_side = 1.0 if DODGE_SIDE == "right" else -1.0
        self.last_side = None     # "left" / "right"，给运行记录和得分快照用
        # 上一次 step() 的时刻：用来发现"我们被从外面踢掉了"（见 STALE_STEP_GAP）
        self._last_step_at = None
        # 第一次被调用的时刻：启动预热用（STARTUP_GRACE_SECONDS，默认 0 即关闭）
        self._first_step_at = None

    # ---------------- 对外 ----------------

    def detect(self, image, now=None) -> VisualDetection:
        """给一帧图像，回答有没有障碍（整幅图像坐标）。

        先走 SDK 机器人识别（如果集成层喂了新鲜观测），没有再退回灰度结构判据。
        """
        return self.detector.detect(image, now)

    def update_robot_observations(self, rows, observed_at=None) -> None:
        """集成层每帧调：把 SDK 机器人识别的结果推进来（见 ObstacleDetector 的同名方法）。

        这是本模块**唯一**的障碍观测入口，由主循环喂；模块自己不订阅 SDK。
        """
        self.detector.update_robot_observations(rows, observed_at)

    def step(self, frame: FramePacket, now: float) -> TaskUpdate:
        """主循环每帧调用一次，必须立刻返回。"""
        if self._first_step_at is None:
            self._first_step_at = now
        if self.stage != "IDLE" and self._was_cut_short(now):
            # 【v8】断档：协调器在外面把控制权拿走又还回来，但**不会通知模块**。
            #
            # 这里**绝不能**清成 IDLE 就直接走 _step_idle —— 实车记录里一共 12 次
            # `obstacle returned NOT_TRIGGERED while owning motion` 就是这么来的：
            # 协调器认为我们还拿着控制权，我们却说不想要了，于是判违约、强制收回。
            #
            # 正确做法：保留控制权、先停一脚（RUNNING + 零速度），下一帧再重新判断。
            # 见 _step_active 里的 RECONFIRM 分支。
            self._enter("RECONFIRM", now)
            self.started_at = now
        self._last_step_at = now

        if self.stage == "IDLE":
            detection = self.detect(frame.image, now)
            self.last_detection = detection
            return self._step_idle(frame.image, detection, now)
        # 绕行途中不再重复检测（结果只用于上报）；只有 SEEK / RECONFIRM 要看画面。
        return self._step_active(frame.image, now)

    def _was_cut_short(self, now: float) -> bool:
        """两次 step() 之间隔得太久，说明中间我们并不在开车。"""
        return (
            self._last_step_at is not None
            and (now - self._last_step_at) > STALE_STEP_GAP
        )

    # ---------------- 还没接管：判断要不要管 ----------------

    def _step_idle(self, image, detection: VisualDetection, now: float) -> TaskUpdate:
        # 启动预热：默认关（STARTUP_GRACE_SECONDS = 0）。
        # 要压制"一启动就避障"，把它设成 3.0 左右即可。
        if (
            STARTUP_GRACE_SECONDS > 0.0
            and self._first_step_at is not None
            and (now - self._first_step_at) < STARTUP_GRACE_SECONDS
        ):
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection, message="startup warm-up"
            )

        # 画面里障碍消失够久 -> 认为换了一段路，连绕计数归零、解锁
        if detection.valid:
            self.last_seen_at = now
        elif self.last_seen_at is not None and now - self.last_seen_at >= CLEAR_SECONDS:
            self.dodge_count = 0
            self.locked = False
            self.last_seen_at = None

        if self.finished_at is not None and now - self.finished_at < REARM_SECONDS:
            # 刚绕完，别对着同一个（可能还在画面里的）障碍马上再来一次
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection, message="re-arm cooldown"
            )

        if not detection.valid:
            self.hit_frames = 0
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED, detection=detection, message="no obstacle"
            )

        # 【v10】关键的一条：**车看到了还不够，必须等蓝线被它挡住**。
        #   蓝线在近处消失 = 车已经开到障碍跟前了 —— 这就是"距离检测"。
        #   原来一看到车就横移，结果每次都早早开始绕、正好停在障碍前面。
        if TRIGGER_NEEDS_LINE_LOST and line_is_visible(image):
            self.hit_frames = 0
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="obstacle seen but the line is still visible; not yet",
            )

        if self.locked:
            # 已经连绕到上限、也失败过一次了：不再插手，等障碍消失后再解锁。
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="locked after %d dodges; waiting for the obstacle to clear"
                % self.dodge_count,
            )

        self.hit_frames += 1
        if self.hit_frames < CONFIRM_FRAMES:
            # 连确认帧数都不够，还不能接管
            return TaskUpdate(
                TaskStatus.NOT_TRIGGERED,
                detection=detection,
                message="candidate %d/%d" % (self.hit_frames, CONFIRM_FRAMES),
            )

        if self.dodge_count >= MAX_CONSECUTIVE_DODGES:
            # 已经连绕这么多次、障碍还在：先接管（协调器只认 RUNNING），
            # 下一帧立刻 FAILED，让它硬停车交给人看；同时上锁不再接管。
            self.locked = True
            self.stage = "ABORT"
            self.started_at = now
            self.segment_started_at = now
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(),
                detection=detection,
                message="dodged %d times in a row and it is still there; stopping"
                % self.dodge_count,
            )

        self.started_at = now
        self.segment_started_at = now
        # 选边：障碍偏哪边就往另一边绕（PDF 要求"看清后自己选左或右"）
        self._active_side = self._choose_side(detection)
        self.last_side = self._side_name()
        if HOLD_BEFORE_GO > 0.0:
            self.stage = "HOLD"
            return TaskUpdate(
                TaskStatus.RUNNING,
                motion=MotionCommand(),
                detection=detection,
                message="obstacle confirmed; holding before the dodge",
            )
        # 【P0-2】不设停车段：确认的这一帧就已经在下发侧移，
        # 不再白送 0.20 秒的零速让车继续前冲。
        self.stage = "OUT"
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=MotionCommand(lateral=self._side() * SIDE_SPEED),
            detection=detection,
            message="obstacle confirmed; stepping aside immediately",
        )

    # ---------------- 已经接管：一步一步绕 ----------------

    def _step_active(self, image, now: float) -> TaskUpdate:
        if self.stage == "ABORT":
            return self._finish(
                TaskStatus.FAILED,
                "obstacle still there after %d dodges; stopped for a human check"
                % self.dodge_count,
                now,
            )

        if self.stage == "RECONFIRM":
            # 断档之后：**保留控制权**（返回 RUNNING），先停一脚，下一帧重新判断。
            # 关键：这条路径**永远不能返回 NOT_TRIGGERED**（那是违约，见 step()）。
            if now - self.segment_started_at < RECONFIRM_HOLD:
                return self._running(
                    MotionCommand(), "control was cut short; stopping to re-check"
                )
            detection = self.detect(image, now)
            self.last_detection = detection
            if detection.valid:
                # 障碍还在：重新开始一轮完整的绕行（重新选边）
                self._enter("OUT", now)
                self.started_at = now
                self._active_side = self._choose_side(detection)
                self.last_side = self._side_name()
                return self._running(
                    MotionCommand(lateral=self._side() * SIDE_SPEED),
                    "re-confirmed after a gap; stepping aside",
                )
            # 障碍没了：正常交回（COMPLETED 会走协调器的归还流程，不算违约）
            return self._finish(
                TaskStatus.COMPLETED, "takeover was cut short; handing back", now
            )

        if now - self.started_at > MAX_TOTAL_TIME:
            return self._finish(
                TaskStatus.FAILED,
                "dodge exceeded %.2fs; stopping" % MAX_TOTAL_TIME,
                now,
            )

        elapsed = now - self.segment_started_at

        # 注意：每进入下一段都必须把 elapsed 归零，否则会**一路穿透**。
        # 之前这里犯过这个错：侧移段结束时 elapsed=1.7，已经 >= 前进段的 1.6、
        # 也 >= 回收段的 1.2，于是一个调用里 OUT→PASS→BACK→SEEK 全走完，
        # **前进和回收两段被整段跳过**，车只横移一下就去找线了。
        if self.stage == "HOLD":
            if elapsed < HOLD_BEFORE_GO:
                return self._running(MotionCommand(), "holding before the dodge")
            self._enter("OUT", now)
            elapsed = 0.0

        if self.stage == "OUT":
            if elapsed < T_OUT_TIME:
                return self._running(
                    MotionCommand(lateral=self._side() * SIDE_SPEED), "stepping aside"
                )
            self._enter("PASS", now)
            elapsed = 0.0

        if self.stage == "PASS":
            if elapsed < T_PASS_TIME:
                return self._running(MotionCommand(forward=FWD_SPEED), "passing the obstacle")
            self._enter("BACK", now)
            elapsed = 0.0

        if self.stage == "BACK":
            if elapsed < T_BACK_TIME:
                return self._running(
                    MotionCommand(lateral=-self._side() * BACK_SPEED), "returning to the line"
                )
            self._enter("SEEK", now)

        if self.stage == "SEEK":
            return self._step_seek(image, now)

        return self._finish(TaskStatus.FAILED, "internal stage error: %s" % self.stage, now)

    def _step_seek(self, image, now: float) -> TaskUpdate:
        """绕完主动把蓝线找回来：一边往线的方向慢慢挪，一边看画面里有没有线。

        骨架的自动恢复只是"停在原地等"，而绕完以后车已经横向偏了几十厘米，
        线很可能已经不在视野里——等是等不回来的，必须自己挪回去。
        """
        if line_is_visible(image):
            self.line_frames += 1
            if self.line_frames >= LINE_CONFIRM_FRAMES:
                return self._finish(
                    TaskStatus.COMPLETED, "line found again; handing back", now
                )
        else:
            self.line_frames = 0

        if now - self.segment_started_at >= SEEK_TIME:
            return self._finish(
                TaskStatus.FAILED,
                "line not found within %.2fs after the dodge; stopping" % SEEK_TIME,
                now,
            )

        return self._running(
            MotionCommand(lateral=-self._side() * SEEK_SPEED), "looking for the line"
        )

    # ---------------- 小工具 ----------------

    def _enter(self, stage: str, now: float) -> None:
        self.stage = stage
        self.segment_started_at = now
        if stage == "SEEK":
            self.line_frames = 0

    def _running(self, motion: MotionCommand, message: str) -> TaskUpdate:
        return TaskUpdate(
            TaskStatus.RUNNING,
            motion=motion,
            detection=self.last_detection,
            message=message,
        )

    def _finish(self, status: TaskStatus, message: str, now: float) -> TaskUpdate:
        if status is TaskStatus.COMPLETED:
            # 只有真正绕过去一次才计数；连绕太多会触发上面的 ABORT 保护
            self.dodge_count += 1
        self.stage = "IDLE"
        self.hit_frames = 0
        self.line_frames = 0
        self.started_at = None
        self.segment_started_at = None
        self.finished_at = now
        return TaskUpdate(
            status,
            motion=MotionCommand(),
            detection=self.last_detection,
            message=message,
        )

    def _side(self) -> float:
        """这一轮往哪边绕（+1 右 / -1 左）。触发时由 _choose_side 定好。"""
        return self._active_side

    def _choose_side(self, detection) -> float:
        """决定从左边还是右边绕。

        PDF item 3 要求"看清后自己选左或右"，所以不能永远往左：
        障碍明显**偏左**就从右边过、明显**偏右**就从左边过（远离它）；
        基本在正中间（偏移不到 SIDE_DEADBAND）就按 DODGE_SIDE 的默认值。
        """
        default = 1.0 if DODGE_SIDE == "right" else -1.0
        if not detection.valid or detection.center is None:
            return default
        roi = self.detector.last_roi
        if roi is None:
            return default
        roi_left, _roi_top, roi_right, _roi_bottom = roi
        half = max((roi_right - roi_left) / 2.0, 1.0)
        offset = (detection.center[0] - (roi_left + roi_right) / 2.0) / half
        if offset > SIDE_DEADBAND:
            return -1.0        # 障碍偏右 -> 往左绕
        if offset < -SIDE_DEADBAND:
            return 1.0         # 障碍偏左 -> 往右绕
        return default

    def _side_name(self) -> str:
        return "right" if self._active_side > 0 else "left"


# =====================================================================
# 离线调参小工具（框架不会调用它）
# =====================================================================
# 用途：给一张现场样图，看检测器认出了什么、掩膜长什么样，方便调阈值。
# 用法：python obstacle.py 样图.png [输出掩膜.png]
#
# 它只读一张图，不开相机、不连车、不联网。
if __name__ == "__main__":  # pragma: no cover
    import sys

    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)

    _image = cv2.imread(sys.argv[1])
    if _image is None:
        print("读不到这张图：%s" % sys.argv[1])
        raise SystemExit(1)

    _detector = ObstacleDetector()
    _result = _detector.detect(_image)
    print("图像尺寸   : %dx%d" % (_image.shape[1], _image.shape[0]))
    print("候选个数   : %d" % _detector.last_candidates)
    print("皮肤剔除   : %d" % _detector.last_skin_rejected)
    print("判定结果   : %s" % ("有障碍" if _result.valid else "没有障碍"))
    if _result.valid:
        print("中心/框    : %s / %s" % (_result.center, _result.box))
        print("置信度     : %.2f" % _result.confidence)
    print("蓝线可见   : %s" % line_is_visible(_image))

    if len(sys.argv) >= 3 and _detector.last_mask is not None:
        cv2.imwrite(sys.argv[2], _detector.last_mask)
        print("掩膜已保存 : %s" % sys.argv[2])
