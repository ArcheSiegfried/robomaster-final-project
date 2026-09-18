"""显式任务注册表：骨架如何知道存在哪些模块。

新增一个模块只需要做两件事：把文件放在仓库根目录，把类加进下面两个元组之一。
其余全部自动生效——main.py、协调器、限幅与超时护栏、单个模块自测脚本都从这里取。

`MOTION_TASK_CLASSES` 的**顺序就是接管优先级**：每个周期协调器按这个顺序询问，
第一个返回 RUNNING 的模块拿到控制权。要调整优先级就调整这里的顺序。

普通模块开发者不需要改这个文件；如果你新建了一个模块文件却没有登记，
`tests/test_task_contract.py` 会直接失败并告诉你该加到哪一行。
"""

from evidence import EvidenceRecorder
from free_junction import FreeJunctionTask
from green_junction import GreenJunctionTask
from number_marker import NumberMarkerTask
from obstacle import ObstacleTask
from route import RouteTask
from traffic_light import TrafficLightTask

# 功能模块：一个文件 = 一个名额 = 一个人。顺序即接管优先级。
#
# 2026-09-18 变更（按期末考场场景顺序重排，用户批准）：
# 依据是决赛要求 PDF（16-Final_Project_Requirements v8）第 41 页的六项现场任务顺序：
# 岔路选绿灯那侧 → 绕障 → 岔路避开拥堵 → 断线恢复 → 红灯停绿灯行 → 标识 1~5 拍照。
# 而考场第一个岔路口是**左红右绿**：`traffic_light` 排第 1 时只认颜色不认位置，
# 红灯 2 帧就返回 RUNNING + 零运动原地锁停，会把 `green_junction` 的选路权整个吃掉
# （15 分），所以岔路选路必须排在它前面。
#
#   1 green_junction  第一岔路：左红右绿，往绿灯那侧走（赛题 4）
#   2 obstacle        绕障：车头前的障碍必须先处理（赛题 3）
#   3 free_junction   第二岔路：避开停着小车的拥堵岔路（赛题 5）
#   4 route           断线恢复：线消失才触发，优先级高于"选路"（赛题 2）
#   5 traffic_light   自选地点红灯停绿灯行（赛题 6）
#   6 number_marker   标识 1~5 拍照计分，不抢在会动作的模块前面（赛题 1）
#
# 两个必须知道的点：
#   1. `traffic_light` 移到第 5 位**不影响红灯停车**：`coordinator.py` 的"红灯否决权"
#      是按 **name** 在任务列表里找它（`coordinator.py:127-129`），与位次无关，
#      所以任何任务在开车时遇到红灯仍会被暂停（暂停时间不计入任务预算）；
#   2. `green_junction` 的认灯判据**不依赖** `traffic_light` 模块：
#      它自带 `builtin_lamp_detection`（`green_junction.py:689`）和
#      `red_blocks_branch`（A18），降到第 5 位不会让第一岔路失去判据。
MOTION_TASK_CLASSES = (
    GreenJunctionTask,  # 1 第一岔路：左红右绿，往绿灯那侧走（赛题 4）
    ObstacleTask,  # 2 绕障（赛题 3）
    FreeJunctionTask,  # 3 第二岔路：避开拥堵（赛题 5）
    RouteTask,  # 4 断线恢复（赛题 2）
    TrafficLightTask,  # 5 自选地点红灯停绿灯行（赛题 6）
    NumberMarkerTask,  # 6 标识 1~5 拍照（赛题 1）
)

# 基础设施观察者：每帧都能看到，但永远不能接管运动。
# 这**不是**功能模块名额，由整合负责人维护，不单独占一个人。
OBSERVER_CLASSES = (
    EvidenceRecorder,
)


def build_motion_tasks(enabled_names=None):
    """Instantiate registered motion tasks, optionally selecting by task name.

    ``None`` keeps the normal production behaviour and builds every registered
    task.  A tuple such as ``("route",)`` is intended for an isolated real-car
    test entry: the registry remains complete, while unrelated unfinished
    detectors cannot take over during that test.
    """
    if enabled_names is None:
        return tuple(cls() for cls in MOTION_TASK_CLASSES)

    requested = tuple(enabled_names)
    if not requested:
        raise ValueError("at least one motion task must be enabled")
    if len(requested) != len(set(requested)):
        raise ValueError("enabled motion task names must be unique")

    by_name = {cls.name: cls for cls in MOTION_TASK_CLASSES}
    unknown = tuple(name for name in requested if name not in by_name)
    if unknown:
        raise ValueError("unknown motion task(s): %s" % ", ".join(unknown))
    return tuple(by_name[name]() for name in requested)


def build_observers(capture_directory=None):
    """Instantiate every registered observer module.

    `capture_directory` 是证据记录的输出目录：
      * `None`（这里是默认值）= **完全不写盘**，测试和离线工具用它；
      * 真实运行由 `main.build_coordinator()` 传入 `captures/`。

    默认不写盘是刻意的：观察者必须在被创建时零副作用，否则跑一次测试就会在
    仓库里留下一个 captures/ 目录。
    """
    built = []
    for cls in OBSERVER_CLASSES:
        if cls is EvidenceRecorder:
            built.append(cls(directory=capture_directory))
        else:
            built.append(cls())
    return tuple(built)
