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
MOTION_TASK_CLASSES = (
    TrafficLightTask,  # 红灯是停车条件，最先判断
    NumberMarkerTask,
    GreenJunctionTask,  # 绿灯岔路
    FreeJunctionTask,  # 无拥堵岔路
    RouteTask,  # 长断线巡回
    ObstacleTask,  # 动作最复杂，放最后
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
