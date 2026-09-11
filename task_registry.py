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


def build_motion_tasks():
    """Instantiate every registered motion task, in takeover priority order."""
    return tuple(cls() for cls in MOTION_TASK_CLASSES)


def build_observers():
    """Instantiate every registered observer module."""
    return tuple(cls() for cls in OBSERVER_CLASSES)
