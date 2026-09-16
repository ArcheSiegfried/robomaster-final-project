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
from green_junction import GreenJunctionTask, make_light_probe
from number_marker import NumberMarkerTask
from obstacle import ObstacleTask
from route import RouteTask
from traffic_light import TrafficLightDetector, TrafficLightTask

#: 绿岔路模块要的"现在是什么灯"。它自己不认灯（契约禁止它直接读别人的判定），
#: 只接受一个注入缝 `light_probe(frame, now) -> LightReading`；而本注册表是
#: **唯一无参构造任务的装配点**，所以这根线必须在这里接（见 PR#45 的 A14）。
#:
#: 两个必须注意的点（依据 PR#45 审查）：
#: 1. 只包**无状态**的 `TrafficLightDetector`。不要包 `TrafficLightTask()`——
#:    那会养出第二台灯机、自己往证据队列塞红灯截图（实测 state=holding_red）。
#: 2. `min_confidence` 不取它默认的 0.0（等于来者不拒）；实车真绿灯实测
#:    conf≈0.62，所以这里要求 0.40。
LIGHT_PROBE_MIN_CONFIDENCE = 0.40


def build_light_probe(detector=None, min_confidence=LIGHT_PROBE_MIN_CONFIDENCE):
    """把交通灯模块的识别结果转成绿岔路模块要的灯色探针。

    探针只做翻译，不重复判定：红绿灯识别仍由 `traffic_light` 负责。
    看不见灯或认不出颜色时如实返回 UNKNOWN —— 绿岔路会因此**不接管**
    （他的 A14：真的拿到红/绿读数才算有判据来源），于是不会再把握不住的
    岔路从后面的模块手里抢走。构造失败就返回 None（等于没接探针）。
    """
    try:
        detector = TrafficLightDetector() if detector is None else detector
        return make_light_probe(detector, min_confidence=min_confidence)
    except Exception:
        return None


def _build_task(cls, light_probe):
    """按类装配任务；需要外部观测的模块在这里注入。"""
    if cls is GreenJunctionTask:
        return cls(light_probe=light_probe)
    return cls()

# 功能模块：一个文件 = 一个名额 = 一个人。顺序即接管优先级。
#
# 2026-09-16 按实车反馈调整（第二版，集成负责人确认）：
#   红绿灯 → 红绿灯岔路 → 障碍物绕行 → 短线巡回 → 障碍物岔路 → 数字识别
# 变化：`obstacle` 从第 6 位升到第 3 位（车前方的障碍必须先处理）；
#       `free_junction` 从第 3 位降到第 5 位；
#       `number_marker` 挪到最后（它目前在实车上"看到标识却不接管"，
#       先让真正会动作的模块先接管；它的诊断见 marker_source.stats() 的宽度字段）。
# 注意顺序的代价：`obstacle` 现在会优先于岔路/巡回/标识接管，
# 而它的误触发率还不低（29 次运行里 26 帧被判成障碍），修判据之前要留意。
MOTION_TASK_CLASSES = (
    TrafficLightTask,  # 1 红绿灯（红灯是停车条件，最先判断）
    GreenJunctionTask,  # 2 红绿灯岔路
    ObstacleTask,  # 3 障碍物绕行
    RouteTask,  # 4 短线巡回
    FreeJunctionTask,  # 5 障碍物岔路
    NumberMarkerTask,  # 6 数字识别
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

    **装配在这里接线**：绿岔路模块自己不认红绿灯，必须由这里注入灯色探针
    （见 `build_light_probe`），否则它在生产环境下永远不会接管。
    """
    light_probe = build_light_probe()
    if enabled_names is None:
        return tuple(_build_task(cls, light_probe) for cls in MOTION_TASK_CLASSES)

    requested = tuple(enabled_names)
    if not requested:
        raise ValueError("at least one motion task must be enabled")
    if len(requested) != len(set(requested)):
        raise ValueError("enabled motion task names must be unique")

    by_name = {cls.name: cls for cls in MOTION_TASK_CLASSES}
    unknown = tuple(name for name in requested if name not in by_name)
    if unknown:
        raise ValueError("unknown motion task(s): %s" % ", ".join(unknown))
    return tuple(_build_task(by_name[name], light_probe) for name in requested)


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
