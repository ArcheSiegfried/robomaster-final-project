"""RoboMaster entry point. Importing this module never connects to hardware."""

import time

import cv2

from camera_source import LatestFrameSource
from config import CONFIG
from coordinator import LINE_FOLLOWING, RELEASING, TASK_ACTIVE, TaskCoordinator
from evidence import DEFAULT_CAPTURE_DIRECTORY, IntegratedScoreEvidence
from route_gimbal_output import RouteGimbalOutput
from gimbal_output import GimbalOutput
from marker_source import MarkerObservationSource
from motion_output import MotionOutput
from robot_source import RobotObservationSource
from runtime import LineFollower
from task_registry import build_motion_tasks, build_observers


def build_coordinator(
    follower,
    output,
    settings=CONFIG,
    capture_directory=DEFAULT_CAPTURE_DIRECTORY,
    gimbal_output=None,
    motion_task_names=None,
    motion_tasks_override=None,
):
    """Wire every registered module into the coordinator.

    Kept as a named factory so tests can build the real wiring, and assert that
    every module in task_registry is actually reachable, without any hardware.

    `capture_directory` is where the evidence recorder writes; pass None to
    disable recording entirely (tests do this so they never litter the repo).
    """
    return TaskCoordinator(
        settings,
        follower,
        output,
        motion_tasks=(
            build_motion_tasks(motion_task_names)
            if motion_tasks_override is None else tuple(motion_tasks_override)
        ),
        observers=build_observers(capture_directory),
        gimbal_output=gimbal_output,
    )


def _find_evidence_sink(coordinator):
    """找到唯一的证据写入器（实现了 save_task_evidence 的观察者）。"""
    for observer in getattr(coordinator, "observers", ()):
        if callable(getattr(observer, "save_task_evidence", None)):
            return observer
    return None


def record_runtime_diagnostics(coordinator, marker_source, robot_source=None) -> None:
    """把接线层的自检结果写进本次运行记录（report.md 的独立小节）。

    为什么要有这一步：时间线只能告诉你"某个模块没接管"，看不出原因。数字标识
    尤其依赖 SDK 的 marker 订阅，而订阅可能**静默失败**——颜色过滤器只能设一个、
    坐标模式猜错、回调频率不够。跑一次就把这些一起写进记录，下次不用靠猜。

    障碍物模块（v10 起）拿 SDK 的"机器人识别"当主路径，同理由集成层订阅后喂给它；
    订阅失败时它会退回灰度结构判据（误触发率明显更高），所以这一小节必须记下来。

    **必须在 `coordinator.close()` 之前调用**（close 会写 report.md）。
    和 main 里其它辅助函数一样：绝不抛异常，绝不影响开车。
    """
    sink = _find_evidence_sink(coordinator)
    record = getattr(sink, "record_diagnostics", None)
    if not callable(record):
        return
    try:
        if marker_source is None:
            record(
                "数字标识观测（SDK marker 订阅）",
                {"状态": "本次运行没有建立 marker 订阅，数字标识不会接管"},
            )
        else:
            values = dict(marker_source.stats())
            warning = marker_source.rate_warning()
            if warning:
                values["提醒"] = warning
            record("数字标识观测（SDK marker 订阅）", values)
    except Exception:
        pass
    try:
        if robot_source is None:
            record(
                "障碍物观测（SDK 机器人识别）",
                {"状态": "本次运行没有建立 robot 订阅，障碍模块退回灰度结构判据"},
            )
        else:
            values = dict(robot_source.stats())
            warning = robot_source.rate_warning()
            if warning:
                values["提醒"] = warning
            record("障碍物观测（SDK 机器人识别）", values)
    except Exception:
        pass


def service_task_evidence(coordinator) -> int:
    """把任务模块交出来的得分截图请求交给证据层，并回传真实结果。

    这是 Final 算分那条链的最后一环：

        task.take_evidence_request() -> 证据层画框写字存盘 -> acknowledge_evidence()

    **必须每帧在主循环里调用。** 任务在等回执期间会一直占着控制权并要求停车，
    所以这里必须给出明确答复，绝不能让请求悬着：
    * 存成功 -> 回传 True，任务才会 COMPLETED 并归还控制权、恢复巡线；
    * 存失败 / 没有可用的证据写入器 -> 回传 False，任务是 FAILED 并立刻交回
      控制权，而不是把车停在那里干等 8 秒超时。

    返回这一帧真正写成的截图数（给日志和测试用）。绝不抛异常。
    """
    sink = _find_evidence_sink(coordinator)
    saved_count = 0
    for task in getattr(coordinator, "motion_tasks", ()):
        take = getattr(task, "take_evidence_request", None)
        acknowledge = getattr(task, "acknowledge_evidence", None)
        if not callable(take) or not callable(acknowledge):
            continue
        try:
            request = take()
        except Exception:
            continue
        if request is None:
            continue
        saved = False
        if sink is not None:
            try:
                saved = bool(sink.save_task_evidence(request))
            except Exception:
                saved = False
        try:
            acknowledge(getattr(request, "request_id", ""), saved)
        except Exception:
            pass
        if saved:
            saved_count += 1
    return saved_count


def feed_marker_observations(coordinator, source, frame, now) -> int:
    """把 SDK marker 订阅的新鲜观测推给需要它的任务。

    **必须每帧在 coordinator.step() 之前调用**，否则任务这一步看到的是上一帧
    甚至没有观测。任务自己不能碰 SDK，所以这条通道由主循环统一喂。

    返回这一帧被喂到的任务数（给日志和测试用）。绝不抛异常。
    """
    if source is None:
        return 0
    try:
        candidates = source.candidates(frame, now)
    except Exception:
        candidates = ()
    fed = 0
    for task in getattr(coordinator, "motion_tasks", ()):
        push = getattr(task, "update_candidates", None)
        if not callable(push):
            continue
        try:
            push(candidates)
            fed += 1
        except Exception:
            pass
    return fed


def feed_robot_observations(coordinator, source, frame, now) -> int:
    """把 SDK"机器人识别"的新鲜观测推给需要它的任务（障碍模块的主路径）。

    **必须每帧在 coordinator.step() 之前调用**：任务自己不能碰 SDK，而且它靠
    `observed_at` 判断观测是否过期，所以这里必须把**回调接收时刻**原样传下去，
    不能刷新成 now —— 否则一个早就开走的车会被当成新鲜观测。

    空元组表示"这一帧没有识别到机器人"，同样要推下去，任务会据此清掉旧框。
    返回这一帧被喂到的任务数（给日志和测试用）。绝不抛异常。
    """
    if source is None:
        return 0
    try:
        rows, observed_at = source.observations(frame)
    except Exception:
        rows, observed_at = (), None
    fed = 0
    for task in getattr(coordinator, "motion_tasks", ()):
        push = getattr(task, "update_robot_observations", None)
        if not callable(push):
            continue
        try:
            push(rows, observed_at)
            fed += 1
        except Exception:
            pass
    return fed


def record_run_events(coordinator, decision, now) -> None:
    """把这一帧的协调器结果交给愿意记录它的观察者，用于生成运行记录。

    观察者是否真的记由它自己决定（`evidence.py` 只在状态变化时记一行）。
    绝不抛异常：记录功能不能影响控制循环。
    """
    for observer in getattr(coordinator, "observers", ()):
        hook = getattr(observer, "record_decision", None)
        if not callable(hook):
            continue
        try:
            hook(decision, now)
        except Exception:
            pass


#: 巡线状态 → 终端上那一行中文。丢线必须看得见，这是操作员最需要的信息。
LINE_STATE_TEXT = {
    "STOPPED": "巡线 STOPPED —— 已停止（还没起步，或刚复位）",
    "TRACKING": "巡线 TRACKING —— 正常跟线",
    "COASTING": "巡线 COASTING —— 短暂漏检，底座自己兜",
    "LINE_LOST": "巡线 LINE_LOST —— 丢线，底座开始找回",
    "VIDEO_LOST": "巡线 VIDEO_LOST —— 视频失效，已锁停",
}


class ConsoleStatus:
    """把车的状态按**变化**打到终端，操作员在 VS Code 里就能看到出了什么事。

    只打"变化"和低频心跳（默认 2 秒一行），绝不逐帧刷屏。行首记号：

        （无）巡线状态变化         >>  任务接管
        <<  任务结束              >>> 得分截图已保存
        !!  问题/异常             --  心跳

    任何打印失败都被吞掉：把状态打到屏幕这件事，绝不能影响控制循环。
    """

    def __init__(self, stream=None, heartbeat_interval: float = 2.0,
                 enabled: bool = True, task_heartbeat_interval: float = 0.5,
                 task_order=()) -> None:
        self.stream = stream
        self.heartbeat_interval = float(heartbeat_interval)
        #: 有模块接管时用更快的节奏：调优先级时要看得出"此刻谁在跑、跑了多久"。
        self.task_heartbeat_interval = float(task_heartbeat_interval)
        #: 注册表顺序 = 撞车时的裁判顺序。用来告诉操作员"谁没被轮到"。
        self.task_order = tuple(task_order or ())
        self.enabled = bool(enabled)
        self.started_at = None
        self.frames = 0
        self.saved_evidence = 0
        self._line_state = None
        self._coordinator_state = None
        self._task_name = None
        self._task_started_at = None
        self._lost_since = None
        self._last_heartbeat = None
        self._errors: tuple = ()

    # -- 对外 ----------------------------------------------------------
    def note(self, text: str) -> None:
        """打一条与当前状态无关的说明（启动信息、人工操作结果等）。"""
        if not self.enabled:
            return
        try:
            print(text, file=self.stream, flush=True)
        except Exception:
            pass

    def update(self, decision, now: float, saved_evidence: int = 0,
               claims=()) -> None:
        """每帧调用一次。

        `claims` 是协调器"竞争探测帧"的结果（那一刻每个模块各自想不想接管）。
        只在这里判断"要不要打"，绝不抛异常。
        """
        if not self.enabled:
            return
        try:
            self._update(decision, now, saved_evidence, claims)
        except Exception:
            pass

    # -- 内部 ----------------------------------------------------------
    def _modules_after(self, name):
        """排在赢家后面、这一帧根本没被问到的模块（调优先级用）。

        协调器按注册表顺序依次问，遇到第一个返回 RUNNING 的模块就接管，所以
        排在它后面的模块这一帧不会被调用。这个信息是免费的——不需要额外问任何模块。
        """
        order = list(self.task_order)
        if not name or name not in order:
            return []
        return order[order.index(name) + 1:]

    def _say(self, now: float, text: str) -> None:
        if self.started_at is None:
            stamp = 0.0
        else:
            stamp = max(0.0, now - self.started_at)
        try:
            print("[%6.1fs] %s" % (stamp, text), file=self.stream, flush=True)
        except Exception:
            pass

    def _update(self, decision, now: float, saved_evidence: int, claims=()) -> None:
        if self.started_at is None:
            self.started_at = now
        self.frames += 1

        # 先处理协调器层面（谁在开车），再处理巡线状态：接管/归还期间底座会被
        # pause 成 STOPPED，那是过渡值，报出来只会误导操作员。
        state = str(getattr(decision, "state", "") or "")
        name = getattr(decision, "task_name", None)
        if name:
            self._task_name = name
        if state != self._coordinator_state:
            previous, self._coordinator_state = self._coordinator_state, state
            message = str(getattr(decision, "message", "") or "")
            if state == TASK_ACTIVE:
                self._task_started_at = now
                self._say(
                    now,
                    ">>> 模块开始运行：%s ｜ %s" % (name or self._task_name or "?", message),
                )
                not_asked = self._modules_after(name or self._task_name)
                if not_asked:
                    # 协调器按注册表顺序问，遇到第一个 RUNNING 就停：排在它后面的
                    # 这些模块这一帧根本没被问过。这就是"优先级被截断"的位置。
                    self._say(
                        now,
                        "    优先级截断：排在它后面、这一帧没被问到的模块 → %s"
                        % ", ".join(not_asked),
                    )
            elif state == RELEASING and previous == TASK_ACTIVE:
                update = getattr(decision, "task_update", None)
                status = getattr(getattr(update, "status", None), "name", None)
                self._say(
                    now,
                    "<<< 模块结束运行：%s（%s，共 %.1fs）｜ %s"
                    % (
                        self._task_name or "?",
                        status or "-",
                        max(0.0, now - (self._task_started_at or now)),
                        message,
                    ),
                )
                self._task_started_at = None
            elif state == LINE_FOLLOWING and previous == RELEASING:
                self._say(now, "巡线恢复")
            if state != LINE_FOLLOWING:
                # 忘掉过渡期的巡线状态，回到巡线后再重新报一次真实状态。
                self._line_state = None

        line_state = getattr(getattr(decision, "line", None), "state", None)
        if line_state is not None and line_state != self._line_state:
            self._line_state = line_state
            self._lost_since = now if line_state == "LINE_LOST" else None
            # 视频失效任何时刻都要报；其余状态只在真正巡线时报。
            if state == LINE_FOLLOWING or line_state == "VIDEO_LOST":
                self._say(now, LINE_STATE_TEXT.get(line_state, "巡线 %s" % line_state))

        errors = tuple(getattr(decision, "errors", ()) or ())
        if errors and errors != self._errors:
            for item in errors:
                self._say(now, "!! %s" % item)
        self._errors = errors

        if saved_evidence:
            self.saved_evidence += int(saved_evidence)
            self._say(
                now,
                ">>> 得分截图已保存（本次第 %d 张）" % self.saved_evidence,
            )

        if claims:
            self._say(now, self._competition_text(claims))

        if self._last_heartbeat is None:
            # 第一帧不打心跳：启动横幅已经说明"还活着"，再打一行是噪音。
            self._last_heartbeat = now
        else:
            # 有模块在接管时用更快的节奏：调优先级时要看得出"此刻谁在跑、跑了多久"。
            interval = (
                self.task_heartbeat_interval
                if self._coordinator_state == TASK_ACTIVE
                else self.heartbeat_interval
            )
            if now - self._last_heartbeat >= interval:
                self._last_heartbeat = now
                self._say(now, self._heartbeat_text(decision, now))

    def _competition_text(self, claims) -> str:
        """把"谁在竞争、最后判给谁"写成一行（只在协调器的探测帧出现）。

        正常帧协调器遇到第一个 RUNNING 就停，后面的模块**根本不会被问**；
        探测帧（默认每 3 秒一次）会把所有模块都问一遍，这里就把它如实打出来。
        """
        wanted = [c for c in claims if c.get("status") == "RUNNING"]
        quiet = [c for c in claims if c.get("status") != "RUNNING"]
        parts = ["竞争探测：本帧问了 %d 个模块" % len(claims)]
        if wanted:
            parts.append("想接管 → %s" % "、".join(
                "%s(%s)" % (c.get("name"), (c.get("message") or "—")[:28]) for c in wanted))
        else:
            parts.append("无人想接管 → 继续巡线")
        if quiet:
            parts.append("不想 → %s" % "、".join(str(c.get("name")) for c in quiet))
        if wanted:
            parts.append("判给 %s（顺序里第一个想接管的）" % wanted[0].get("name"))
        return "?? " + " ｜ ".join(parts)

    def _heartbeat_text(self, decision, now: float) -> str:
        bits = [
            "运行 %.1fs" % max(0.0, now - (self.started_at or now)),
            "帧 %d" % self.frames,
        ]
        state = self._line_state or "?"
        if state == "LINE_LOST" and self._lost_since is not None:
            state += "（已丢线 %.1fs）" % max(0.0, now - self._lost_since)
        bits.append("巡线 %s" % state)
        task_name = getattr(decision, "task_name", None) or self._task_name or "无"
        if getattr(decision, "state", None) == TASK_ACTIVE and self._task_started_at is not None:
            bits.append(
                "任务 %s 已 %.1fs" % (task_name, max(0.0, now - self._task_started_at))
            )
        else:
            bits.append("任务 %s" % task_name)
        if getattr(decision, "state", None) == TASK_ACTIVE:
            command = decision.command
            bits.append(
                "实际命令 x=%.2f y=%.2f yaw=%.0f"
                % (command.forward, command.lateral, command.yaw)
            )
            update = getattr(decision, "task_update", None)
            if update is not None and update.gimbal is not None:
                bits.append(
                    "云台 pitch=%.0f yaw=%.0f"
                    % (update.gimbal.pitch, update.gimbal.yaw)
                )
            message = str(getattr(decision, "message", "") or "")
            if message:
                bits.append(message)
        return "-- " + " | ".join(bits)


def draw_debug(frame, decision):
    shown = frame.copy()
    line = decision.line
    if line is not None:
        left, top, right, bottom = line.detection.roi
        cv2.rectangle(shown, (left, top), (right, bottom), (180, 180, 180), 1)
        if line.detection.contour is not None:
            cv2.drawContours(
                shown, [line.detection.contour], -1, (0, 255, 0), 2
            )
        points = (
            (line.detection.near_point, (0, 255, 255)),
            (line.detection.far_point, (255, 255, 0)),
        )
        for point, color in points:
            if point is not None:
                cv2.circle(shown, point, 5, color, -1)
    task_detection = (
        decision.task_update.detection
        if decision.task_update is not None
        else None
    )
    if task_detection is not None and task_detection.valid:
        if task_detection.box is not None:
            left, top, right, bottom = task_detection.box
            cv2.rectangle(shown, (left, top), (right, bottom), (255, 0, 255), 2)
        if task_detection.center is not None:
            cv2.circle(shown, task_detection.center, 6, (255, 0, 255), -1)
        cv2.putText(
            shown,
            task_detection.kind,
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 0, 255),
            2,
        )
    label = (
        f"{decision.state} owner={decision.owner} "
        f"v={decision.command.forward:.2f} "
        f"yaw={decision.command.yaw:.0f}"
    )
    if decision.task_name:
        label = f"{label} task={decision.task_name}"
    color = (0, 0, 255) if decision.force_stop else (0, 255, 0)
    cv2.putText(
        shown, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
    )
    if line is not None:
        cv2.imshow("Line mask", line.detection.mask)
    return shown


def _main_window_open() -> bool:
    if not CONFIG.display:
        return True
    try:
        return cv2.getWindowProperty(
            "Low-speed line base", cv2.WND_PROP_VISIBLE
        ) >= 1.0
    except cv2.error:
        return False


def _align_camera(ep_robot, output, robot_module) -> None:
    if not output.hard_stop():
        raise RuntimeError("cannot confirm chassis stop before camera alignment")
    if ep_robot.set_robot_mode(mode=robot_module.FREE) is not True:
        raise RuntimeError("cannot enter FREE mode for camera alignment")
    action = ep_robot.gimbal.moveto(
        pitch=CONFIG.gimbal_pitch,
        yaw=CONFIG.gimbal_yaw,
        pitch_speed=CONFIG.gimbal_pitch_speed,
        yaw_speed=CONFIG.gimbal_yaw_speed,
    )
    if action.wait_for_completed() is not True:
        raise RuntimeError("gimbal alignment failed")
    if ep_robot.set_robot_mode(mode=robot_module.CHASSIS_LEAD) is not True:
        raise RuntimeError("cannot enter CHASSIS_LEAD mode")
    if not output.hard_stop():
        raise RuntimeError("cannot confirm chassis stop after camera alignment")


def main(
    motion_task_names=None,
    marker_subscription=True,
    robot_subscription=True,
    run_label="FULL",
    motion_tasks_override=None,
    gimbal_output_factory=None,
) -> None:
    # Keeping this import inside main makes every offline import hardware-safe.
    from robomaster import camera, robot

    ep_robot = robot.Robot()
    source = None
    output = None
    gimbal_output = None
    marker_source = None
    robot_source = None
    stream_started = False
    follower = LineFollower(CONFIG)
    coordinator = None
    last_sequence = 0
    have_frame = False
    # 启动横幅：必须打在"碰硬件之前"，而且 flush=True。
    # 否则连不上车时（SDK 的 initialize 没有超时，会静默阻塞）终端一片空白，
    # 现场分不清是卡住了还是根本没跑起来。
    print(
        "[main] 正在连接机器人（AP / UDP，机器人固定地址 192.168.2.1）...",
        flush=True,
    )
    print(
        "[main] 若这里卡住没有下文：车没开机，或本机没连上 RMEP-XXXX 热点。\n"
        "[main]   自检：ipconfig 里应出现 192.168.2.x，且能 ping 通 192.168.2.1。",
        flush=True,
    )
    try:
        ep_robot.initialize(conn_type="ap", proto_type="udp")
        print("[main] 机器人已连接，正在做云台起始回中 ...", flush=True)
        output = MotionOutput(ep_robot.chassis, CONFIG)
        _align_camera(ep_robot, output, robot)
        print("[main] 云台就位，正在打开视频流 ...", flush=True)
        if gimbal_output_factory is not None:
            gimbal_output = gimbal_output_factory(ep_robot, CONFIG, robot)
        elif motion_tasks_override is None and (
            motion_task_names is None or "route" in motion_task_names
        ):
            # The integrated route task uses a one-time side-looking yaw;
            # ordinary GimbalOutput clamps that request to +/-30 degrees and
            # never enters the required FREE mode.
            gimbal_output = RouteGimbalOutput(ep_robot, CONFIG, robot)
        else:
            gimbal_output = GimbalOutput(ep_robot.gimbal, CONFIG)
        coordinator = build_coordinator(
            follower,
            output,
            gimbal_output=gimbal_output,
            motion_task_names=motion_task_names,
            motion_tasks_override=motion_tasks_override,
        )
        resolution_name = f"STREAM_{CONFIG.camera_resolution.upper()}"
        resolution = getattr(camera, resolution_name)
        ep_robot.camera.start_video_stream(
            display=False, resolution=resolution
        )
        stream_started = True
        # 数字标识的观测来源：SDK 的 marker 订阅（任务模块不许自己碰 SDK）。
        # 视频流起来之后再订阅；订阅失败只是模块不触发，不影响巡线。
        if marker_subscription:
            marker_source = MarkerObservationSource(
                ep_robot.vision,
                CONFIG.marker_color,
                CONFIG.marker_coordinate_mode,
            )
            marker_source.start()
        # 障碍物模块的观测来源：SDK 的"机器人识别"（v10 起是它的主路径）。
        # 同样在视频流起来之后订阅；订阅失败它只是退回灰度结构判据，不影响巡线。
        if robot_subscription:
            robot_source = RobotObservationSource(ep_robot.vision)
            robot_source.start()
        source = LatestFrameSource(
            ep_robot.camera,
            CONFIG.camera_strategy,
            CONFIG.camera_read_timeout,
        )
        source.start()
        if CONFIG.display:
            cv2.namedWindow("Low-speed line base")
            cv2.namedWindow("Line mask")
        print(
            f"[{run_label}] Ready and stopped. "
            "SPACE resume/pause, R reset, Q/ESC quit. "
            f"{len(coordinator.motion_tasks)} task module(s) registered."
        )
        console = ConsoleStatus(
            heartbeat_interval=CONFIG.console_heartbeat_seconds,
            enabled=CONFIG.console_status,
            task_heartbeat_interval=CONFIG.console_task_heartbeat_seconds,
            task_order=tuple(task.name for task in coordinator.motion_tasks),
        )
        sink = _find_evidence_sink(coordinator)
        score_evidence = IntegratedScoreEvidence()
        run_directory = getattr(sink, "run_directory", None)
        if run_directory is not None:
            console.note(
                "运行记录目录：%s（结束时写 report.md）" % run_directory
            )
        console.note(
            "marker 订阅：%s"
            % ("成功" if marker_source is not None and marker_source.enabled
               else "未订阅")
        )
        console.note(
            "robot 订阅（障碍模块主路径）：%s"
            % ("成功" if robot_source is not None and robot_source.enabled
               else "未订阅（障碍会退回灰度结构判据，误触发率更高）")
        )
        console.note("提示：丢线/接管/异常都会打在这里，不用盯 cv2 窗口。")

        while True:
            if not _main_window_open():
                break
            packet = source.wait_after(
                last_sequence, CONFIG.consumer_wait_timeout
            )
            now = time.monotonic()
            if packet is None:
                if have_frame:
                    decision = coordinator.video_gap(source.age(now), now)
                    if decision.force_stop:
                        output.hard_stop()
                    console.update(decision, now)
                key = cv2.waitKey(1) & 0xFF if CONFIG.display else -1
            else:
                have_frame = True
                last_sequence = packet.sequence
                # 先把新鲜观测喂给任务，再让它 step，否则它看到的是上一帧的数据。
                feed_marker_observations(coordinator, marker_source, packet, now)
                feed_robot_observations(coordinator, robot_source, packet, now)
                decision = coordinator.step(packet, now)
                # Final 的得分截图链：任务交出请求 -> 证据层画框写字存盘 ->
                # 回传真实结果。放在 step() 之后，任务在等回执期间会保持接管。
                saved = service_task_evidence(coordinator)
                for task in coordinator.motion_tasks:
                    if getattr(task, "evidence_failed", False):
                        console.note("!! 红绿灯得分截图写入失败，已暂停")
                        console.note(coordinator.human_stop(now))
                        task.reset()
                failures_before = len(score_evidence.failures)
                saved += score_evidence.process(packet, decision, coordinator, sink)
                for failure in score_evidence.failures[failures_before:]:
                    console.note("!! 得分截图失败：" + failure)
                    console.note(coordinator.human_stop(now))
                # 运行记录：把这一帧的接管/释放/限幅/异常写进本次运行的 report.md。
                record_run_events(coordinator, decision, now)
                # 终端反馈：状态变化 + 心跳。丢线、接管、异常都会打出来。
                # claims = 协调器"竞争探测帧"的结果（谁想接管、判给了谁）。
                console.update(decision, now, saved, coordinator.last_claims)
                if CONFIG.display:
                    cv2.imshow(
                        "Low-speed line base",
                        draw_debug(packet.image, decision),
                    )
                key = cv2.waitKey(1) & 0xFF if CONFIG.display else -1

            if not _main_window_open():
                break

            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                console.note(coordinator.human_reset(now))
            elif key == ord(" "):
                if coordinator.task_active or follower.motion_enabled:
                    console.note(coordinator.human_stop(now))
                elif coordinator.human_resume(now):
                    console.note("Resumed on a fresh valid line.")
                else:
                    console.note("Resume refused: reset fault and show a fresh line.")
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception:
        if output is not None:
            output.hard_stop()
        raise
    finally:
        # Safety stop precedes optional recorder/UI cleanup.  A slow or broken
        # observer must never delay the final zero-speed command.
        if output is not None:
            output.hard_stop()
        if coordinator is not None:
            # 先把自检结果写进记录，再 close（close 会生成 report.md）。
            record_runtime_diagnostics(coordinator, marker_source, robot_source)
            # Flush and close the evidence recorder after the chassis stop.
            coordinator.close()
        if gimbal_output is not None:
            try:
                gimbal_output.restore_line_view()
            except Exception:
                pass
            close_gimbal_output = getattr(gimbal_output, "close", None)
            if callable(close_gimbal_output):
                try:
                    close_gimbal_output()
                except Exception:
                    pass
        if source is not None:
            source.close()
        if marker_source is not None:
            # 退订 marker 识别：不能把 SDK 的订阅留给下一次运行。
            marker_source.stop()
        if robot_source is not None:
            # 退订机器人识别：同样不能把 SDK 的订阅留给下一次运行。
            robot_source.stop()
        if stream_started:
            try:
                ep_robot.camera.stop_video_stream()
            except Exception:
                pass
        try:
            ep_robot.close()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
