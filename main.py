"""RoboMaster entry point. Importing this module never connects to hardware."""

import time

import cv2

from camera_source import LatestFrameSource
from config import CONFIG
from coordinator import TaskCoordinator
from evidence import DEFAULT_CAPTURE_DIRECTORY
from gimbal_output import GimbalOutput
from marker_source import MarkerObservationSource
from motion_output import MotionOutput
from runtime import LineFollower
from task_registry import build_motion_tasks, build_observers


def build_coordinator(
    follower,
    output,
    settings=CONFIG,
    capture_directory=DEFAULT_CAPTURE_DIRECTORY,
    gimbal_output=None,
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
        motion_tasks=build_motion_tasks(),
        observers=build_observers(capture_directory),
        gimbal_output=gimbal_output,
    )


def _find_evidence_sink(coordinator):
    """找到唯一的证据写入器（实现了 save_task_evidence 的观察者）。"""
    for observer in getattr(coordinator, "observers", ()):
        if callable(getattr(observer, "save_task_evidence", None)):
            return observer
    return None


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


def main() -> None:
    # Keeping this import inside main makes every offline import hardware-safe.
    from robomaster import camera, robot

    ep_robot = robot.Robot()
    source = None
    output = None
    gimbal_output = None
    marker_source = None
    stream_started = False
    follower = LineFollower(CONFIG)
    coordinator = None
    last_sequence = 0
    have_frame = False
    try:
        ep_robot.initialize(conn_type="ap", proto_type="udp")
        output = MotionOutput(ep_robot.chassis, CONFIG)
        _align_camera(ep_robot, output, robot)
        gimbal_output = GimbalOutput(ep_robot.gimbal, CONFIG)
        coordinator = build_coordinator(
            follower, output, gimbal_output=gimbal_output
        )
        resolution_name = f"STREAM_{CONFIG.camera_resolution.upper()}"
        resolution = getattr(camera, resolution_name)
        ep_robot.camera.start_video_stream(
            display=False, resolution=resolution
        )
        stream_started = True
        # 数字标识的观测来源：SDK 的 marker 订阅（任务模块不许自己碰 SDK）。
        # 视频流起来之后再订阅；订阅失败只是模块不触发，不影响巡线。
        marker_source = MarkerObservationSource(
            ep_robot.vision,
            CONFIG.marker_color,
            CONFIG.marker_coordinate_mode,
        )
        if not marker_source.start():
            print("marker observations: NOT subscribed — 数字标识不会触发。")
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
            "Ready and stopped. SPACE resume/pause, R reset, Q/ESC quit. "
            f"{len(coordinator.motion_tasks)} task module(s) registered."
        )

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
                key = cv2.waitKey(1) & 0xFF if CONFIG.display else -1
            else:
                have_frame = True
                last_sequence = packet.sequence
                # 先把新鲜观测喂给任务，再让它 step，否则它看到的是上一帧的数据。
                feed_marker_observations(coordinator, marker_source, packet, now)
                decision = coordinator.step(packet, now)
                # Final 的得分截图链：任务交出请求 -> 证据层画框写字存盘 ->
                # 回传真实结果。放在 step() 之后，任务在等回执期间会保持接管。
                service_task_evidence(coordinator)
                # 运行记录：把这一帧的接管/释放/限幅/异常写进本次运行的 report.md。
                record_run_events(coordinator, decision, now)
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
                print(coordinator.human_reset(now))
            elif key == ord(" "):
                if coordinator.task_active or follower.motion_enabled:
                    print(coordinator.human_stop(now))
                elif coordinator.human_resume(now):
                    print("Resumed on a fresh valid line.")
                else:
                    print("Resume refused: reset fault and show a fresh line.")
    except KeyboardInterrupt:
        print("Interrupted.")
    except Exception:
        if output is not None:
            output.hard_stop()
        raise
    finally:
        if coordinator is not None:
            # Flush and close the evidence recorder before tearing anything else
            # down. Never raises.
            coordinator.close()
        if output is not None:
            output.hard_stop()
        if gimbal_output is not None:
            try:
                gimbal_output.restore_line_view()
            except Exception:
                pass
        if source is not None:
            source.close()
        if marker_source is not None:
            # 退订 marker 识别：不能把 SDK 的订阅留给下一次运行。
            marker_source.stop()
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
