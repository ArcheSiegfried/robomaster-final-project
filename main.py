"""RoboMaster entry point. Importing this module never connects to hardware."""

import time

import cv2

from camera_source import LatestFrameSource
from config import CONFIG
from coordinator import TaskCoordinator
from evidence import DEFAULT_CAPTURE_DIRECTORY
from motion_output import MotionOutput
from runtime import LineFollower
from task_registry import build_motion_tasks, build_observers


def build_coordinator(
    follower,
    output,
    settings=CONFIG,
    capture_directory=DEFAULT_CAPTURE_DIRECTORY,
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
    )


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
        pitch_speed=30,
        yaw_speed=60,
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
    stream_started = False
    follower = LineFollower(CONFIG)
    coordinator = None
    last_sequence = 0
    have_frame = False
    try:
        ep_robot.initialize(conn_type="ap", proto_type="udp")
        output = MotionOutput(ep_robot.chassis, CONFIG)
        coordinator = build_coordinator(follower, output)
        _align_camera(ep_robot, output, robot)
        resolution_name = f"STREAM_{CONFIG.camera_resolution.upper()}"
        resolution = getattr(camera, resolution_name)
        ep_robot.camera.start_video_stream(
            display=False, resolution=resolution
        )
        stream_started = True
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
                decision = coordinator.step(packet, now)
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
        if source is not None:
            source.close()
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
