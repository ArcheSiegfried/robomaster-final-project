"""RoboMaster entry point. Importing this module never connects to hardware."""

import time

import cv2

from camera_source import LatestFrameSource
from config import CONFIG
from motion_output import MotionOutput
from runtime import LineFollower


def draw_debug(frame, decision):
    shown = frame.copy()
    left, top, right, bottom = decision.detection.roi
    cv2.rectangle(shown, (left, top), (right, bottom), (180, 180, 180), 1)
    if decision.detection.contour is not None:
        cv2.drawContours(
            shown, [decision.detection.contour], -1, (0, 255, 0), 2
        )
    points = (
        (decision.detection.near_point, (0, 255, 255)),
        (decision.detection.far_point, (255, 255, 0)),
    )
    for point, color in points:
        if point is not None:
            cv2.circle(shown, point, 5, color, -1)
    label = (
        f"{decision.state} "
        f"v={decision.command.forward:.2f} "
        f"yaw={decision.command.yaw:.0f}"
    )
    color = (0, 0, 255) if decision.force_stop else (0, 255, 0)
    cv2.putText(
        shown, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
    )
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
    last_sequence = 0
    have_frame = False
    try:
        ep_robot.initialize(conn_type="ap", proto_type="udp")
        output = MotionOutput(ep_robot.chassis, CONFIG)
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
        print("Ready and stopped. SPACE resume/pause, R reset, Q/ESC quit.")

        while True:
            if not _main_window_open():
                break
            packet = source.wait_after(
                last_sequence, CONFIG.consumer_wait_timeout
            )
            now = time.monotonic()
            if packet is None:
                if have_frame:
                    decision = follower.process_video_gap(source.age(now), now)
                    if decision.force_stop:
                        output.hard_stop()
                key = cv2.waitKey(1) & 0xFF if CONFIG.display else -1
            else:
                have_frame = True
                last_sequence = packet.sequence
                decision = follower.process_frame(
                    packet.image, packet.captured_at
                )
                if decision.force_stop:
                    output.hard_stop()
                elif follower.motion_enabled:
                    output.send("line", decision.command)
                if CONFIG.display:
                    cv2.imshow(
                        "Low-speed line base",
                        draw_debug(packet.image, decision),
                    )
                    cv2.imshow("Line mask", decision.detection.mask)
                key = cv2.waitKey(1) & 0xFF if CONFIG.display else -1

            if not _main_window_open():
                break

            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                follower.reset_fault(now)
                output.hard_stop()
                print("Reset: stopped; show a valid line, then press SPACE.")
            elif key == ord(" "):
                if follower.motion_enabled:
                    follower.pause(now)
                    output.hard_stop()
                    print("Paused.")
                elif follower.resume(now):
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
