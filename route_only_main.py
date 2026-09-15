"""Real-car entry for isolated long-gap route-recovery testing.

Running this file still connects to the robot.  It keeps the normal line
follower, safety stops, centralized chassis/gimbal outputs and evidence log,
but registers only ``RouteTask``.  Unfinished final-task detectors therefore
cannot interrupt a route-recovery test.
"""

from main import main


if __name__ == "__main__":
    main(
        motion_task_names=("route",),
        marker_subscription=False,
        run_label="ROUTE-ONLY",
    )
