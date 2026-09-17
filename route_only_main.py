"""Real-car entry for isolated long-gap route-recovery testing.

Running this file still connects to the robot. It keeps the normal line
follower, safety stops, centralized chassis/gimbal outputs and evidence log,
but substitutes only the experimental side-looking route task. The normal
``main.py`` route task and robot mode are unchanged.
"""

from main import main
from route_gimbal import GimbalAlignedRouteTask
from route_gimbal_output import RouteGimbalOutput


if __name__ == "__main__":
    main(
        motion_tasks_override=(GimbalAlignedRouteTask(),),
        gimbal_output_factory=RouteGimbalOutput,
        marker_subscription=False,
        run_label="ROUTE-ONLY",
    )
