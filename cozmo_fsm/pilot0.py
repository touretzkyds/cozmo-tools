from .base import *
from .rrt import *
from .events import PilotEvent

"""
To avoid circular dependencies between pilot.fsm and doorpass.fsm, we
put some pilot classes here so both pilot and doorpass can import them.
"""

class PilotCheckStart(StateNode):
    "Fails if rrt planner indicates start_collides"

    def start(self, event=None):
        super().start(event)
        (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)
        try:
            self.robot.world.rrt.plan_path(start_node,start_node)
        except StartCollides as e:
            print('PilotCheckStart: Start collides!',e)
            self.post_event(PilotEvent(StartCollides, e.args))
            self.post_failure()
            return
        except Exception as e:
            print('PilotCheckStart: Unexpected planner exception',e)
            self.post_failure()
            return
        self.post_event(PilotEvent(True))
        self.post_success()

