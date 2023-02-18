"""
To avoid circular dependencies between pilot.fsm, doorpass.fsm, and
path_planner.py, we put some pilot classes here so everyone can import
them.

"""

from .base import *
from .rrt import *
from .events import PilotEvent

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
            self.post_event(PilotEvent(StartCollides, args=e.args))
            self.post_failure()
            return
        except Exception as e:
            print('PilotCheckStart: Unexpected planner exception',e)
            self.post_failure()
            return
        self.post_success()


class PilotCheckStartDetail(StateNode):
    "Posts collision object if rrt planner indicates start_collides"

    def start(self, event=None):
        super().start(event)
        (pose_x, pose_y, pose_theta) = self.robot.world.particle_filter.pose
        start_node = RRTNode(x=pose_x, y=pose_y, q=pose_theta)
        try:
            self.robot.world.rrt.plan_path(start_node,start_node)
        except StartCollides as e:
            print('PilotCheckStartDetail: Start collides!',e)
            self.post_event(PilotEvent(StartCollides, args=e.args))
            self.post_data(e.args)
            return
        except Exception as e:
            print('PilotCheckStartDetail: Unexpected planner exception',e)
            self.post_failure()
            return
        self.post_success()

#---------------- Navigation Plan ----------------

class NavStep():
    DRIVE = "drive"
    DOORPASS = "doorpass"
    BACKUP = "backup"

    def __init__(self, type, param):
        """For DRIVE and BACKUP types, param is a list of RRTNode instances.  The
        reason we group these into a list instead of having one node per step is that
        the DriveContinuous function is going to be interpolating over the entire sequence.
        For a DOORPASS step the param is the door object."""
        self.type = type
        self.param = param

    def __repr__(self):
        if self.type == NavStep.DOORPASS:
            pstring = self.param.id
        elif self.type == NavStep.DRIVE:
            psteps = [(round(node.x,1),round(node.y,1)) for node in self.param]
            pstring = repr(psteps)
        else:   # NavStep.BACKUP and anything else
            pstring = repr(self.param)
            if len(pstring) > 40:
                pstring = pstring[0:20] + ' ...' + pstring[-20:]
        return '<NavStep %s %s>' % (self.type, pstring)


class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    def __repr__(self):
        steps = [(('doorpass(%s)' % s.param.id) if s.type == NavStep.DOORPASS else s.type) for s in self.steps]
        return '<NavPlan %s>' % repr(steps)

    def extract_path(self):
        nodes = []
        for step in self.steps:
            if step.type in (NavStep.DRIVE, NavStep.BACKUP):
                nodes += step.param
        return nodes
