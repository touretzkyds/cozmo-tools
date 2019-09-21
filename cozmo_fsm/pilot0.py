"""
To avoid circular dependencies between pilot.fsm and doorpass.fsm, we
put some pilot classes here so both pilot and doorpass can import them.
"""

from .base import *
from .rrt import *
from .events import PilotEvent
from .geometry import segment_intersect_test

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

#---------------- Navigation Plan ----------------

class NavStep():
    DRIVE = "drive"
    DOORPASS = "doorpass"
    BACKUP = "backup"

    def __init__(self, type, param):
        """For DRIVE and BACKUP types, param is a list of RRTNode instances.  The
        reason we group these into a list instead of having one node per step is that
        the DriveContinuous function is going to be interpolating over the entire sequence."""
        self.type = type
        self.param = param

    def __repr__(self):
        if self.type == NavStep.DOORPASS:
            pstring = self.param.id
        elif self.type == NavStep.DRIVE:
            psteps = [(round(node.x,1),round(node.y,1)) for node in self.param]
            pstring = repr(psteps)
        else:
            pstring = repr(self.param)
            if len(pstring) > 40:
                pstring = pstring[0:20] + ' ...' + pstring[-20:]
        return '<NavStep %s %s>' % (self.type, pstring)


class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    @staticmethod
    def intersects_doorway(node1, node2, doorways):
        for door in doorways:
            p1 = (node1.x, node1.y)
            p2 = (node2.x, node2.y)
            p3 = door[1][0]
            p4 = door[1][1]
            result = segment_intersect_test(p1, p2, p3, p4)
            #label = '**INTERSECTS**' if result else 'no_int:'
            #print(label,door[0].id,' (%.1f,%.1f)--(%.1f,%.1f)  vs  (%.1f,%.1f)--(%.1f,%.1f)' % (p1+p2+p3+p4))
            if result:
                return door[0]
        return None

    @staticmethod
    def from_path(path, doorways):
        steps = []
        door = None
        pt1 = path[0]
        for i in range(1, len(path)):
            pt2 = path[i]
            door = NavPlan.intersects_doorway(pt1,pt2,doorways)
            #print('i=',i,'pt1=',pt1,'pt2=',pt2,'door=',door)
            if door:
                i -= 1
                break
            pt1 = pt2
        new_path = path[0:i+1]
        step1 = NavStep(NavStep.DRIVE, new_path)
        steps.append(step1)
        if door:
            step2 = NavStep(NavStep.DOORPASS, door)
            steps.append(step2)
        plan = NavPlan(steps)
        return plan

    def __repr__(self):
        steps = [(('doorpass(%s)' % s.param.id) if s.type == NavStep.DOORPASS else s.type) for s in self.steps]
        return '<NavPlan %s>' % repr(steps)

