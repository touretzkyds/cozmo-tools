from .base import *
from .rrt import *

class NavStep():
    FORWARD = "forward"
    BACKWARD = "backward"

    def __init__(self, type, params):
        self.type = type
        self.params = params

    def __repr__(self):
        return '<NavStep %.1f,%.1f>' % params

class NavPlan():
    def __init__(self, steps=[]):
        self.steps = steps

    @staticmethod
    def from_path(path):
        steps = []
        for node in path:
            steps.append(NavStep(NavStep.FORWARD,
                                 (node.x, node.y, node.q)))
        return NavPlan(steps)

class PilotToPose(StateNode):
    def __init__(self,pose):
        super().__init__()
        self.target_pose = pose

    def start(self,event=None):
        pose = self.robot.pose
        start = RRTNode(x=pose.position.x, y=pose.position.y,
                        q=pose.rotation.angle_z.radians)
        tpose = self.target_pose
        goal = RRTNode(x=tpose.position.x, y=tpose.position.y,
                       q=tpose.rotation.angle_z.radians)

        try:
            (treeA, treeB, path) = self.robot.world.rrt.plan_path(start,goal)
        except StartCollides:
            print('Start collides!')
            self.post_failure()
            return
        except GoalCollides:
            print('Goal collides!')
            self.post_failure()
            return
        except MaxIterations:
            print('Max iteration exceeded!')
            self.post_failure()
            return

        print(len(treeA)+len(treeB),'nodes')
        if self.robot.world.path_viewer:
            self.robot.world.path_viewer.add_tree(path, (1,1,0,0.5))

        # smooth the path...

        # construct nav plan
        plan = NavPlan.from_path(path)
