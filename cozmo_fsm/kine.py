import math
import numpy as np

from . import transform
from . import shapes

class Joint():
    def __init__(self, name, parent=None, type='fixed', getter=(lambda:0),
                 d=0, theta=0, r=0, alpha=0,
                 collision_model=None, ctransform=transform.identity()):
        self.name = name
        self.parent = parent
        self.type = type
        if type == 'fixed':
            self.apply_q = self.fixed
        elif type == 'revolute':
            self.apply_q = self.revolute
        elif type == 'prismatic':
            self.apply_q = self.prismatic
        elif type == 'world':
            self.apply_q = self.world_joint
        else:
            raise ValueError("Type must be 'fixed', 'revolute', or 'prismatic'.")
        self.getter = getter
        self.children = []
        self.d = d
        self.theta = theta
        self.r = r
        self.alpha = alpha
        self.children = []
        self.collision_model = collision_model
        self.q = 0
        self.qmin = -math.inf
        self.qmax = math.inf
        self.parent_link_to_this_joint = transform.dh_matrix(-d,-theta,-r,-alpha)
        self.this_joint_to_parent_link = np.linalg.inv(self.parent_link_to_this_joint)

        self.solver = None

    def __repr__(self):
        if self.type == 'fixed':
            qval = 'fixed'
        else:
            qval = "q=" + ("%7.2f deg." % (self.q*180/math.pi)).strip()
        return "<Joint '%s' %s>" % (self.name, qval)

    def this_joint_to_this_link(self):
        return self.apply_q()

    def this_link_to_this_joint(self):
        return np.linalg.inv(self.this_joint_to_this_link())

    def revolute(self):
        return transform.aboutZ(-self.q)

    def prismatic(self):
        return transform.translate(0.,0.,-self.q)

    def fixed(self):
        return transform.identity()

    def world_joint(self):
        return transform.translate(self.q[0],self.q[1]).transform.aboutZ(self.q[2])

class Kinematics():
    def __init__(self,joint_list,robot):
        self.joints = dict()
        for j in joint_list:
            self.joints[j.name] = j
            if j.parent:
                j.parent.children.append(j)
        self.base = self.joints[joint_list[0].name]
        self.robot = robot
        robot.kine = self
        self.get_pose()

    def joint_to_base(self,joint):
        if isinstance(joint,str):
            joint = self.joints[joint]
        Tinv = transform.identity()
        j = joint
        while j is not self.base and j.parent is not None:
            Tinv = j.parent.this_link_to_this_joint().dot(j.this_joint_to_parent_link.dot(Tinv))
            j = j.parent
        if j:
            return Tinv
        else:
            raise Exception('Joint %s has no path to base frame' % joint)

    def link_to_base(self,joint):
        return self.joint_to_base(joint).dot(joint.this_link_to_this_joint())

    def base_to_joint(self,joint):
        return np.linalg.inv(self.joint_to_base(joint))

    def joint_to_joint(self,joint1,joint2):
        if isinstance(joint1, str):
            joint1 = self.joints[joint1]
        if isinstance(joint2, str):
            joint2 = self.joints[joint2]
        return self.base_to_joint(joint2).dot(self.joint_to_base(joint1))

    def get_pose(self):
        for j in self.joints.values():
            j.q = j.getter()
