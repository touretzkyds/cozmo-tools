

class RRTNode():
    def __init__(self,x=0,y=0,q=0):
        self.x = x
        self.y = y
        self.q = q

    def make_robot_obstacles(self,robot):
        result = []
        for joint in robot.kine.joints.values():
            if joint.collision_model:
                tmat = robot.kine.link_to_base(j)
                robot_obst = joint.collision_model.instantiate(tmat, self.x, self.y, self.q)
                result.append(robot_obst)
        return result

