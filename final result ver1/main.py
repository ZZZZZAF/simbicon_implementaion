import dartpy as dart
import numpy as np
import math


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, node):
        super(InputHandler, self).__init__()
        self.node = node
        self.force = np.zeros(3)
        self.impulse_duration = 100
        self.force_magnitude = 200
        self.turning_angle = 10

    def handle(self, ea, aa):
        if ea.getEventType() == dart.gui.osg.GUIEventAdapter.KEYDOWN:
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_1:
                ext_force = np.zeros(3)
                ext_force[0] = self.force_magnitude
                self.node.set_external_force(ext_force, self.impulse_duration)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_2:
                ext_force = np.zeros(3)
                ext_force[0] = -self.force_magnitude
                self.node.set_external_force(ext_force, self.impulse_duration)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_3:
                ext_force = np.zeros(3)
                ext_force[1] = self.force_magnitude
                self.node.set_external_force(ext_force, self.impulse_duration)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_4:
                ext_force = np.zeros(3)
                ext_force[1] = -self.force_magnitude
                self.node.set_external_force(ext_force, self.impulse_duration)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_R:
                self.node.reset_robot()
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Left:
                # print("left")
                rotation_matrix = dart.math.eulerXYZToMatrix([0, 0, np.radians(self.turning_angle)])
                desired_facing_direction = rotation_matrix @ self.node.get_desired_facing_direction()
                self.node.set_desired_facing_direction(desired_facing_direction)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Right:
                rotation_matrix = dart.math.eulerXYZToMatrix([0, 0, np.radians(-self.turning_angle)])
                desired_facing_direction = rotation_matrix @ self.node.get_desired_facing_direction()
                self.node.set_desired_facing_direction(desired_facing_direction)
                # print("right")
                return True
        return False


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super(MyWorldNode, self).__init__(world)
        self.world = world
        self.skel = skel
        self.dofs = self.skel.getNumDofs()
        self.ground_skel = world.getSkeleton("ground_skeleton")
        self.ground_link = self.ground_skel.getBodyNode("ground_link")
        print("self.dofs:", self.dofs)

        self.torso_ry = self.skel.getDof('j_abdomen_ry').getIndexInSkeleton()

        self.left_thigh_rx = self.skel.getDof('j_thigh_left_rx').getIndexInSkeleton()
        self.left_thigh_ry = self.skel.getDof('j_thigh_left_ry').getIndexInSkeleton()
        self.left_thigh_rz = self.skel.getDof('j_thigh_left_rz').getIndexInSkeleton()
        self.left_shin = self.skel.getDof('j_shin_left').getIndexInSkeleton()
        self.left_heel = self.skel.getDof('j_heel_left_ry').getIndexInSkeleton()

        self.right_thigh_rx = self.skel.getDof('j_thigh_right_rx').getIndexInSkeleton()
        self.right_thigh_ry = self.skel.getDof('j_thigh_right_ry').getIndexInSkeleton()
        self.right_thigh_rz = self.skel.getDof('j_thigh_right_rz').getIndexInSkeleton()
        self.right_shin = self.skel.getDof('j_shin_right').getIndexInSkeleton()
        self.right_heel = self.skel.getDof('j_heel_right_ry').getIndexInSkeleton()

        self.state0 = self.skel.getPositions()
        # self.state0[self.right_thigh_ry] = -0.5

        temp = self.skel.getPositions()
        temp[self.left_thigh_ry] = -0.5
        temp[self.left_shin] = 1.1
        temp[self.left_heel] = -0.6
        temp[self.right_shin] = 0.05
        temp[self.right_heel] = 0.0
        self.state1 = temp
        # print(type(self.state1))

        temp = self.skel.getPositions()
        temp[self.left_thigh_ry] = 0.1
        temp[self.left_shin] = 0.05
        temp[self.left_heel] = -0.15
        temp[self.right_shin] = 0.1
        temp[self.right_heel] = 0.0
        self.state2 = temp

        temp = self.skel.getPositions()
        temp[self.right_thigh_ry] = -0.5
        temp[self.right_shin] = 1.1
        temp[self.right_heel] = -0.6
        temp[self.left_shin] = 0.05
        temp[self.left_heel] = 0.0
        self.state3 = temp

        temp = self.skel.getPositions()
        temp[self.right_thigh_ry] = 0.1
        temp[self.right_shin] = 0.05
        temp[self.right_heel] = -0.15
        temp[self.left_shin] = 0.1
        temp[self.left_heel] = 0.0
        self.state4 = temp

        self.modified_state = self.skel.getPositions()

        self.current_controller = 'stand'
        self.current_state = 'state0'
        self.skel.setPositions(self.state0)

        self.left_heel = self.skel.getBodyNode('l_heel_left')
        self.right_heel = self.skel.getBodyNode('l_heel_right')

        self.left_toe = self.skel.getBodyNode('l_toe_left')
        self.right_toe = self.skel.getBodyNode('l_toe_right')

        self.pelvis = self.skel.getBodyNode('l_pelvis')

        self.root_joint = self.skel.getRootJoint()

        self.stanceFoot = None

        self.left_foot = [
            self.skel.getDof('j_heel_left_ry').getIndexInSkeleton(),
            self.skel.getDof('j_toe_left').getIndexInSkeleton(),
        ]

        self.right_foot = [
            self.skel.getDof('j_heel_right_ry').getIndexInSkeleton(),
            self.skel.getDof('j_toe_right').getIndexInSkeleton(),
        ]

        self.timestep = world.getTimeStep()

        # self.skel.getDof('j_pelvis_rot_y').setPosition(-0.5)
        # print(self.getSagitalPelvisAngle())

        self.Kp_stand = np.eye(self.dofs)
        self.Kd_stand = np.eye(self.dofs)

        self.Kp = np.eye(self.dofs)
        self.Kd = np.eye(self.dofs)

        self.torques = np.zeros(self.dofs)

        for i in range(6):
            self.Kp_stand[i, i] = 0.0
            self.Kd_stand[i, i] = 0.0
            self.Kp[i, i] = 0.0
            self.Kd[i, i] = 0.0

        for i in range(6, self.dofs):
            self.Kp_stand[i, i] = 700.0
            self.Kd_stand[i, i] = 70.0

            self.Kp[i, i] = 700.0
            self.Kd[i, i] = 78

        self.Kp_torso_sagital = 500
        self.Kd_torso_sagital = 175

        self.Kp_torso_coronal = 500
        self.Kd_torso_coronal = 175

        self.Kp_turning = 00
        self.Kd_turning = 0

        self.desired_facing_direction = np.array([1, 0, 0])

        self.cd = 0.5
        self.cv = 0.2

        self.pre_offset = 0
        self.elapsed_time = 0

        self.ext_force_duration = 0
        self.ext_force = np.zeros(3)

        self.ext_force_arrow_shape = dart.dynamics.ArrowShape([0, 0, 0], [0, 0, 0])

        self.ext_force_simple_frame = dart.dynamics.SimpleFrame()
        self.ext_force_simple_frame.setShape(self.ext_force_arrow_shape)
        self.ext_force_visual = self.ext_force_simple_frame.createVisualAspect()
        self.ext_force_visual.setColor([1.0, 0.0, 0.0])
        self.ext_force_visual.hide()

        self.world.addSimpleFrame(self.ext_force_simple_frame)

    def customPreStep(self):
        # time = self.world.getTime()
        q = self.skel.getPositions();

        dq = self.skel.getVelocities();

        constraint_forces = self.skel.getConstraintForces()

        pelvis_position_z = self.pelvis.getTransform().translation()[2]
        if pelvis_position_z <= 0.15:
            self.current_controller = "stand"

        if self.current_controller == 'stand':
            self.elapsed_time += self.timestep
            invM = np.linalg.inv(self.skel.getMassMatrix() + self.Kd_stand * self.timestep)
            p = np.matmul(-self.Kp_stand, q + dq * self.timestep - self.state0)
            d = np.matmul(-self.Kd_stand, dq)
            ddq = np.matmul(invM, -self.skel.getCoriolisAndGravityForces() + p + d + constraint_forces)

            self.torques = p + d + np.matmul(-self.Kd_stand, ddq) * self.timestep

            if self.elapsed_time >= 0.5:
                self.current_controller = 'walk'
                self.current_state = 'state1'
                self.elapsed_time = 0

            for i in range(6):
                self.torques[i] = 0

            self.skel.setForces(self.torques)

        elif self.current_controller == 'walk':
            if self.current_state == 'state1':
                self.stanceFoot = self.right_heel
                self.elapsed_time += self.timestep

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_state = self.state1.copy()
                self.modified_state[self.left_thigh_ry] -= self.cd * self.sagitalD
                self.modified_state[self.left_thigh_ry] -= self.cv * self.sagitalV
                self.modified_state[self.left_thigh_rx] += self.cd * self.coronalD
                self.modified_state[self.left_thigh_rx] += self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.modified_state)
                d = np.matmul(-self.Kd, dq)

                self.torques = p + d

                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                self.tauTorsoSagital = -self.Kp_torso_sagital * self.pelvisSagitalAngle - self.Kd_torso_sagital * self.root_joint.getVelocity(
                    1)  # - Tao_torso
                self.torques[self.right_thigh_ry] = -self.tauTorsoSagital - self.torques[
                    self.left_thigh_ry]  # quan zhong zai hou mian

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                self.tauTorsoCoronal = -self.Kp_torso_coronal * self.pelvisCoronalAngle - self.Kd_torso_coronal * self.root_joint.getVelocity(
                    0)
                self.torques[self.right_thigh_rx] = -self.tauTorsoCoronal + self.torques[
                    self.left_thigh_rx]  # quan zhong zai hou mian

                self.angle_difference = self.get_angle_difference()
                self.tauRightThighRZ = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.root_joint.getVelocity(2)
                self.torques[self.right_thigh_rz] = self.tauRightThighRZ

                self.printState()

                if self.elapsed_time >= 0.3:
                    self.current_state = 'state2'
                    self.elapsed_time = 0

            elif self.current_state == 'state2':
                self.stanceFoot = self.right_heel

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_state = self.state2.copy()
                self.modified_state[self.left_thigh_ry] -= self.cd * self.sagitalD
                self.modified_state[self.left_thigh_ry] -= self.cv * self.sagitalV
                self.modified_state[self.left_thigh_rx] += self.cd * self.coronalD
                self.modified_state[self.left_thigh_rx] += self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.modified_state)
                d = np.matmul(-self.Kd, dq)
                self.torques = p + d

                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                self.tauTorsoSagital = -self.Kp_torso_sagital * self.pelvisSagitalAngle - self.Kd_torso_sagital * self.root_joint.getVelocity(
                    1)  # - Tao_torso
                self.torques[self.right_thigh_ry] = -self.tauTorsoSagital - self.torques[self.left_thigh_ry]

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                self.tauTorsoCoronal = -self.Kp_torso_coronal * self.pelvisCoronalAngle - self.Kd_torso_coronal * self.root_joint.getVelocity(
                    0)
                self.torques[self.right_thigh_rx] = -self.tauTorsoCoronal + self.torques[self.left_thigh_rx]

                self.angle_difference = self.get_angle_difference()
                self.tauRightThighRZ = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.root_joint.getVelocity(2)
                self.torques[self.right_thigh_rz] = self.tauRightThighRZ

                self.printState()

                if (constraint_forces[self.left_foot[0]] != 0 or constraint_forces[self.left_foot[1]] != 0):
                    self.current_state = 'state3'


            elif self.current_state == 'state3':
                self.stanceFoot = self.left_heel
                self.elapsed_time += self.timestep

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_state = self.state3.copy()
                self.modified_state[self.right_thigh_ry] -= self.cd * self.sagitalD
                self.modified_state[self.right_thigh_ry] -= self.cv * self.sagitalV
                self.modified_state[self.right_thigh_rx] += self.cd * self.coronalD
                self.modified_state[self.right_thigh_rx] += self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.modified_state)
                d = np.matmul(-self.Kd, dq)

                self.torques = p + d

                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                self.tauTorsoSagital = -self.Kp_torso_sagital * self.pelvisSagitalAngle - self.Kd_torso_sagital * self.root_joint.getVelocity(
                    1)  # - Tao_torso
                self.torques[self.left_thigh_ry] = -self.tauTorsoSagital - self.torques[self.right_thigh_ry]

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                self.tauTorsoCoronal = -self.Kp_torso_coronal * self.pelvisCoronalAngle - self.Kd_torso_coronal * self.root_joint.getVelocity(
                    0)
                self.torques[self.left_thigh_rx] = -self.tauTorsoCoronal + self.torques[self.right_thigh_rx]

                self.angle_difference = self.get_angle_difference()
                self.tauLeftThighRZ = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.root_joint.getVelocity(2)
                self.torques[self.left_thigh_rz] = self.tauLeftThighRZ

                self.printState()

                if self.elapsed_time >= 0.3:
                    self.current_state = 'state4'
                    self.elapsed_time = 0

            elif self.current_state == 'state4':
                self.stanceFoot = self.left_heel

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_state = self.state4.copy()
                self.modified_state[self.right_thigh_ry] -= self.cd * self.sagitalD
                self.modified_state[self.right_thigh_ry] -= self.cv * self.sagitalV
                self.modified_state[self.right_thigh_rx] += self.cd * self.coronalD
                self.modified_state[self.right_thigh_rx] += self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.modified_state)
                d = np.matmul(-self.Kd, dq)

                self.torques = p + d

                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                self.tauTorsoSagital = -self.Kp_torso_sagital * self.pelvisSagitalAngle - self.Kd_torso_sagital * self.root_joint.getVelocity(
                    1)  # - Tao_torso
                self.torques[self.left_thigh_ry] = -self.tauTorsoSagital - self.torques[self.right_thigh_ry]
                #
                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                self.tauTorsoCoronal = -self.Kp_torso_coronal * self.pelvisCoronalAngle - self.Kd_torso_coronal * self.root_joint.getVelocity(
                    0)
                self.torques[self.left_thigh_rx] = -self.tauTorsoCoronal + self.torques[self.right_thigh_rx]

                self.angle_difference = self.get_angle_difference()
                self.tauLeftThighRZ = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.root_joint.getVelocity(2)
                self.torques[self.left_thigh_rz] = self.tauLeftThighRZ

                self.printState()

                if (constraint_forces[self.right_foot[0]] != 0 or constraint_forces[self.right_foot[1]] != 0):
                    self.current_state = 'state1'

            for i in range(6):
                self.torques[i] = 0

            self.skel.setForces(self.torques)

            # Apply external force
            self.ext_force_duration = self.ext_force_duration - 1
            if self.ext_force_duration <= 0:
                self.ext_force_duration = 0
                self.ext_force = np.zeros(3)

            self.pelvis.addExtForce(self.ext_force)
            if self.ext_force_duration > 0:
                arrow_head = self.pelvis.getTransform().translation()
                arrow_tail = arrow_head - self.ext_force / 30
                self.ext_force_arrow_shape.setPositions(arrow_tail, arrow_head)
                self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.DYNAMIC)
                self.ext_force_visual.show()
            else:
                self.ext_force_arrow_shape.setDataVariance(dart.dynamics.Shape.STATIC)
                self.ext_force_visual.hide()



    def printState(self):
        # print("state:", self.current_state)
        # print("sagital D", self.sagitalD)
        # print("sagital V", self.sagitalV)
        # print("coronal D", self.coronalD)
        # print("coronal V", self.coronalV)
        #
        # print("left thigh ry:", self.modified_state[self.left_thigh_ry])
        # print("left thigh rx:", self.modified_state[self.left_thigh_rx])
        # print("right thigh ry:", self.modified_state[self.right_thigh_ry])
        # print("right thigh rx:", self.modified_state[self.right_thigh_rx])

        # print("pelvis Sagital Angle", self.pelvisSagitalAngle)
        # print("tao Torso Sagital", self.tauTorsoSagital)
        # print("pelvis Coronal Angle", self.pelvisCoronalAngle)
        # print("tao Torso Coronal", self.tauTorsoCoronal)
        #
        # print("left thigh ry", self.torques [self.left_thigh_ry])
        # print("left thigh rx", self.torques[self.left_thigh_rx])
        # print("right thigh ry", self.torques[self.right_thigh_ry])
        # print("right thigh rx", self.torques[self.right_thigh_rx])

        # print(self.angle_difference)
        # print()
        print(self.skel.getPosition(self.left_thigh_rx))
        pass

    def get_angle_difference(self):
        self.current_facing_direction = self.get_current_facing_direction()
        angle = self.getAngleBetweenTwoVectors(self.current_facing_direction, self.desired_facing_direction)
        cross = np.cross(self.current_facing_direction, self.desired_facing_direction)

        if cross[2] > 0:
            return angle
        else:
            return -angle


    def get_current_facing_direction(self):
        xAxis = self.getCOMFrame().rotation()[:, 0]
        return xAxis

    def get_desired_facing_direction(self):
        return self.desired_facing_direction

    def set_desired_facing_direction(self, fd):
        self.desired_facing_direction = fd

    def getCOMFrame(self):
        T = np.identity(4)

        # Z-axis
        zAxis = np.array([0, 0, 1])

        # X-axis
        pelvisXAxis = self.pelvis.getTransform().rotation()[:, 0]
        mag = np.dot(zAxis, pelvisXAxis)
        pelvisXAxis -= mag * zAxis
        xAxis = self.normalizeVector(pelvisXAxis)

        # Y-axis
        yAxis = np.cross(zAxis, xAxis)

        T[0:3, 3] = self.skel.getCOM()
        T[0:3, 0] = xAxis
        T[0:3, 1] = yAxis
        T[0:3, 2] = zAxis

        return dart.math.Isometry3(T)

    def getSagitalPelvisAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        pelvisZ = self.pelvis.getTransform().rotation()[:, 2]

        projPelvisZ = comR.T @ pelvisZ
        projPelvisZ[1] = 0
        projPelvisZ = self.normalizeVector(projPelvisZ)
        angle = self.getAngleBetweenTwoVectors(projPelvisZ, comZ)
        cross = np.cross(comZ, projPelvisZ)

        if cross[1] > 0:
            return angle
        else:
            return -angle

    def getCoronalPelvisAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        pelvisZ = self.pelvis.getTransform().rotation()[:, 2]

        projPelvisZ = comR.T @ pelvisZ
        projPelvisZ[0] = 0
        projPelvisZ = self.normalizeVector(projPelvisZ)
        angle = self.getAngleBetweenTwoVectors(projPelvisZ, comZ)
        cross = np.cross(comZ, projPelvisZ)

        if cross[0] > 0:  # wang zuo mian xie shi zheng
            return angle
        else:
            return -angle

    def normalizeVector(self, v):
        l2norm = np.linalg.norm(v, ord=2)
        result = v / l2norm
        return result

    def getAngleBetweenTwoVectors(self, v1, v2):
        return np.arccos((np.dot(v1, v2) / np.linalg.norm(v1) * np.linalg.norm(v2)))

    def getSagitalCOMDistance(self):
        xAxis = self.getCOMFrame().rotation()[:, 0]
        d = self.skel.getCOM() - self.getStanceAnklePosition()
        return np.dot(d, xAxis)

    def getSagitalCOMVelocity(self):
        xAxis = self.getCOMFrame().rotation()[:, 0]
        v = self.skel.getCOMLinearVelocity()
        return np.dot(v, xAxis)

    def getCoronalCOMDistance(self):
        yAxis = self.getCOMFrame().rotation()[:, 1]
        d = self.skel.getCOM() - self.getStanceAnklePosition()
        return np.dot(d, yAxis)

    def getCoronalCOMVelority(self):
        yAxis = self.getCOMFrame().rotation()[:, 1]
        v = self.skel.getCOMLinearVelocity()
        return np.dot(v, yAxis)

    def getStanceAnklePosition(self):
        if self.stanceFoot == None:
            return self.skel.getCOM()
        else:
            return self.getJointPosition(self.stanceFoot)

    def getJointPosition(self, bodyNode):
        parentJoint = bodyNode.getParentJoint()
        localJointPosition = parentJoint.getTransformFromChildBodyNode().translation()
        return bodyNode.getTransform().multiply(localJointPosition)

    def set_external_force(self, force, duration=10):
        self.ext_force = force
        self.ext_force_duration = duration

    def reset_robot(self):
        self.skel.setPositions(self.state0)
        self.skel.setVelocities(np.zeros(self.dofs))


def main():
    pathname = "/./home/sizheng/桌面"
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    robot = urdfParser.parseSkeleton(pathname + "/final result/fullbody.urdf")
    ground = urdfParser.parseSkeleton(pathname + "/final result/ground.urdf")
    world.addSkeleton(robot)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.001)

    # robot.getDof('j_pelvis_rot_z').setPosition(1.14159)

    dofs = robot.getDofs()
    for i in dofs:
        print(i.getName())
        # print(i.hasPositionLimit())
        # print(i.getPositionUpperLimit())
        # print(i.getPositionLowerLimit())
    # robot.getDof('j_pelvis_rot_z').setPosition(1)

    r = 0.05
    ball = dart.dynamics.EllipsoidShape(math.sqrt(2) * np.ones(3) * r)

    num_bodynode = robot.getNumBodyNodes()
    for i in range(num_bodynode):
        bodynode = robot.getBodyNode(i)
        shape_node = bodynode.createShapeNode(ball)
        visual = shape_node.createVisualAspect()
        visual.setColor([1, 0, 0, 0.8])
    # bodynode = robot.getBodyNode("l_thigh_left")
    # transformation = bodynode.getTransform()
    # print(transformation)

    # Create world node and add it to viewer
    node = MyWorldNode(world, robot)

    # create a viewer with background color (red, green, blue, alpha), here: white
    # viewer = dart.gui.osg.Viewer([1.0, 1.0, 1.0, 1.0])
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    input_handler = InputHandler(node)
    viewer.addEventHandler(input_handler)

    # Grid settings
    grid = dart.gui.osg.GridVisual()
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.XY)
    grid.setOffset([0, 0, -0.5])
    grid.setNumCells(1000)
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(1100, 0, 1600, 1200)
    viewer.setCameraHomePosition([5.0, 0.0, 1.0],
                                 [0.00, 0.00, 0.0],
                                 [0, 0, 1])
    viewer.run()


if __name__ == "__main__":
    main()
