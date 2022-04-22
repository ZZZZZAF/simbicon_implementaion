import dartpy as dart
import numpy as np
import math


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, node):
        super(InputHandler, self).__init__()
        self.node = node
        self.force = np.zeros(3)
        self.impulse_duration = 100
        self.force_magnitude = 300
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
                rotation_matrix = dart.math.eulerXYZToMatrix([0, 0, np.radians(self.turning_angle)])
                desired_facing_direction = rotation_matrix @ self.node.get_current_facing_direction()
                self.node.set_desired_facing_direction(desired_facing_direction,300)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Right:
                rotation_matrix = dart.math.eulerXYZToMatrix([0, 0, np.radians(-self.turning_angle)])
                desired_facing_direction = rotation_matrix @ self.node.get_current_facing_direction()
                self.node.set_desired_facing_direction(desired_facing_direction, 300)
                return True
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Z:
                self.node.EnableFreeCamera()
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

        self.desired_thigh_sagital_angle13 = -0.5
        self.desired_thigh_coronal_angle13 = 0.0

        self.desired_thigh_sagital_angle24 = 0.1
        self.desired_thigh_coronal_angle24 = 0.0

        self.modified_desired_thigh_sagital_angle13 = -0.5
        self.modified_desired_thigh_coronal_angle13 = 0.0

        self.modified_desired_thigh_sagital_angle24 = 0.1
        self.modified_desired_thigh_coronal_angle24 = 0.0

        self.current_controller = 'stand'
        self.current_state = 'state0'
        self.skel.setPositions(self.state0)
        self.left_thigh = self.skel.getBodyNode('l_thigh_left')
        self.right_thigh = self.skel.getBodyNode('l_thigh_right')

        self.left_foot = self.skel.getBodyNode('l_heel_left')
        self.right_foot = self.skel.getBodyNode('l_heel_right')

        self.pelvis = self.skel.getBodyNode('l_pelvis')

        self.stanceFoot = None

        self.timestep = world.getTimeStep()

        self.freeCamera = True

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

            self.Kp[i, i] = 500.0
            self.Kd[i, i] = 50

        self.Kp_torso_sagital = 400
        self.Kd_torso_sagital = 40

        self.Kp_torso_coronal = 800
        self.Kd_torso_coronal = 80

        self.Kp_thigh_sagital = 400
        self.Kd_thigh_sagital = 40

        self.Kp_thigh_coronal = 1200
        self.Kd_thigh_coronal = 120

        self.Kp_turning = 100
        self.Kd_turning = 10


        self.desired_facing_direction = np.array([1, 0, 0])

        self.cd = 0.5
        self.cv = 0.2

        self.pre_offset = 0
        self.elapsed_time = 0

        self.ext_force_duration = 0
        self.ext_force = np.zeros(3)
        self.facing_direction_duration = 0
        self.ext_force_arrow_shape = dart.dynamics.ArrowShape([0, 0, 0], [0, 0, 0])
        self.facing_direction_arrow_shape = dart.dynamics.ArrowShape([0, 0, 0], [1, 0, 0])


        self.ext_force_simple_frame = dart.dynamics.SimpleFrame()
        self.ext_force_simple_frame.setShape(self.ext_force_arrow_shape)
        self.ext_force_visual = self.ext_force_simple_frame.createVisualAspect()
        self.ext_force_visual.setColor([1.0, 0.0, 0.0])
        self.ext_force_visual.hide()

        self.facing_direction_simple_frame = dart.dynamics.SimpleFrame()
        self.facing_direction_simple_frame.setShape(self.facing_direction_arrow_shape)
        self.facing_direction_visual = self.facing_direction_simple_frame.createVisualAspect()
        self.facing_direction_visual.setColor([1.0, 0.0, 0.0])
        self.facing_direction_visual.hide()

        self.world.addSimpleFrame(self.ext_force_simple_frame)
        self.world.addSimpleFrame(self.facing_direction_simple_frame)

    def addViewer(self, viewer):
        self.viewer = viewer

    def setCameraPosition(self):
        comFrame = self.getCOMFrame()
        eyePosition = comFrame.multiply([-5,0,1])
        targetPoint = comFrame.translation()
        self.viewer.setCameraHomePosition(eyePosition,
                                          targetPoint,
                                          [0,0,1])

    def EnableFreeCamera(self):
        if self.freeCamera == True:
            self.freeCamera = False
        else:
            self.freeCamera = True


    def customPreStep(self):
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

            if self.elapsed_time >= 0.01:
                self.current_controller = 'walk'
                self.current_state = 'state1'
                self.elapsed_time = 0

            for i in range(6):
                self.torques[i] = 0

            self.skel.setForces(self.torques)

        elif self.current_controller == 'walk':
            if self.current_state == 'state1':
                self.stanceFoot = self.right_foot
                self.elapsed_time += self.timestep

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_desired_thigh_sagital_angle13 = self.desired_thigh_sagital_angle13 - self.cd * self.sagitalD - self.cv * self.sagitalV
                self.modified_desired_thigh_coronal_angle13 = self.desired_thigh_coronal_angle13 + self.cd * self.coronalD + self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.state1)
                d = np.matmul(-self.Kd, dq)
                self.torques = p + d

                p = -self.Kp_thigh_sagital * (self.getSagitalLeftLegAngle() - self.modified_desired_thigh_sagital_angle13)
                d = -self.Kd_thigh_sagital * dq[self.left_thigh_ry]
                self.tau_left_thigh_sagital = p + d
                self.torques[self.left_thigh_ry] = self.tau_left_thigh_sagital

                p = -self.Kp_thigh_coronal * (self.getCoronalLeftLegAngle() - self.modified_desired_thigh_coronal_angle13)
                d = -self.Kd_thigh_coronal * dq[self.left_thigh_rx]
                self.tau_left_thigh_coronal = p + d
                self.torques[self.left_thigh_rx] = self.tau_left_thigh_coronal



                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                p = -self.Kp_torso_sagital * (self.pelvisSagitalAngle - 0)
                d = -self.Kd_torso_sagital * (self.pelvis.getSpatialVelocity()[1] - 0)
                self.tauTorsoSagital = p + d  # - Tao_torso
                self.torques[self.right_thigh_ry] = -self.tauTorsoSagital - self.torques[self.left_thigh_ry]  # quan zhong zai hou mian

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                p = -self.Kp_torso_coronal * self.pelvisCoronalAngle
                d = -self.Kd_torso_coronal * (self.pelvis.getSpatialVelocity()[0] - 0)
                self.tauTorsoCoronal = p + d
                self.torques[self.right_thigh_rx] = -self.tauTorsoCoronal - self.torques[self.left_thigh_rx]  # quan zhong zai hou mian


                if self.facing_direction_duration > 0:
                    self.angle_difference = self.get_angle_difference()
                    self.tauLeftThighTurning = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.skel.getCOMSpatialVelocity()[2]
                    self.torques[self.right_thigh_rz] = self.tauLeftThighTurning

                self.printState()

                if self.elapsed_time >= 0.5:
                    self.current_state = 'state2'
                    self.elapsed_time = 0

            elif self.current_state == 'state2':
                self.stanceFoot = self.right_foot

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_desired_thigh_sagital_angle24 = self.desired_thigh_sagital_angle24 - self.cd * self.sagitalD - self.cv * self.sagitalV
                self.modified_desired_thigh_coronal_angle24 = self.desired_thigh_coronal_angle24 + self.cd * self.coronalD + self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.state2)
                d = np.matmul(-self.Kd, dq)
                self.torques = p + d



                p = -self.Kp_thigh_sagital * (self.getSagitalLeftLegAngle() - self.modified_desired_thigh_sagital_angle24)
                d = -self.Kd_thigh_sagital * dq[self.left_thigh_ry]
                self.tau_left_thigh_sagital = p + d
                self.torques[self.left_thigh_ry] = self.tau_left_thigh_sagital

                p = -self.Kp_thigh_coronal * (self.getCoronalLeftLegAngle() - self.modified_desired_thigh_coronal_angle24)
                d = -self.Kd_thigh_coronal * dq[self.left_thigh_rx]
                self.tau_left_thigh_coronal = p + d
                self.torques[self.left_thigh_rx] = self.tau_left_thigh_coronal



                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                p = -self.Kp_torso_sagital * self.pelvisSagitalAngle
                d = -self.Kd_torso_sagital * (self.pelvis.getSpatialVelocity()[1] - 0)
                self.tauTorsoSagital = p + d  # - Tao_torso
                self.torques[self.right_thigh_ry] = -self.tauTorsoSagital - self.torques[self.left_thigh_ry]  # quan zhong zai hou mian

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                p = -self.Kp_torso_coronal * self.pelvisCoronalAngle
                d = -self.Kd_torso_coronal * (self.pelvis.getSpatialVelocity()[0] - 0)
                self.tauTorsoCoronal = p + d
                self.torques[self.right_thigh_rx] = -self.tauTorsoCoronal - self.torques[self.left_thigh_rx]  # quan zhong zai hou mian


                if self.facing_direction_duration > 0:
                    self.angle_difference = self.get_angle_difference()
                    self.tauLeftThighTurning = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.skel.getCOMSpatialVelocity()[2]
                    self.torques[self.right_thigh_rz] = self.tauLeftThighTurning

                self.printState()

                if constraint_forces[self.left_heel] != 0 :
                    self.current_state = 'state3'


            elif self.current_state == 'state3':
                self.stanceFoot = self.left_foot
                self.elapsed_time += self.timestep

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_desired_thigh_sagital_angle13 = self.desired_thigh_sagital_angle13 - self.cd * self.sagitalD - self.cv * self.sagitalV
                self.modified_desired_thigh_coronal_angle13 = self.desired_thigh_coronal_angle13 + self.cd * self.coronalD + self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.state3)
                d = np.matmul(-self.Kd, dq)
                self.torques = p + d



                p = -self.Kp_thigh_sagital * (self.getSagitalRightLegAngle() - self.modified_desired_thigh_sagital_angle13)
                d = -self.Kd_thigh_sagital * dq[self.right_thigh_ry]
                self.tau_right_thigh_sagital = p + d
                self.torques[self.right_thigh_ry] = self.tau_right_thigh_sagital

                p = -self.Kp_thigh_coronal * (self.getCoronalRightLegAngle() - self.modified_desired_thigh_coronal_angle13)
                d = -self.Kd_thigh_coronal * dq[self.right_thigh_rx]
                self.tau_right_thigh_coronal = p + d
                self.torques[self.right_thigh_rx] = self.tau_right_thigh_coronal



                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                p = -self.Kp_torso_sagital * self.pelvisSagitalAngle
                d = -self.Kd_torso_sagital * (self.pelvis.getSpatialVelocity()[1] - 0)
                self.tauTorsoSagital = p + d
                self.torques[self.left_thigh_ry] = -self.tauTorsoSagital - self.torques[self.right_thigh_ry]

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                p = -self.Kp_torso_coronal * self.pelvisCoronalAngle
                d = - self.Kd_torso_coronal * (self.pelvis.getSpatialVelocity()[0] - 0)
                self.tauTorsoCoronal = p + d
                self.torques[self.left_thigh_rx] = -self.tauTorsoCoronal - self.torques[self.right_thigh_rx]


                if self.facing_direction_duration > 0:
                    self.angle_difference = self.get_angle_difference()
                    self.tauLeftThighTurning = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.skel.getCOMSpatialVelocity()[2]
                    self.torques[self.left_thigh_rz] = self.tauLeftThighTurning

                self.printState()

                if self.elapsed_time >= 0.5:
                    self.current_state = 'state4'
                    self.elapsed_time = 0

            elif self.current_state == 'state4':
                self.stanceFoot = self.left_foot

                self.sagitalD = self.getSagitalCOMDistance()
                self.sagitalV = self.getSagitalCOMVelocity()
                self.coronalD = self.getCoronalCOMDistance()
                self.coronalV = self.getCoronalCOMVelority()

                self.modified_desired_thigh_sagital_angle24 = self.desired_thigh_sagital_angle24 - self.cd * self.sagitalD - self.cv * self.sagitalV
                self.modified_desired_thigh_coronal_angle24 = self.desired_thigh_coronal_angle24 + self.cd * self.coronalD + self.cv * self.coronalV

                p = np.matmul(-self.Kp, q - self.state4)
                d = np.matmul(-self.Kd, dq)
                self.torques = p + d



                p = -self.Kp_thigh_sagital * (self.getSagitalRightLegAngle() - self.modified_desired_thigh_sagital_angle24)
                d = -self.Kd_thigh_sagital * dq[self.right_thigh_ry]
                self.tau_right_thigh_sagital = p + d
                self.torques[self.right_thigh_ry] = self.tau_right_thigh_sagital

                p = -self.Kp_thigh_coronal * (self.getCoronalRightLegAngle() - self.modified_desired_thigh_coronal_angle24)
                d = -self.Kd_thigh_coronal * dq[self.right_thigh_rx]
                self.tau_right_thigh_coronal = p + d
                self.torques[self.right_thigh_rx] = self.tau_right_thigh_coronal



                self.pelvisSagitalAngle = self.getSagitalPelvisAngle()
                p = -self.Kp_torso_sagital * self.pelvisSagitalAngle
                d = -self.Kd_torso_sagital * (self.pelvis.getSpatialVelocity()[1] - 0)
                self.tauTorsoSagital = p + d
                self.torques[self.left_thigh_ry] = -self.tauTorsoSagital - self.torques[self.right_thigh_ry]

                self.pelvisCoronalAngle = self.getCoronalPelvisAngle()
                p = -self.Kp_torso_coronal * self.pelvisCoronalAngle
                d = -self.Kd_torso_coronal * (self.pelvis.getSpatialVelocity()[0] - 0)
                self.tauTorsoCoronal = p + d
                self.torques[self.left_thigh_rx] = -self.tauTorsoCoronal - self.torques[self.right_thigh_rx]


                if self.facing_direction_duration > 0:
                    self.angle_difference = self.get_angle_difference()
                    self.tauLeftThighTurning = -self.Kp_turning * self.angle_difference - self.Kd_turning * self.skel.getCOMSpatialVelocity()[2]
                    self.torques[self.left_thigh_rz] = self.tauLeftThighTurning

                self.printState()

                if constraint_forces[self.right_heel] != 0 :
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

            # Draw facing direction
            self.facing_direction_duration -= 1
            if self.facing_direction_duration <= 0:
                self.facing_direction_duration = 0

            # if self.facing_direction_duration > 0:
            #     modified_desired_facing_direction = self.desired_facing_direction.copy()
            #     modified_desired_facing_direction[2] = 1
            #     arrow_head = modified_desired_facing_direction + self.getCOMFrame().translation()
            #     arrow_tail = np.array([-modified_desired_facing_direction[0], -modified_desired_facing_direction[1], modified_desired_facing_direction[2]]) + self.getCOMFrame().translation()
            #     self.facing_direction_arrow_shape.setPositions(arrow_tail, arrow_head)
            #     self.facing_direction_arrow_shape.setDataVariance(dart.dynamics.Shape.DYNAMIC)
            #     self.facing_direction_visual.show()
            #
            # else:
            #     self.facing_direction_arrow_shape.setDataVariance(dart.dynamics.Shape.STATIC)
            #     self.facing_direction_visual.hide()

            if self.freeCamera == False:
                self.setCameraPosition()


    def printState(self):
        # print("state:", self.current_state)
        # print("sagital V", self.sagitalV)
        # print("sagital D", self.sagitalD)
        # print("coronal V", self.coronalV)
        # print("coronal D", self.coronalD)
        # print()

        # print("pelvis Sagital Angle", self.pelvisSagitalAngle)
        # print("tao Torso Sagital", self.tauTorsoSagital)
        # print("pelvis Coronal Angle", self.pelvisCoronalAngle)
        # print("tao Torso Coronal", self.tauTorsoCoronal)
        #
        # print("left thigh ry", self.torques [self.left_thigh_ry])
        # print("left thigh rx", self.torques[self.left_thigh_rx])
        # print("left thigh rz", self.torques[self.left_thigh_rz])

        # print("right thigh ry", self.torques[self.right_thigh_ry])
        # print("right thigh rx", self.torques[self.right_thigh_rx])
        # print("right thigh rz", self.torques[self.right_thigh_rz])

        # print("sagital left thigh angle:", self.getSagitalLeftLegAngle())
        # print("sagital right thigh angle:", self.getSagitalRightLegAngle())
        # print("coronal left thigh angle:", self.getCoronalLeftLegAngle())
        # print("coronal right thigh angle:", self.getCoronalRightLegAngle())

        # print("modified_desired_thigh_sagital_angle13:", self.modified_desired_thigh_sagital_angle13)
        # print("modified_desired_thigh_coronal_angle13:", self.modified_desired_thigh_coronal_angle13)
        # print("modified_desired_thigh_sagital_angle24:", self.modified_desired_thigh_sagital_angle24)
        # print("modified_desired_thigh_coronal_angle24:", self.modified_desired_thigh_coronal_angle24)

        # print("COM spatial velocity:", self.skel.getCOMSpatialVelocity())
        # print("desired facing direction:", self.desired_facing_direction)
        # print()

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

    def set_desired_facing_direction(self, fd, duration = 10):
        self.desired_facing_direction = fd
        self.facing_direction_duration = duration

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

    def getSagitalLeftLegAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        thighAxisZ = self.left_thigh.getTransform().rotation()[:, 2]

        projThighAxisZ = comR.T @ thighAxisZ
        projThighAxisZ[1] = 0
        projThighAxisZ = self.normalizeVector(projThighAxisZ)
        angle = self.getAngleBetweenTwoVectors(projThighAxisZ, comZ)
        cross = np.cross(comZ, projThighAxisZ)

        if cross[1] > 0:
            return angle
        else:
            return -angle

    def getSagitalRightLegAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        thighAxisZ = self.right_thigh.getTransform().rotation()[:, 2]

        projThighAxisZ = comR.T @ thighAxisZ
        projThighAxisZ[1] = 0
        projThighAxisZ = self.normalizeVector(projThighAxisZ)
        angle = self.getAngleBetweenTwoVectors(projThighAxisZ, comZ)
        cross = np.cross(comZ, projThighAxisZ)

        if cross[1] > 0:
            return angle
        else:
            return -angle

    def getCoronalLeftLegAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        thighAxisZ = self.left_thigh.getTransform().rotation()[:, 2]

        projThighAxisZ = comR.T @ thighAxisZ
        projThighAxisZ[0] = 0
        projThighAxisZ = self.normalizeVector(projThighAxisZ)
        angle = self.getAngleBetweenTwoVectors(projThighAxisZ, comZ)
        cross = np.cross(comZ, projThighAxisZ)

        if cross[0] > 0:
            return angle
        else:
            return -angle

    def getCoronalRightLegAngle(self):
        comR = self.getCOMFrame().rotation()
        comZ = comR[:, 2]
        thighAxisZ = self.right_thigh.getTransform().rotation()[:, 2]

        projThighAxisZ = comR.T @ thighAxisZ
        projThighAxisZ[0] = 0
        projThighAxisZ = self.normalizeVector(projThighAxisZ)
        angle = self.getAngleBetweenTwoVectors(projThighAxisZ, comZ)
        cross = np.cross(comZ, projThighAxisZ)

        if cross[0] > 0:
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
        # self.skel.setVelocity(4, 0.8)



def main():
    pathname = "/./home/sizheng/simbicon_implementation"
    world = dart.simulation.World()

    urdfParser = dart.utils.DartLoader()
    robot = urdfParser.parseSkeleton(pathname + "/final result ver2/fullbody.urdf")
    ground = urdfParser.parseSkeleton(pathname + "/final result ver2/ground.urdf")
    world.addSkeleton(robot)
    world.addSkeleton(ground)
    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.0001)

    # robot.getDof('j_pelvis_rot_z').setPosition(1.14159)

    dofs = robot.getDofs()
    for i in dofs:
        print(i.getName())
        # print(i.hasPositionLimit())
        # print(i.getPositionUpperLimit())
        # print(i.getPositionLowerLimit())
    # robot.getDof('j_pelvis_rot_z').setPosition(2)

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
    grid.setOffset([0, 0, -0.49])
    grid.setNumCells(1000)
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(1100, 0, 1600, 1200)
    viewer.setCameraHomePosition([5.0, 0.0, 1.0],
                                 [0.00, 0.00, 0.0],
                                 [0, 0, 1])

    node.addViewer(viewer)

    viewer.run()


if __name__ == "__main__":
    main()
