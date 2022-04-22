import dartpy as dart
import numpy as np
import math


class InputHandler(dart.gui.osg.GUIEventHandler):
    def __init__(self, node, viewer):
        super(InputHandler, self).__init__()
        self.node = node
        self.viewer = viewer

    def handle(self, ea, aa):
        if ea.getEventType() == dart.gui.osg.GUIEventAdapter.KEYDOWN:
            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Left:
                self.node.desired_pelvis_target_angle_wrt_gf_z += 0.2
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Right:
                self.node.desired_pelvis_target_angle_wrt_gf_z -= 0.2
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Up:
                if (self.node.current_controller == 'stand'):
                    self.node.current_controller = 'walk'
                    self.node.current_state = 'state1'
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Down:
                if (self.node.current_controller == 'walk'):
                    self.node.current_controller = 'stand'
                    self.node.current_state = 'state0'
                    self.node.skel.setVelocities(np.zeros(self.node.dofs))
                    self.node.elapsed_time = 0
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_X:
                self.node.restart()
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Q:
                self.node.torso_target_angle_wrt_gf_pitch = 0.2
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_A:
                self.node.torso_target_angle_wrt_gf_pitch = 0.0
                return True

            if ea.getKey() == dart.gui.osg.GUIEventAdapter.KEY_Z:
                pelvis_transform = self.node.pelvis.getTransform()
                target_point = pelvis_transform.translation()
                target_point[2] += 1
                camera_position1 = [-10, 0, 6]
                camera_position2 = pelvis_transform.multiply(camera_position1)

                self.viewer.setCameraHomePosition(camera_position2,
                                                  target_point,
                                                  [0, 0, 1])
                return True
        return False


class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def __init__(self, world, skel):
        super(MyWorldNode, self).__init__(world)
        self.world = world
        self.skel = skel

        self.board = world.getSkeleton('board_skeleton1')
        self.base = world.getSkeleton('base_skeleton1')
        self.lar_pendulum = world.getSkeleton('large pendulum skeleton')
        self.windmill1 = world.getSkeleton('windmill_skeleton1')
        self.windmill2 = world.getSkeleton("windmill_skeleton2")
        self.windmill3 = world.getSkeleton("windmill_skeleton3")

        self.dofs = self.skel.getNumDofs()
        print("self.dofs:", self.dofs)

        self.torso = self.skel.getBodyNode('l_abdomen')
        self.pelvis = self.skel.getBodyNode('l_pelvis')
        self.left_thigh = self.skel.getBodyNode('l_thigh_left')
        self.right_thigh = self.skel.getBodyNode('l_thigh_right')
        self.left_scapula = self.skel.getBodyNode('l_scapula_left')

        self.pelvis_rz = self.skel.getDof('j_pelvis_rot_z').getIndexInSkeleton()

        self.torso_rx = self.skel.getDof('j_abdomen_rx').getIndexInSkeleton()
        self.torso_ry = self.skel.getDof('j_abdomen_ry').getIndexInSkeleton()

        self.left_thigh_rx = self.skel.getDof('j_thigh_left_rx').getIndexInSkeleton()
        self.left_thigh_ry = self.skel.getDof('j_thigh_left_ry').getIndexInSkeleton()
        self.left_thigh_rz = self.skel.getDof('j_thigh_left_rz').getIndexInSkeleton()

        self.left_shin = self.skel.getDof('j_shin_left').getIndexInSkeleton()
        self.left_heel = self.skel.getDof('j_heel_left_ry').getIndexInSkeleton()
        self.left_bicep = self.skel.getDof('j_bicep_left_ry').getIndexInSkeleton()
        self.left_forearm = self.skel.getDof('j_forearm_left').getIndexInSkeleton()

        self.right_thigh_rx = self.skel.getDof('j_thigh_right_rx').getIndexInSkeleton()
        self.right_thigh_ry = self.skel.getDof('j_thigh_right_ry').getIndexInSkeleton()
        self.right_thigh_rz = self.skel.getDof('j_thigh_right_rz').getIndexInSkeleton()

        self.right_shin = self.skel.getDof('j_shin_right').getIndexInSkeleton()
        self.right_heel = self.skel.getDof('j_heel_right_ry').getIndexInSkeleton()
        self.right_bicep = self.skel.getDof('j_bicep_right_ry').getIndexInSkeleton()
        self.right_forearm = self.skel.getDof('j_forearm_right').getIndexInSkeleton()

        temp0 = self.skel.getPositions()
        temp0[self.left_thigh_ry] = -0.15
        temp0[self.left_shin] = 0.4
        temp0[self.left_heel] = -0.25
        temp0[self.right_thigh_ry] = -0.15
        temp0[self.right_shin] = 0.4
        temp0[self.right_heel] = -0.25
        self.state0 = temp0

        self.desired_pelvis_target_angle_wrt_gf_x = 0.0
        self.desired_pelvis_target_angle_wrt_gf_y = 0.0
        self.desired_pelvis_target_angle_wrt_gf_z = 0.0

        self.pelvis_x = 0
        self.pelvis_y = 0
        self.pelvis_z = 0

        temp1 = self.skel.getPositions()
        temp1[self.left_thigh_ry] = -0.5
        temp1[self.left_shin] = 1.1
        temp1[self.left_heel] = -0.6
        temp1[self.left_bicep] = 0.3
        temp1[self.left_forearm] = -0.2
        temp1[self.right_thigh_ry] = 0.0
        temp1[self.right_shin] = 0.05
        temp1[self.right_heel] = 0.0
        temp1[self.right_bicep] = -0.3
        temp1[self.right_forearm] = -0.2
        self.state1 = temp1

        temp2 = self.skel.getPositions()
        temp2[self.left_thigh_ry] = 0.1
        temp2[self.left_shin] = 0.05
        temp2[self.left_heel] = -0.15
        temp2[self.right_thigh_ry] = 0.0
        temp2[self.right_shin] = 0.1
        temp2[self.right_heel] = 0.0
        self.state2 = temp2

        temp3 = self.skel.getPositions()
        temp3[self.right_thigh_ry] = -0.5
        temp3[self.right_shin] = 1.1
        temp3[self.right_heel] = -0.6
        temp3[self.right_bicep] = 0.3
        temp3[self.right_forearm] = -0.2
        temp3[self.left_thigh_ry] = 0.0
        temp3[self.left_shin] = 0.05
        temp3[self.left_heel] = 0.0
        temp3[self.left_bicep] = -0.3
        temp3[self.left_forearm] = -0.2
        self.state3 = temp3

        temp4 = self.skel.getPositions()
        temp4[self.right_thigh_ry] = 0.1
        temp4[self.right_shin] = 0.05
        temp4[self.right_heel] = -0.15
        temp4[self.left_thigh_ry] = 0.0
        temp4[self.left_shin] = 0.1
        temp4[self.left_heel] = 0.0
        self.state4 = temp4

        temp5 = self.skel.getPositions()
        self.state5 = temp5

        self.current_controller = 'stand'
        self.current_state = 'state0'
        self.skel.setPositions(self.state0)

        self.left_heel = self.skel.getBodyNode('l_heel_left')
        self.right_heel = self.skel.getBodyNode('l_heel_right')

        self.left_foot = [
            self.skel.getDof('j_heel_left_ry').getIndexInSkeleton()
        ]

        self.right_foot = [
            self.skel.getDof('j_heel_right_ry').getIndexInSkeleton()
        ]
        self.timestep = world.getTimeStep()

        self.cd = 0.5
        self.cv = 0.2
        self.Kp_0 = np.eye(self.dofs)
        self.Kd_0 = np.eye(self.dofs)

        self.Kp_walk = np.eye(self.dofs)
        self.Kd_walk = np.eye(self.dofs)

        self.Kp_walk = np.eye(self.dofs)
        self.Kd_walk = np.eye(self.dofs)

        self.torques = np.zeros(self.dofs)

        self.torso_target_angle_wrt_gf_roll = 0.0
        self.torso_target_angle_wrt_gf_pitch = 0.0

        self.swh_target_angle_wrt_gf_13_x = -0.5
        self.swh_target_angle_wrt_gf_13_y = 0

        self.swh_target_angle_wrt_gf_24_x = 0.1
        self.swh_target_angle_wrt_gf_24_y = 0

        self.restart_point1 = [8, 0, 0.1]
        self.restart_point1_pelvis_orientation = [0, 0, -1.57]
        self.restart_point1_desired_pelvis_target_angle = -1.57

        self.restart_point2 = [9, -5, 0.1]
        self.restart_point2_pelvis_orientation = [0, 0, -1.57]
        self.restart_point2_desired_pelvis_target_angle = -1.57

        self.restart_point3 = [28, -8, 0.3]
        self.restart_point3_pelvis_orientation = [0, 0, 1.57]
        self.restart_point3_desired_pelvis_target_angle = 1.57

        self.restart_point4 = [27, 6, 0.3]
        self.restart_point4_pelvis_orientation = [0, 0, 0]
        self.restart_point4_desired_pelvis_target_angle = 0

        for i in range(6):
            self.Kp_0[i, i] = 1000.0
            self.Kd_0[i, i] = 100.0

            self.Kp_walk[i, i] = 1000.0
            self.Kd_walk[i, i] = 100.0

        for i in range(6, self.dofs):
            self.Kp_0[i, i] = 300.0
            self.Kd_0[i, i] = 30.0

            self.Kp_walk[i, i] = 400.0  # 300
            self.Kd_walk[i, i] = 40

        self.Kp = 400.0  # .t global frame coefficient
        self.Kd = 40.0

        self.Kp_turning = 60.0
        self.Kd_turning = 6.0

        self.Kp_pelvis_torque = 1000.0
        self.Kd_pelvis_torque = 100.0

        self.pre_offset = 0
        self.elapsed_time = 0

    def customPreStep(self):
        self.pelvis_transform = self.pelvis.getTransform()
        self.pelvis_translation = self.pelvis_transform.translation()
        pelvis_rotation_matrix = self.pelvis_transform.rotation()
        self.pelvis_rotation_euler_XYZ = dart.math.matrixToEulerXYZ(pelvis_rotation_matrix)
        # print(self.pelvis_rotation_euler_XYZ)

        # if (self.pelvis_rotation_euler_XYZ[0] > 0.5
        #         or self.pelvis_rotation_euler_XYZ[0] < -0.5
        #         or self.pelvis_rotation_euler_XYZ[1] > 0.5
        #         or self.pelvis_rotation_euler_XYZ[1] < -0.50
        #         or self.pelvis_translation[2] < 0.5):
        #     # self.restart()
        #     self.current_controller = 'fall down'
        #     self.current_state = 'state5'

        self.pelvis_rotation_euler_XYZ_with_xy_zero = self.pelvis_rotation_euler_XYZ.copy()
        self.pelvis_rotation_euler_XYZ_with_xy_zero[0:2] = 0
        # print(self.pelvis_rotation_euler_XYZ)
        pelvis_rotation_matrix = dart.math.eulerXYZToMatrix(self.pelvis_rotation_euler_XYZ_with_xy_zero)

        simpleFrame = dart.dynamics.SimpleFrame()
        simpleFrame.setTranslation(np.array([0, 0, 0]))
        simpleFrame.setRotation(pelvis_rotation_matrix)

        pelvisFrame = dart.dynamics.SimpleFrame()
        pelvisFrame.setTranslation(self.pelvis_translation)
        pelvisFrame.setRotation(pelvis_rotation_matrix)

        q = self.skel.getPositions()

        dq = self.skel.getVelocities()
        # print("left thigh v :", dq[self.left_thigh_ry])
        # print("right thigh v :", dq[self.right_thigh_ry])

        constraint_forces = self.skel.getConstraintForces()

        COM = self.skel.getCOM(simpleFrame)
        # print(COM)

        if self.current_controller == 'stand':
            p = np.matmul(-self.Kp_0, q - self.state0)
            d = np.matmul(-self.Kd_0, dq)

            self.torques = p + d

            cop1 = self.left_heel.getTransform(pelvisFrame).multiply([0.05, 0, 0])
            cop2 = self.right_heel.getTransform(pelvisFrame).multiply([0.05, 0, 0])

            cop = (cop1[0] + cop2[0]) / 2
            offset = COM[0] - cop

            if offset < 1 and offset > 0:
                k1 = 1000
                kd = 100
                self.torques[self.left_foot[0]] += k1 * offset + kd * (offset - self.pre_offset)
                self.torques[self.right_foot[0]] += k1 * offset + kd * (offset - self.pre_offset)
            elif offset > -0.5 and offset < 0:
                k1 = 2000
                kd = 200
                self.torques[self.left_foot[0]] += k1 * offset + kd * (offset - self.pre_offset)
                self.torques[self.right_foot[0]] += k1 * offset + kd * (offset - self.pre_offset)


        elif self.current_controller == 'fall down':
            p = np.matmul(-self.Kp_0, q - self.state5)
            d = np.matmul(-self.Kd_0, dq)

            self.torques = p + d

            for i in range(6):
                self.torques[i] = 0

        elif self.current_controller == 'walk':

            new_velocity_of_COM = self.skel.getCOMLinearVelocity(simpleFrame, simpleFrame)
            # new_velocity_of_COM = self.skel.getCOMLinearVelocity(pelvisFrame, pelvisFrame)
            # print(new_velocity_of_COM)

            # if(new_velocity_of_COM[0] < 0):
            #     new_velocity_of_COM[0] = 0
            torso_rotation = dart.math.matrixToEulerXYZ(self.torso.getTransform(pelvisFrame).rotation())

            p_torso_x = -self.Kp * (torso_rotation[0] - self.torso_target_angle_wrt_gf_roll)
            d_torso_x = -self.Kd * dq[self.torso_rx]
            torso_x = p_torso_x + d_torso_x

            p_torso_y = -self.Kp * (torso_rotation[1] - self.torso_target_angle_wrt_gf_pitch)
            d_torso_y = -self.Kd * dq[self.torso_ry]
            torso_y = p_torso_y + d_torso_y

            pelvis_com_spatial_velocity = self.pelvis.getCOMSpatialVelocity()
            pelvis_rotation_velocity = pelvis_com_spatial_velocity[0:3]
            p_pelvis_x = -self.Kp_pelvis_torque * (
                    self.pelvis_rotation_euler_XYZ[0] - self.desired_pelvis_target_angle_wrt_gf_x)
            d_pelvis_x = -self.Kd_pelvis_torque * pelvis_rotation_velocity[0]
            self.pelvis_x = p_pelvis_x + d_pelvis_x

            p_pelvis_y = -self.Kp_pelvis_torque * (
                    self.pelvis_rotation_euler_XYZ[1] - self.desired_pelvis_target_angle_wrt_gf_y)
            d_pelvis_y = -self.Kd_pelvis_torque * pelvis_rotation_velocity[1]
            self.pelvis_y = p_pelvis_y + d_pelvis_y

            p_pelvis_z = -self.Kp_turning * (
                    self.pelvis_rotation_euler_XYZ[2] - self.desired_pelvis_target_angle_wrt_gf_z)
            d_pelvis_z = -self.Kd_turning * pelvis_rotation_velocity[2]
            # print("p:", p_pelvis_z)
            # print("d:", d_pelvis_z)

            self.pelvis_z = p_pelvis_z + d_pelvis_z
            # print("rotation euler xyz:", self.pelvis_rotation_euler_XYZ)
            # print(self.pelvis_x, end = " ")
            # print(self.pelvis_y, end = " ")
            # print(self.pelvis_z)

            if self.current_state == 'state1':
                self.elapsed_time += self.timestep

                p = np.matmul(-self.Kp_walk, q - self.state1)
                d = np.matmul(-self.Kd_walk, dq)

                self.torques = p + d

                self.torques[self.torso_ry] = torso_y

                COP = self.right_heel.getTransform(simpleFrame).translation()

                d_x = COM[0] - COP[0]
                v_x = new_velocity_of_COM[0]

                d_y = COM[1] - COP[1]  # +
                v_y = new_velocity_of_COM[1]  # +

                left_thigh_rotation = dart.math.matrixToEulerXYZ(
                    self.left_thigh.getTransform(pelvisFrame, pelvisFrame).rotation())
                swh_target_angle_wrt_gf_13_x_d = self.swh_target_angle_wrt_gf_13_x - self.cd * d_x - self.cv * v_x  # direction x

                swh_target_angle_wrt_gf_13_y_d = self.swh_target_angle_wrt_gf_13_y + self.cd * d_y + self.cv * v_y  # +

                p_left_thigh_y = -self.Kp * (left_thigh_rotation[1] - swh_target_angle_wrt_gf_13_x_d)  # y축을 중심으로의 회전 토크
                d_left_thigh_y = -self.Kd * dq[self.left_thigh_ry]
                left_thigh_y = p_left_thigh_y + d_left_thigh_y
                self.torques[self.left_thigh_ry] = left_thigh_y

                p_left_thigh_x = -self.Kp * (left_thigh_rotation[0] - swh_target_angle_wrt_gf_13_y_d)  # x축을 중심으로의 회전 토크
                d_left_thigh_x = -self.Kd * dq[self.left_thigh_rx]
                left_thigh_x = p_left_thigh_x + d_left_thigh_x
                self.torques[self.left_thigh_rx] = left_thigh_x

                self.torques[self.right_thigh_ry] = (-torso_y - left_thigh_y)

                self.torques[self.right_thigh_rz] = -self.pelvis_z

                if self.elapsed_time >= 0.3:
                    self.current_state = 'state2'
                    self.elapsed_time = 0

            elif self.current_state == 'state2':

                p = np.matmul(-self.Kp_walk, q - self.state2)
                d = np.matmul(-self.Kd_walk, dq)

                self.torques = p + d

                self.torques[self.torso_ry] = torso_y

                COP = self.right_heel.getTransform(simpleFrame).translation()
                d_x = COM[0] - COP[0]
                v_x = new_velocity_of_COM[0]
                d_y = COM[1] - COP[1]  # +
                v_y = new_velocity_of_COM[1]  # +

                left_thigh_rotation = dart.math.matrixToEulerXYZ(
                    self.left_thigh.getTransform(pelvisFrame, pelvisFrame).rotation())

                swh_target_angle_wrt_gf_24_x_d = self.swh_target_angle_wrt_gf_24_x - self.cd * d_x - self.cv * v_x
                # swh_target_angle_wrt_gf_24_x_d = self.swh_target_angle_wrt_gf_24_x
                swh_target_angle_wrt_gf_24_y_d = self.swh_target_angle_wrt_gf_24_y + self.cd * d_y + self.cv * v_y  # +

                p_left_thigh_y = -self.Kp * (left_thigh_rotation[1] - swh_target_angle_wrt_gf_24_x_d)
                d_left_thigh_y = -self.Kd * dq[self.left_thigh_ry]
                left_thigh_y = p_left_thigh_y + d_left_thigh_y
                self.torques[self.left_thigh_ry] = left_thigh_y

                p_left_thigh_x = -self.Kp * (left_thigh_rotation[0] - swh_target_angle_wrt_gf_24_y_d)  # x축을 중심으로의 회전 토크
                d_left_thigh_x = -self.Kd * dq[self.left_thigh_rx]
                left_thigh_x = p_left_thigh_x + d_left_thigh_x
                self.torques[self.left_thigh_rx] = left_thigh_x

                self.torques[self.right_thigh_ry] = (-torso_y - left_thigh_y)

                self.torques[self.right_thigh_rz] = -self.pelvis_z

                if constraint_forces[self.left_foot[0]] != 0:
                    self.current_state = 'state3'

            elif self.current_state == 'state3':
                self.elapsed_time += self.timestep

                p = np.matmul(-self.Kp_walk, q - self.state3)
                d = np.matmul(-self.Kd_walk, dq)

                self.torques = p + d

                self.torques[self.torso_ry] = torso_y

                COP = self.left_heel.getTransform(simpleFrame).translation()
                d_x = COM[0] - COP[0]
                v_x = new_velocity_of_COM[0]
                d_y = COM[1] - COP[1]  # -
                v_y = new_velocity_of_COM[1]  # -
                # print("state 3 d_x:", d_x)
                # print("v_x:", v_x)

                right_thigh_rotation = dart.math.matrixToEulerXYZ(
                    self.right_thigh.getTransform(pelvisFrame, pelvisFrame).rotation())

                swh_target_angle_wrt_gf_13_x_d = self.swh_target_angle_wrt_gf_13_x - self.cd * d_x - self.cv * v_x
                # swh_target_angle_wrt_gf_13_x_d = self.swh_target_angle_wrt_gf_13_x

                swh_target_angle_wrt_gf_13_y_d = self.swh_target_angle_wrt_gf_13_y + self.cd * d_y + self.cv * v_y
                # print("state3:", swh_target_angle_wrt_gf_13_x_d)

                p_right_thigh_y = -self.Kp * (right_thigh_rotation[1] - swh_target_angle_wrt_gf_13_x_d)
                d_right_thigh_y = -self.Kd * dq[self.right_thigh_ry]
                right_thigh_y = p_right_thigh_y + d_right_thigh_y
                self.torques[self.right_thigh_ry] = right_thigh_y

                p_right_thigh_x = -self.Kp * (
                            right_thigh_rotation[0] - swh_target_angle_wrt_gf_13_y_d)  # x축을 중심으로의 회전 토크
                d_right_thigh_x = -self.Kd * dq[self.right_thigh_rx]
                right_thigh_x = p_right_thigh_x + d_right_thigh_x
                self.torques[self.right_thigh_rx] = right_thigh_x

                self.torques[self.left_thigh_ry] = (-torso_y - right_thigh_y)

                self.torques[self.left_thigh_rz] = -self.pelvis_z

                if self.elapsed_time >= 0.3:
                    self.current_state = 'state4'
                    self.elapsed_time = 0

            elif self.current_state == 'state4':

                p = np.matmul(-self.Kp_walk, q - self.state4)
                d = np.matmul(-self.Kd_walk, dq)

                self.torques = p + d

                self.torques[self.torso_ry] = torso_y

                COP = self.left_heel.getTransform(simpleFrame).translation()
                d_x = COM[0] - COP[0]
                v_x = new_velocity_of_COM[0]
                d_y = COM[1] - COP[1]  # -
                v_y = new_velocity_of_COM[1]  # -

                right_thigh_rotation = dart.math.matrixToEulerXYZ(
                    self.right_thigh.getTransform(pelvisFrame, pelvisFrame).rotation())

                swh_target_angle_wrt_gf_24_x_d = self.swh_target_angle_wrt_gf_24_x - self.cd * d_x - self.cv * v_x
                # swh_target_angle_wrt_gf_24_x_d = self.swh_target_angle_wrt_gf_24_x

                swh_target_angle_wrt_gf_24_y_d = self.swh_target_angle_wrt_gf_24_y + self.cd * d_y + self.cv * v_y

                p_right_thigh_y = -self.Kp * (right_thigh_rotation[1] - swh_target_angle_wrt_gf_24_x_d)
                d_right_thigh_y = -self.Kd * dq[self.right_thigh_ry]
                right_thigh_y = p_right_thigh_y + d_right_thigh_y
                self.torques[self.right_thigh_ry] = right_thigh_y

                p_right_thigh_x = -self.Kp * (
                            right_thigh_rotation[0] - swh_target_angle_wrt_gf_24_y_d)  # x축을 중심으로의 회전 토크
                d_right_thigh_x = -self.Kd * dq[self.right_thigh_rx]
                right_thigh_x = p_right_thigh_x + d_right_thigh_x
                self.torques[self.right_thigh_rx] = right_thigh_x

                self.torques[self.left_thigh_ry] = (-torso_y - right_thigh_y)

                self.torques[self.left_thigh_rz] = -self.pelvis_z

                if constraint_forces[self.right_foot[0]] != 0:
                    self.current_state = 'state1'

            self.torques[self.torso_rx] = torso_x

        for i in range(6):
            self.torques[i] = 0

        self.skel.setForces(self.torques)

        if (self.windmill1.getVelocities()[0] > -0.5):
            self.windmill1.setForces([-30])
        if (self.windmill2.getVelocities()[0] > -0.5):
            self.windmill2.setForces([-20])
        if (self.windmill3.getVelocities()[0] > -0.5):
            self.windmill3.setForces([-40])

    def restart(self):
        self.pelvis_transform = self.pelvis.getTransform()
        self.pelvis_translation = self.pelvis_transform.translation()

        distance_from_restart_point1 = np.sqrt((self.pelvis_translation[0] - self.restart_point1[0]) ** 2
                                               + (self.pelvis_translation[1] - self.restart_point1[1]) ** 2
                                               + (self.pelvis_translation[2] - self.restart_point1[2]) ** 2)

        distance_from_restart_point2 = np.sqrt((self.pelvis_translation[0] - self.restart_point2[0]) ** 2
                                               + (self.pelvis_translation[1] - self.restart_point2[1]) ** 2
                                               + (self.pelvis_translation[2] - self.restart_point2[2]) ** 2)

        distance_from_restart_point3 = np.sqrt((self.pelvis_translation[0] - self.restart_point3[0]) ** 2
                                               + (self.pelvis_translation[1] - self.restart_point3[1]) ** 2
                                               + (self.pelvis_translation[2] - self.restart_point3[2]) ** 2)

        distance_from_restart_point4 = np.sqrt((self.pelvis_translation[0] - self.restart_point4[0]) ** 2
                                               + (self.pelvis_translation[1] - self.restart_point4[1]) ** 2
                                               + (self.pelvis_translation[2] - self.restart_point4[2]) ** 2)

        distance_list = [distance_from_restart_point1, distance_from_restart_point2, distance_from_restart_point3,
                         distance_from_restart_point4]
        min_distance_index = distance_list.index(min(distance_list)) + 1
        restart_state = self.state0.copy()
        if (min_distance_index == 1):
            restart_state[0:3] = self.restart_point1_pelvis_orientation
            restart_state[3:6] = self.restart_point1
            self.desired_pelvis_target_angle_wrt_gf_z = self.restart_point1_desired_pelvis_target_angle

        elif (min_distance_index == 2):
            restart_state[0:3] = self.restart_point2_pelvis_orientation
            restart_state[3:6] = self.restart_point2
            self.desired_pelvis_target_angle_wrt_gf_z = self.restart_point2_desired_pelvis_target_angle

        elif (min_distance_index == 3):
            restart_state[0:3] = self.restart_point3_pelvis_orientation
            restart_state[3:6] = self.restart_point3
            self.desired_pelvis_target_angle_wrt_gf_z = self.restart_point3_desired_pelvis_target_angle

        elif (min_distance_index == 4):
            restart_state[0:3] = self.restart_point4_pelvis_orientation
            restart_state[3:6] = self.restart_point4
            self.desired_pelvis_target_angle_wrt_gf_z = self.restart_point4_desired_pelvis_target_angle

        self.skel.setPositions(restart_state)
        self.skel.setVelocities(np.zeros(self.dofs))

        self.current_controller = 'stand'
        self.current_state = 'state0'
        self.elapsed_time = 0
        return True


def main():
    global large_pendulum
    world = dart.simulation.World()

    path = "/./home/sizheng/桌面/final result ver6"

    urdfParser = dart.utils.DartLoader()
    ground = urdfParser.parseSkeleton(path + "/ground.urdf")
    biped = urdfParser.parseSkeleton(path + "/biped.urdf")

    # for i in range(biped.getNumJoints()):
    #     biped.getJoint(i).setLimitEnforcement(True)

    ground1 = urdfParser.parseSkeleton(path + "/ground1.urdf")
    ground2 = urdfParser.parseSkeleton(path + "/ground2.urdf")
    ground3 = urdfParser.parseSkeleton(path + "/ground3.urdf")
    ground4 = urdfParser.parseSkeleton(path + "/ground4.urdf")

    box1 = urdfParser.parseSkeleton(path + "/box1.urdf")
    box2 = urdfParser.parseSkeleton(path + "/box2.urdf")
    box3 = urdfParser.parseSkeleton(path + "/box3.urdf")
    box4 = urdfParser.parseSkeleton(path + "/box4.urdf")
    box5 = urdfParser.parseSkeleton(path + "/box5.urdf")
    box6 = urdfParser.parseSkeleton(path + "/box6.urdf")
    box7 = urdfParser.parseSkeleton(path + "/box7.urdf")

    starting_point_ball = urdfParser.parseSkeleton(path + "/ball1.urdf")
    ending_point_ball = urdfParser.parseSkeleton(path + "/ball2.urdf")

    windmill1 = urdfParser.parseSkeleton(path + "/windmill1.urdf")
    windmill2 = urdfParser.parseSkeleton(path + "/windmill2.urdf")
    windmill3 = urdfParser.parseSkeleton(path + "/windmill3.urdf")

    slope1 = urdfParser.parseSkeleton(path + "/slope1.urdf")
    slope2 = urdfParser.parseSkeleton(path + "/slope2.urdf")
    slope3 = urdfParser.parseSkeleton(path + "/slope3.urdf")
    slope4 = urdfParser.parseSkeleton(path + "/slope4.urdf")

    base = urdfParser.parseSkeleton(path + "/base1.urdf")
    board = urdfParser.parseSkeleton(path + "/board1.urdf")

    large_pendulum = urdfParser.parseSkeleton(path + "/large pendulum.urdf")

    world.addSkeleton(ground)
    world.addSkeleton(biped)
    world.addSkeleton(ground1)
    world.addSkeleton(ground2)
    world.addSkeleton(ground3)
    world.addSkeleton(ground4)

    world.addSkeleton(box1)
    world.addSkeleton(box2)
    world.addSkeleton(box3)
    world.addSkeleton(box4)
    world.addSkeleton(box5)
    world.addSkeleton(box6)
    world.addSkeleton(box7)

    world.addSkeleton(starting_point_ball)
    world.addSkeleton(ending_point_ball)

    world.addSkeleton(windmill1)
    world.addSkeleton(windmill2)
    world.addSkeleton(windmill3)

    world.addSkeleton(slope1)
    world.addSkeleton(slope2)
    world.addSkeleton(slope3)
    world.addSkeleton(slope4)

    world.addSkeleton(base)
    world.addSkeleton(board)

    world.addSkeleton(large_pendulum)

    world.setGravity([0, 0, -9.81])
    world.setTimeStep(0.001)

    dofs = biped.getDofs()
    for i in dofs:
        print(i.getName())

    # dofs1 = box1.getDofs()
    # for i in dofs1:
    #     print(i.getName())

    r = 0.042
    ball = dart.dynamics.EllipsoidShape(math.sqrt(2) * np.ones(3) * r)
    num_bodynode = biped.getNumBodyNodes()
    for i in range(num_bodynode):
        bodynode = biped.getBodyNode(i)
        if bodynode.getName() == "l_pelvis":
            shape_node = bodynode.createShapeNode(ball)
            visual = shape_node.createVisualAspect()
            visual.setColor([0, 1, 0, 0.8])
        else:
            shape_node = bodynode.createShapeNode(ball)
            visual = shape_node.createVisualAspect()
            visual.setColor([200, 0, 0.2])

    node = MyWorldNode(world, biped)

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)

    input_hander = InputHandler(node, viewer)
    viewer.addEventHandler(input_hander)

    grid = dart.gui.osg.GridVisual()
    grid.setOffset([0, 0, -0.5])
    grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.XY)
    grid.setNumCells(600)
    viewer.addAttachment(grid)

    viewer.setUpViewInWindow(960, 0, 1920, 1280)
    viewer.setCameraHomePosition([-10.0, 0.0, 5.0],
                                 [0.00, 0.00, 0.7],
                                 [0, 0, 1])

    viewer.run()


if __name__ == "__main__":
    main()
