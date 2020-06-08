import pybullet as p
import pybullet_data
import time
import math
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
import pdb
from tower import Tower
import controller
import pandas as pd
from data_class import State as s
from robot import Kuka
import matplotlib.pyplot as plt
from numpy.matlib import repmat


class World():
    def __init__(self, withVis=True):
        # connect to pybullet server
        if withVis:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        # set additional path to find kuka model
        print(pybullet_data.getDataPath())

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # set camera
        p.resetDebugVisualizerCamera(cameraDistance=1,
                                     cameraYaw=-60,
                                     cameraPitch=-20,
                                     cameraTargetPosition=[0,0,0])

        # add plane to push on (slightly below the base of the robot)
        self.planeId = p.loadURDF("plane.urdf", [0, 0, -0.021], useFixedBase=True)
        p.changeDynamics(self.planeId, -1, lateralFriction=0.99)
        # add robot
        self.robot = Kuka()

        # add the tower
        # num_block=27
        # self.tower = Tower(num_block, frame_origin=np.array([0, 0, 0]))
        # self.add_tower()
        #
        # # add controller
        # self.controller = controller.SimpleController()
        # self.stabilizer = controller.PrePush()

        # set gravity
        p.setGravity(0, 0, -10)

        # set simulation length
        # self.simLength = 1000 # 4000
        # self.state = s(self.simLength)
        #
        # # pre-define the trajectory/force vectors
        # self.traj = np.zeros((self.simLength, 5))
        # self.contactForce = np.zeros((self.simLength, ))
        # self.contactCount = np.empty_like(self.contactForce)
        #
        # # reset sim time
        # self.t = 0

        # for testing
        self.block_id = p.loadURDF("block.urdf", [0, 0, 0.05])
        p.changeDynamics(self.block_id, -1, mass=0.8, lateralFriction=0.89)
        p.resetBasePositionAndOrientation(self.block_id, [0, 0.0, 0], [0, 0, 0, 1])


    def pick(self):
        # x, y, z, ori, fingerAngle

        x0 = 0.0
        y0 = 0.07
        yf = 0.1
        finger_init = 0.08
        # pre-push pose
        command_pre = [[x0, y0, 0.50, 0.0, finger_init],
                       [x0, y0, 0.45, 0.0, finger_init],
                       [x0, y0, 0.40, 0.0, finger_init],
                       [x0, y0, 0.35, 0.0, finger_init],
                       [x0, y0, 0.30, 0.0, finger_init],
                       [x0, y0, 0.27, 0.0, finger_init]]

        big_steps = 100
        phase_length = big_steps * len(command_pre)

        for t in range(phase_length):
            ind = t // big_steps
            command_now = command_pre[ind]

            self.robot.applyAction(command_now, with_top=True)
            p.stepSimulation()

        # push phase
        sim_length = 500
        joint_reaction = np.zeros((sim_length, 6))
        block_pos = np.zeros((sim_length, 3))
        block_ori = np.zeros((sim_length, 3))
        block_for = np.zeros((sim_length, 3))

        y = - np.linspace(- y0, yf, sim_length)

        start_pos = np.array([x0, yf, 0.27, 0.0, finger_init])
        command_push = repmat(start_pos, sim_length, 1)
        command_push[:, 1] = y

        for t in range(sim_length):
            command_now = command_push[t]

            self.robot.applyAction(command_now, with_top=True)
            p.stepSimulation()

            joint_state = p.getJointState(self.robot.kukaUid,
                                          self.robot.kukaEndEffectorIndex)
            joint_reaction[t, :] = joint_state[2]

            block_pose = p.getBasePositionAndOrientation(self.block_id)
            block_pos[t, :] = block_pose[0]
            block_ori[t, :] = p.getEulerFromQuaternion(block_pose[1])

            ct_block_floor = p.getContactPoints(self.block_id, self.planeId)

            n_c = len(ct_block_floor)
            f_total = np.zeros((3, ))


            for pt in range(n_c):
                normal = np.array(ct_block_floor[pt][7]) * ct_block_floor[pt][9]
                tangent_1 = np.array(ct_block_floor[pt][11]) * ct_block_floor[pt][10]
                tangent_2 = np.array(ct_block_floor[pt][13]) * ct_block_floor[pt][12]

                print(tangent_1)
                # pdb.set_trace()
                # if n_c > 4:
                #     pdb.set_trace()
                f_total += normal + tangent_1 + tangent_2

            block_for[t, :] = f_total


        command_post = [[x0, -0.10, 0.27, 0.0, 0.5],
                        [x0, -0.20, 0.27, 0.0, 0.5],
                        [x0, -0.20, 0.27, 0.0, 0.0],
                        [x0, -0.15, 0.30, 0.0, 0.0],
                        [x0, -0.10, 0.35, 0.0, 0.0],
                        [x0, -0.05, 0.40, 0.0, 0.0],
                        [x0, 0.00, 0.45, 0.0, 0.0],
                        [x0, 0.00, 0.50, 0.0, 0.0]]

        sim_length = len(command_post) * big_steps

        for t in range(sim_length):
            ind = t // big_steps
            command_now = command_post[ind]

            self.robot.applyAction(command_now, with_top=True)
            p.stepSimulation()


        fig, ax = plt.subplots(1, 2)
        ax[0].plot(block_pos[:, 0], label='x')
        ax[0].plot(block_pos[:, 1], label='y')
        ax[1].plot(block_ori[:, 0], label='rx')
        ax[1].plot(block_ori[:, 1], label='ry')
        ax[1].plot(block_ori[:, 2], label='rz')
        ax[0].legend()
        ax[1].legend()

        fig, ax = plt.subplots()
        # ax.plot(joint_reaction[:, 0], label='Joint fx')
        # ax.plot(joint_reaction[:, 1], label='Joint fy')
        N = 10
        # fx = np.convolve(block_for[:, 0], np.ones((N,))/N, mode='valid')
        # fy = np.convolve(block_for[:, 1], np.ones((N,))/N, mode='valid')
        ax.plot(block_for[:, 0], label='block fx')
        ax.plot(block_for[:, 1], label='block fy')
        # ax.plot(block_for[:, 2], label='block fz')
        ax.legend()
        plt.show()

if __name__=="__main__":
    env = World()

    env.pick()

    # input('Simulation done! Hit Enter ... [ENTER] ')
