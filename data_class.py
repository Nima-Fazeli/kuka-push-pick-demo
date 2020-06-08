import numpy as np
import pandas as pd


class State():
    def __init__(self, N=1):
        self.bx = np.zeros((N, ))
        self.by = np.zeros((N, ))
        self.bt = np.zeros((N, ))
        self.bz = np.zeros((N, ))

        self.lx = np.zeros((N, ))
        self.ly = np.zeros((N, ))
        self.lt = np.zeros((N, ))

        self.rx = np.zeros((N, ))
        self.ry = np.zeros((N, ))

    def update(self, bc, lc, br, time):
        self.bx[time] = bc[0]
        self.by[time] = bc[1]
        self.bt[time] = bc[2]
        self.bz[time] = bc[3]

        self.lx[time] = lc[0]
        self.ly[time] = lc[1]
        self.lt[time] = lc[2]

        self.rx[time] = br[0]
        self.ry[time] = br[1]

    def to_array(self):

        state_array = np.vstack((self.bx,
                                 self.by,
                                 self.bt,
                                 self.bz,
                                 self.lx,
                                 self.ly,
                                 self.lt,
                                 self.rx,
                                 self.ry)).T

        return state_array

    def from_array(self, state_array):
        self.bx = state_array[:, 0]
        self.by = state_array[:, 1]
        self.bt = state_array[:, 2]
        self.bz = state_array[:, 3]

        self.lx = state_array[:, 4]
        self.ly = state_array[:, 5]
        self.lt = state_array[:, 6]

        self.rx = state_array[:, 7]
        self.ry = state_array[:, 8]

    def to_csv(self, file_name='test'):
        state_array = self.to_array()
        pd.DataFrame(state_array).to_csv('data/'+file_name+'.csv', header=None, index=None)

    def from_csv(self, file_name='test'):
        state_array = pd.read_csv('data/'+file_name+'.csv').values
        self.from_array(state_array)
