from collections import namedtuple
from ctypes import *
import numpy as np
import gym, os
from gym import spaces
from gym.utils import seeding


RAYS = 64
END_TIME = 100


class SmapExplore(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array', 'state_pixels'],
        'video.frames_per_second' : 1000
    }

    def __init__(self):
        self._seed()
        self.reward = 0.0
        self.t = 0.0

        self.action_space = spaces.Box(np.array([-0.05,-0.1]), np.array([+0.05,+0.1]))  # velocity, angular velocity
        self.observation_space = spaces.Box(low=0, high=255, shape=(RAYS, 1, 1))

        self.lib = cdll.LoadLibrary('/home/eric/catkin_ws/build/smap/devel/lib/libgym.so')

        self.lib.act.argtypes = [c_float, c_float]
        self.lib.act.restype = c_float

        self.lib.observe.argtypes = [c_int]
        self.lib.observe.restype = POINTER(c_float)

        self.lib.initialize()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _destroy(self):
        pass

    def _render(self, mode='human', close=False):
        if mode == 'human':
            self.human_render = True
        return [1]

    def _reset(self):
        self._destroy()
        self.reward = 0.0
        self.t = 0.0
        self.lib.reset()

        return self._step(None)[0]

    def _step(self, action):
        step_reward = 0.
        if action is not None:
            self.t += 1.
            step_reward = self.lib.act(action[0], action[1])
            self.reward += step_reward
        done = self.t >= END_TIME
        return self._observe(), step_reward, done, {}

    def _observe(self, rays=RAYS):
        ptr = self.lib.observe(rays)
        self.state = np.array([int(ptr[i]*255.) for i in range(rays)])
        return self.state


if __name__=="__main__":
    os.system("/home/eric/catkin_ws/build/smap/devel/lib/smap/smap")
