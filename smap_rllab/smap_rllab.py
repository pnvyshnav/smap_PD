from collections import namedtuple
from ctypes import *
import _ctypes
import numpy as np
import os, sys, math

from rllab import spaces
from rllab.envs.base import Env, Step
from rllab.envs.env_spec import EnvSpec
from rllab.misc.overrides import overrides

RAYS = 32
END_TIME = 2000

# DISCRETE_ACTIONS = [
#     (0.025, 0),
#     (0.05, 0),
#     (0, -.1),
#     (0, .1),
#     (0.025, -.03),
#     (0.025, .03),
# ]

DISCRETE_ACTIONS = [
    (0.05, 0),
    (0, -math.pi/2.),
    (0, math.pi/2.),
]


def make_nd_array(c_pointer, shape, dtype=np.float64, order='C', own_data=True):
    arr_size = np.prod(shape[:]) * np.dtype(dtype).itemsize
    if sys.version_info.major >= 3:
        buf_from_mem = pythonapi.PyMemoryView_FromMemory
        buf_from_mem.restype = py_object
        buf_from_mem.argtypes = (c_void_p, c_int, c_int)
        buffer = buf_from_mem(c_pointer, arr_size, 0x100)
    else:
        buf_from_mem = pythonapi.PyBuffer_FromMemory
        buf_from_mem.restype = py_object
        buffer = buf_from_mem(c_pointer, arr_size)
    arr = np.ndarray(tuple(shape[:]), dtype, buffer, order=order)
    if own_data and not arr.flags.owndata:
        return arr.copy()
    else:
        return arr


lib = None


def load(skip_frame=1):
    global lib
    lib = cdll.LoadLibrary('/home/wal/catkin_ws/devel/lib/libgym.so')

    lib.act.argtypes = [c_float, c_float]
    lib.act.restype = c_float

    lib.observeLocal.argtypes = [c_int]
    lib.observeLocal.restype = POINTER(c_float)

    lib.observeGlobal.restype = POINTER(c_float)
    lib.goalView.restype = POINTER(c_float)

    lib.mapWidth.restype = c_int
    lib.mapHeight.restype = c_int

    lib.inside.restype = c_bool

    lib.initialize(skip_frame)
    lib.reset()


def isLoaded():
    return lib is not None
    # libp = os.path.abspath(lib)
    # ret = os.system("lsof -p %d | grep %s > /dev/null" % (os.getpid(), libp))
    # return (ret == 0)


class SmapExplore(Env):
    def __init__(self, skip_frame=5, global_view=False, discrete_actions=False):
        if not isLoaded():
            load(skip_frame)
            print("loaded library", lib)
        self.skip_frame = skip_frame
        self.reward = 0.0
        self.last_reward = 0.0
        self.t = 0.0
        self.resets = 0
        self.global_view = global_view
        self.map_width = lib.mapWidth()
        self.map_height = lib.mapHeight()
        self.discrete_actions = discrete_actions

    @property
    @overrides
    def action_space(self):
        if self.discrete_actions:
            return spaces.Discrete(len(DISCRETE_ACTIONS))
        return spaces.Box(np.array([0, -0.1]), np.array([+0.05, +0.1]))

    @property
    @overrides
    def observation_space(self):
        if self.global_view:
            return spaces.Box(low=0, high=1, shape=(self.map_width, self.map_height, 2))
        return spaces.Box(np.zeros(RAYS), np.ones(RAYS))

    @property
    def action_bounds(self):
        return self.action_space.bounds

    @overrides
    def render(self, states=None, actions=None, pause=False):
        pass

    @overrides
    def reset(self):
        global lib

        self.reward = 0.0
        self.t = 0.0
        self.resets += 1
        # if self.resets >= 5:
        #     lib.destroy()
        #     handle = lib._handle
        #     del lib
        #     # while isLoaded('/home/eric/catkin_ws/build/smap/devel/lib/libgym.so'):
        #     _ctypes.dlclose(handle)
        #     self.resets = 0
        #     load()
        # else:
        if not isLoaded():
            load(self.skip_frame)
        else:
            lib.reset()

        return self.step(None).observation

    @overrides
    def step(self, action):
        step_reward = 0.
        self.reward = 0.
        self.last_reward = self.reward
        if action is not None:
            if self.discrete_actions:
                action = DISCRETE_ACTIONS[action]
            self.t += 1.
            self.reward = lib.act(action[0], action[1])
        done = self.t >= END_TIME or not lib.inside() or self.reward < -500

        if self.global_view:
            ptr = lib.observeGlobal()
            obs = make_nd_array(ptr, (self.map_width, self.map_height), np.float32)
            # encode current position in extra channel (one-hot encoding)
            pos = np.zeros((self.map_width, self.map_height))
            pos[int(self.map_width/2), int(self.map_height/2)] = 1
            obs = np.asarray([obs, pos])
        else:
            ptr = lib.observeLocal(RAYS)
            obs = make_nd_array(ptr, (RAYS,), np.float32)
        # obs = np.append(obs, [self.t])
        # print(obs)
        # print(self.reward)
        return Step(observation=obs, reward=self.reward, done=done)

    def env_spec(self):
        return EnvSpec(observation_space=self.observation_space,
                       action_space=self.action_space)


# def env_spec():
#     se = SmapExplore()
#     return EnvSpec(observation_space=se.observation_space,
#                    action_space=se.action_space)

if __name__ == "__main__":
    os.system("/home/wal/catkin_ws/devel/lib/smap/smap")
