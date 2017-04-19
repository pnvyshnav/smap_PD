from collections import namedtuple
from ctypes import *
import _ctypes
import numpy as np
import os, sys

from rllab import spaces
from rllab.envs.base import Env, Step
from rllab.envs.env_spec import EnvSpec
from rllab.misc.overrides import overrides

RAYS = 32
END_TIME = 2000


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


def load(skip_frame=10):
    global lib
    lib = cdll.LoadLibrary('/home/wal/catkin_ws/devel/lib/libgym.so')

    lib.act.argtypes = [c_float, c_float]
    lib.act.restype = c_float

    lib.observe.argtypes = [c_int]
    lib.observe.restype = POINTER(c_float)

    lib.inside.restype = c_bool

    lib.initialize(skip_frame)
    lib.reset()



def isLoaded():
    return lib is not None
    # libp = os.path.abspath(lib)
    # ret = os.system("lsof -p %d | grep %s > /dev/null" % (os.getpid(), libp))
    # return (ret == 0)


class SmapExplore(Env):
    def __init__(self, skip_frame=10):
        if not isLoaded():
            load(skip_frame)
            print("loaded library", lib)
        self.skip_frame = skip_frame
        self.reward = 0.0
        self.last_reward = 0.0
        self.t = 0.0
        self.resets = 0

    @property
    @overrides
    def action_space(self):
        return spaces.Box(np.array([0, -0.1]), np.array([+0.05, +0.1]))

    @property
    @overrides
    def observation_space(self):
        upper = np.ones(RAYS + 1)
        upper[-1] = 1000  # time
        return spaces.Box(np.zeros(RAYS + 1), upper)

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
            self.t += 1.
            self.reward = lib.act(action[0], action[1]) - 10.
        done = self.t >= END_TIME or not lib.inside() or self.reward < -500
        ptr = lib.observe(RAYS)
        # obs = np.array([ptr[i] for i in range(RAYS)])
        # obs = np.frombuffer((c_float * RAYS).from_address(ptr), np.float32).copy()
        # obs = np.ctypeslib.as_array(ptr,shape=(RAYS,))
        # ap = cast(ptr, POINTER(c_float * RAYS))
        obs = make_nd_array(ptr, (RAYS + 1,), np.float32)
        obs[-1] = self.t
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
