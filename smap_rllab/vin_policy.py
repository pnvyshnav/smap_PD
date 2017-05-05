import lasagne
from rllab.core.lasagne_layers import ParamLayer
from rllab.core.lasagne_powered import LasagnePowered
import lasagne.layers as L
from rllab.core.network import ConvNetwork
from rllab.distributions.diagonal_gaussian import DiagonalGaussian
from rllab.policies.base import StochasticPolicy
from rllab.misc import tensor_utils
from rllab.spaces.discrete import Discrete
from rllab.spaces import Box

from rllab.core.serializable import Serializable
from rllab.misc import ext
from rllab.misc import logger
from rllab.misc.overrides import overrides
import numpy as np
import lasagne.nonlinearities as NL
import theano
import theano.tensor as TT

import pickle
import scipy.io as sio
import time

from .theano_utils import *


# Value Iteration Network (VIN) Policy
class VinPolicy(StochasticPolicy, LasagnePowered):
    def __init__(
            self,
            name,
            env_spec,
            conv_filters,
            conv_filter_sizes,
            conv_strides,
            conv_pads,
            hidden_sizes=(32, 32),
            learn_std=True,
            init_std=1.0,
            min_std=1e-6,
            hidden_nonlinearity=NL.rectify,
            output_nonlinearity=None,
            mean_network=None,
            std_network=None,
            std_hidden_nonlinearity=NL.rectify,
            std_hidden_sizes=(32, 32),
            adaptive_std=False,
    ):
        """
        :param env_spec: A spec for the mdp.
        :param hidden_sizes: list of sizes for the fully connected hidden layers
        :param hidden_nonlinearity: nonlinearity used for each hidden layer
        :param std_network: manually specified network for this policy, other network params
        are ignored
        :return:
        """
        Serializable.quick_init(self, locals())
        assert isinstance(env_spec.action_space, Box)
        # assert isinstance(env_spec.action_space, Discrete)

        self._env_spec = env_spec
        self.min_std = min_std

        action_dim = env_spec.action_space.flat_dim

        # create network
        if mean_network is None:
            mean_network = ConvNetwork(
                input_shape=env_spec.observation_space.shape,
                output_dim=action_dim,
                conv_filters=conv_filters,
                conv_filter_sizes=conv_filter_sizes,
                conv_strides=conv_strides,
                conv_pads=conv_pads,
                hidden_sizes=hidden_sizes,
                hidden_nonlinearity=hidden_nonlinearity,
                output_nonlinearity=output_nonlinearity,
                name="mean_network",
            )
        self._mean_network = mean_network

        l_mean = mean_network.output_layer
        obs_var = mean_network.input_layer.input_var

        if std_network is None:
            std_network = ConvNetwork(
                input_shape=env_spec.observation_space.shape,
                output_dim=action_dim,
                conv_filters=conv_filters,
                conv_filter_sizes=conv_filter_sizes,
                conv_strides=conv_strides,
                conv_pads=conv_pads,
                hidden_sizes=std_hidden_sizes,
                hidden_nonlinearity=std_hidden_nonlinearity,
                output_nonlinearity=None,
                name="std_network",
            )
            if adaptive_std:
                l_log_std = std_network.output_layer
            else:
                l_log_std = ParamLayer(
                    mean_network.input_layer,
                    num_units=action_dim,
                    param=lasagne.init.Constant(np.log(init_std)),
                    name="output_log_std",
                    trainable=learn_std,
                )
        else:
            l_log_std = std_network.output_layer

        mean_var, log_std_var = L.get_output([l_mean, l_log_std])

        if self.min_std is not None:
            log_std_var = TT.maximum(log_std_var, np.log(min_std))

        self._mean_var, self._log_std_var = mean_var, log_std_var

        self._l_mean = l_mean
        self._l_log_std = l_log_std

        # self._l_prob = std_network.output_layer
        # self._l_obs = std_network.input_layer

        LasagnePowered.__init__(self, [l_mean, l_log_std])
        super(GaussianConvPolicy, self).__init__(env_spec)

        self._f_dist = ext.compile_function(
            inputs=[obs_var],
            outputs=[mean_var, log_std_var],
        )

        self._dist = DiagonalGaussian(action_dim)

    @property
    def vectorized(self):
        return True

    # @overrides
    # def dist_info_sym(self, obs_var, state_info_vars=None):
    #     return dict(
    #         prob=L.get_output(
    #             self._l_prob,
    #             {self._l_obs: obs_var}
    #         )
    #     )
    @overrides
    def dist_info_sym(self, obs_var, state_info_vars=None):
        mean_var, log_std_var = L.get_output([self._l_mean, self._l_log_std], obs_var)
        if self.min_std is not None:
            log_std_var = TT.maximum(log_std_var, np.log(self.min_std))
        return dict(mean=mean_var, log_std=log_std_var)

    @overrides
    def dist_info(self, obs, state_infos=None):
        return dict(prob=self._f_dist(obs))

    # The return value is a pair. The first item is a matrix (N, A), where each
    # entry corresponds to the action value taken. The second item is a vector
    # of length N, where each entry is the density value for that action, under
    # the current policy
    # @overrides
    # def get_action(self, observation):
    #     flat_obs = self.observation_space.flatten(observation)
    #     prob = self._f_dist([flat_obs])[0]
    #     action = self.action_space.weighted_sample(prob)
    #     return action, dict(prob=prob)
    #
    # def get_actions(self, observations):
    #     flat_obs = self.observation_space.flatten_n(observations)
    #     probs = self._f_dist(flat_obs)
    #     actions = list(map(self.action_space.weighted_sample, probs))
    #     return actions, dict(prob=probs)

    @overrides
    def get_action(self, observation):
        flat_obs = self.observation_space.flatten(observation)
        mean, log_std = [x[0] for x in self._f_dist([flat_obs])]
        rnd = np.random.normal(size=mean.shape)
        action = rnd * np.exp(log_std) + mean
        return action, dict(mean=mean, log_std=log_std)

    def get_actions(self, observations):
        flat_obs = self.observation_space.flatten_n(observations)
        means, log_stds = self._f_dist(flat_obs)
        rnd = np.random.normal(size=means.shape)
        actions = rnd * np.exp(log_stds) + means
        return actions, dict(mean=means, log_std=log_stds)

    def get_reparam_action_sym(self, obs_var, action_var, old_dist_info_vars):
        """
        Given observations, old actions, and distribution of old actions, return a symbolically reparameterized
        representation of the actions in terms of the policy parameters
        :param obs_var:
        :param action_var:
        :param old_dist_info_vars:
        :return:
        """
        new_dist_info_vars = self.dist_info_sym(obs_var, action_var)
        new_mean_var, new_log_std_var = new_dist_info_vars["mean"], new_dist_info_vars["log_std"]
        old_mean_var, old_log_std_var = old_dist_info_vars["mean"], old_dist_info_vars["log_std"]
        epsilon_var = (action_var - old_mean_var) / (TT.exp(old_log_std_var) + 1e-8)
        new_action_var = new_mean_var + epsilon_var * TT.exp(new_log_std_var)
        return new_action_var

    @property
    def distribution(self):
        return self._dist


class NNobj:
    "Class for a multi-layer perceptron object"

    def __init__(self):
        pass

    def save_weights(self, outfile="weight_dump.pk"):
        pickle.dump([n.op.get_value() for n in self.params], open(outfile, 'w'))

    def load_weights(self, infile="weight_dump.pk"):
        dump = pickle.load(open(infile, 'r'))
        [n.op.set_value(p) for n, p in zip(self.params, dump)]


# helper methods to print nice table (taken from CGT code)
def fmt_item(x, l):
    if isinstance(x, np.ndarray):
        assert x.ndim == 0
        x = x.item()
    if isinstance(x, float):
        rep = "%g" % x
    else:
        rep = str(x)
    return " " * (l - len(rep)) + rep


def fmt_row(width, row, header=False):
    out = " | ".join(fmt_item(x, width) for x in row)
    if header: out = out + "\n" + "-" * len(out)
    return out


class vin(NNobj):
    "Class for a neural network that does k iterations of value iteration"

    def __init__(self, model="VIN", im_size=[28, 28], dropout=False, devtype="cpu", grad_check=False, reg=0, k=10,
                 statebatchsize=10, batchsize=128):
        super().__init__()
        self.im_size = im_size  # input image size
        self.model = model
        self.reg = reg  # regularization (currently not implemented)
        self.k = k  # number of VI iterations
        self.batchsize = batchsize  # batch size for training
        self.statebatchsize = statebatchsize  # number of state inputs for every image input, since each image is the
        # same for many states in the data
        np.random.seed(0)
        print(model)
        theano.config.blas.ldflags = "-L/usr/local/lib -lopenblas"

        # X input : l=2 stacked images: obstacle map and reward function prior
        self.X = T.ftensor4(name="X")
        # S1,S2 input : state position (vertical and horizontal position)
        self.S1 = T.bmatrix("S1")  # state first dimension * statebatchsize
        self.S2 = T.bmatrix("S2")  # state second dimension * statebatchsize
        self.y = T.bvector("y")  # output action * statebatchsize

        l = 2  # channels in input layer
        l_h = 150  # channels in initial hidden layer
        l_q = 10  # channels in q layer (~actions)

        self.vin_net = VinBlock(in_x=self.X, in_s1=self.S1, in_s2=self.S2, in_x_channels=l,
                                imsize=self.im_size, batchsize=self.batchsize,
                                state_batch_size=self.statebatchsize,
                                l_h=l_h, l_q=l_q,
                                k=self.k)
        self.p_of_y = self.vin_net.output
        self.params = self.vin_net.params
        # Total 1910 parameters

        self.cost = -T.mean(T.log(self.p_of_y)[T.arange(self.y.shape[0]),
                                               self.y], dtype=theano.config.floatX)
        self.y_pred = T.argmax(self.p_of_y, axis=1)
        self.err = T.mean(T.neq(self.y_pred, self.y.flatten()), dtype=theano.config.floatX)

        self.computeloss = theano.function(inputs=[self.X, self.S1, self.S2, self.y],
                                           outputs=[self.err, self.cost])
        self.y_out = theano.function(inputs=[self.X, self.S1, self.S2], outputs=[self.y_pred])

    def run_training(self, input, stepsize=0.01, epochs=10, output='None', batch_size=128, grad_check=True,
                     profile=False, data_fraction=1):
        # run training from input matlab data file, and save test data prediction in output file
        # load data from Matlab file, including
        # im_data: flattened images
        # state_data: concatenated one-hot vectors for each state variable
        # state_xy_data: state variable (x,y position)
        # label_data: one-hot vector for action (state difference)
        matlab_data = sio.loadmat(input)
        im_data = matlab_data["batch_im_data"]
        im_data = (im_data - 1) / 255  # obstacles = 1, free zone = 0
        value_data = matlab_data["batch_value_data"]
        state1_data = matlab_data["state_x_data"]
        state2_data = matlab_data["state_y_data"]
        label_data = matlab_data["batch_label_data"]
        ydata = label_data.astype('int8')
        Xim_data = im_data.astype(theano.config.floatX)
        Xim_data = Xim_data.reshape(-1, 1, self.im_size[0], self.im_size[1])
        Xval_data = value_data.astype(theano.config.floatX)
        Xval_data = Xval_data.reshape(-1, 1, self.im_size[0], self.im_size[1])
        Xdata = np.append(Xim_data, Xval_data, axis=1)
        S1data = state1_data.astype('int8')
        S2data = state2_data.astype('int8')

        all_training_samples = int(6 / 7.0 * Xdata.shape[0])
        training_samples = int(data_fraction * all_training_samples)
        Xtrain = Xdata[0:training_samples]
        S1train = S1data[0:training_samples]
        S2train = S2data[0:training_samples]
        ytrain = ydata[0:training_samples]

        Xtest = Xdata[all_training_samples:]
        S1test = S1data[all_training_samples:]
        S2test = S2data[all_training_samples:]
        ytest = ydata[all_training_samples:]
        ytest = ytest.flatten()

        sortinds = np.random.permutation(training_samples)
        Xtrain = Xtrain[sortinds]
        S1train = S1train[sortinds]
        S2train = S2train[sortinds]
        ytrain = ytrain[sortinds]
        ytrain = ytrain.flatten()

        self.updates = rmsprop_updates_T(self.cost, self.params, stepsize=stepsize)
        self.train = theano.function(inputs=[self.X, self.S1, self.S2, self.y], outputs=[], updates=self.updates)

        print(fmt_row(10, ["Epoch", "Train NLL", "Train Err", "Test NLL", "Test Err", "Epoch Time"]))
        for i_epoch in range(int(epochs)):
            tstart = time.time()
            # do training
            for start in range(0, Xtrain.shape[0], batch_size):
                end = start + batch_size
                if end <= Xtrain.shape[0]:
                    self.train(Xtrain[start:end], S1train[start:end], S2train[start:end],
                               ytrain[start * self.statebatchsize:end * self.statebatchsize])
            elapsed = time.time() - tstart
            # compute losses
            trainerr = 0.
            trainloss = 0.
            testerr = 0.
            testloss = 0.
            num = 0
            for start in range(0, Xtest.shape[0], batch_size):
                end = start + batch_size
                if end <= Xtest.shape[0]:
                    num += 1
                    trainerr_, trainloss_ = self.computeloss(Xtrain[start:end], S1train[start:end], S2train[start:end],
                                                             ytrain[
                                                             start * self.statebatchsize:end * self.statebatchsize])
                    testerr_, testloss_ = self.computeloss(Xtest[start:end], S1test[start:end], S2test[start:end],
                                                           ytest[start * self.statebatchsize:end * self.statebatchsize])
                    trainerr += trainerr_
                    trainloss += trainloss_
                    testerr += testerr_
                    testloss += testloss_
            print(fmt_row(10, [i_epoch, trainloss / num, trainerr / num, testloss / num, testerr / num, elapsed]))

    def predict(self, input):
        # NN output for a single input, read from file
        matlab_data = sio.loadmat(input)
        im_data = matlab_data["im_data"]
        im_data = (im_data - 1) / 255  # obstacles = 1, free zone = 0
        state_data = matlab_data["state_xy_data"]
        value_data = matlab_data["value_data"]
        xim_test = im_data.astype(theano.config.floatX)
        xim_test = xim_test.reshape(-1, 1, self.im_size[0], self.im_size[1])
        xval_test = value_data.astype(theano.config.floatX)
        xval_test = xval_test.reshape(-1, 1, self.im_size[0], self.im_size[1])
        x_test = np.append(xim_test, xval_test, axis=1)
        s_test = state_data.astype('int8')
        s1_test = s_test[:, 0].reshape([1, 1])
        s2_test = s_test[:, 1].reshape([1, 1])
        out = self.y_out(x_test, s1_test, s2_test)
        return out[0][0]

    def predict_value(self, input):
        # Value and reward for a single input, read from file
        val_pred = theano.function(inputs=[self.X], outputs=[self.vin_net.v])
        r_pred = theano.function(inputs=[self.X], outputs=[self.vin_net.r])
        matlab_data = sio.loadmat(input)
        im_data = matlab_data["im_data"]
        im_data = (im_data - 1) / 255  # obstacles = 1, free zone = 0
        value_data = matlab_data["value_data"]
        xim_test = im_data.astype(theano.config.floatX)
        xim_test = xim_test.reshape(-1, 1, self.im_size[0], self.im_size[1])
        xval_test = value_data.astype(theano.config.floatX)
        xval_test = xval_test.reshape(-1, 1, self.im_size[0], self.im_size[1])
        x_test = np.append(xim_test, xval_test, axis=1)
        out_v = val_pred(x_test)
        out_r = r_pred(x_test)
        return [out_v[0][0], out_r[0][0]]

    def load_weights(self, infile="weight_dump.pk"):
        dump = pickle.load(open(infile, 'r'))
        [n.set_value(p) for n, p in zip(self.params, dump)]

    def save_weights(self, outfile="weight_dump.pk"):
        pickle.dump([n.get_value() for n in self.params], open(outfile, 'w'))


class VinBlock(object):
    """VIN block"""

    def __init__(self, in_x, in_s1, in_s2, in_x_channels, imsize, batchsize=128,
                 state_batch_size=1, l_h=150, l_q=10, k=0):
        """
        Allocate a VIN block with shared variable internal parameters.

        :type in_x: theano.tensor.dtensor4
        :param in_x: symbolic input image tensor, of shape [batchsize, in_x_channels, imsize[0], imsize[1]]
        Typically : first channel is image, second is the reward prior.

        :type in_s1: theano.tensor.bmatrix
        :param in_s1: symbolic input batches of vertical positions, of shape [batchsize, state_batch_size]

        :type in_s2: theano.tensor.bmatrix
        :param in_s2: symbolic input batches of horizontal positions, of shape [batchsize, state_batch_size]

        :type in_x_channels: int32
        :param in_x_channels: number of input channels

        :type imsize: tuple or list of length 2
        :param imsize: (image height, image width)

        :type batchsize: int32
        :param batchsize: batch size

        :type state_batch_size: int32
        :param state_batch_size: number of state inputs for each sample

        :type l_h: int32
        :param l_h: number of channels in first hidden layer

        :type l_q: int32
        :param l_q: number of channels in q layer (~actions)

        :type k: int32
        :param k: number of VI iterations (actually, real number of iterations is k+1)

        """
        self.bias = theano.shared((np.random.randn(l_h) * 0.01).astype(theano.config.floatX))  # 150 parameters
        self.w0 = init_weights_T(l_h, in_x_channels, 3, 3)  # 1350 parameters
        # initial conv layer over image+reward prior
        self.h = conv2D_keep_shape(in_x, self.w0, image_shape=[batchsize, self.w0.shape.eval()[1],
                                                               imsize[0], imsize[1]],
                                   filter_shape=self.w0.shape.eval())
        self.h = self.h + self.bias.dimshuffle('x', 0, 'x', 'x')

        self.w1 = init_weights_T(1, l_h, 1, 1)  # 150 parameters
        self.r = conv2D_keep_shape(self.h, self.w1, image_shape=[batchsize, self.w0.shape.eval()[0],
                                                                 imsize[0], imsize[1]],
                                   filter_shape=self.w1.shape.eval())

        # weights from inputs to q layer (~reward in Bellman equation)
        self.w = init_weights_T(l_q, 1, 3, 3)  # 90 parameters
        # feedback weights from v layer into q layer (~transition probabilities in Bellman equation)
        self.w_fb = init_weights_T(l_q, 1, 3, 3)  # 90 parameters

        self.q = conv2D_keep_shape(self.r, self.w, image_shape=[batchsize, self.w1.shape.eval()[0],
                                                                imsize[0], imsize[1]],
                                   filter_shape=self.w.shape.eval())
        self.v = TT.max(self.q, axis=1, keepdims=True)

        for i in range(0, k - 1):
            self.q = conv2D_keep_shape(TT.concatenate([self.r, self.v], axis=1), TT.concatenate([self.w, self.w_fb],
                                                                                                axis=1),
                                       image_shape=[batchsize, self.w1.shape.eval()[0] + 1, imsize[0], imsize[1]],
                                       filter_shape=TT.concatenate([self.w, self.w_fb], axis=1).shape.eval())
            self.v = T.max(self.q, axis=1, keepdims=True)

        # do one last convolution
        self.q = conv2D_keep_shape(TT.concatenate([self.r, self.v], axis=1),
                                   TT.concatenate([self.w, self.w_fb], axis=1),
                                   image_shape=[batchsize, self.w1.shape.eval()[0] + 1, imsize[0], imsize[1]],
                                   filter_shape=TT.concatenate([self.w, self.w_fb], axis=1).shape.eval())

        # Select the conv-net channels at the state position (S1,S2).
        # This intuitively corresponds to each channel representing an action, and the convnet the Q function.
        # The tricky thing is we want to select the same (S1,S2) position *for each* channel and for each sample
        self.q_out = self.q[TT.extra_ops.repeat(TT.arange(self.q.shape[0]), state_batch_size), :, in_s1.flatten(),
                     in_s2.flatten()]

        # softmax output weights
        self.w_o = init_weights_T(l_q, 8)  # 80 parameters
        self.output = TT.nnet.softmax(TT.dot(self.q_out, self.w_o))

        self.params = [self.w0, self.bias, self.w1, self.w, self.w_fb, self.w_o]
