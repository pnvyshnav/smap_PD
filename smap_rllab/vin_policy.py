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
import theano.tensor as TT

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
        self.v = T.max(self.q, axis=1, keepdims=True)

        for i in range(0, k-1):
            self.q = conv2D_keep_shape(T.concatenate([self.r, self.v], axis=1), T.concatenate([self.w, self.w_fb],
                                                                                              axis=1),
                                       image_shape=[batchsize, self.w1.shape.eval()[0]+1, imsize[0], imsize[1]],
                                       filter_shape=T.concatenate([self.w, self.w_fb], axis=1).shape.eval())
            self.v = T.max(self.q, axis=1, keepdims=True)

        # do one last convolution
        self.q = conv2D_keep_shape(T.concatenate([self.r, self.v], axis=1), T.concatenate([self.w, self.w_fb], axis=1),
                                   image_shape=[batchsize, self.w1.shape.eval()[0]+1, imsize[0], imsize[1]],
                                   filter_shape=T.concatenate([self.w, self.w_fb], axis=1).shape.eval())

        # Select the conv-net channels at the state position (S1,S2).
        # This intuitively corresponds to each channel representing an action, and the convnet the Q function.
        # The tricky thing is we want to select the same (S1,S2) position *for each* channel and for each sample
        self.q_out = self.q[T.extra_ops.repeat(T.arange(self.q.shape[0]), state_batch_size), :, in_s1.flatten(),
                     in_s2.flatten()]

        # softmax output weights
        self.w_o = init_weights_T(l_q, 8)  # 80 parameters
        self.output = T.nnet.softmax(T.dot(self.q_out, self.w_o))

        self.params = [self.w0, self.bias, self.w1, self.w, self.w_fb, self.w_o]