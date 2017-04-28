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


class GaussianConvPolicy(StochasticPolicy, LasagnePowered):
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

        self._l_prob = std_network.output_layer
        self._l_obs = std_network.input_layer
        self._f_dist = ext.compile_function(
            inputs=[obs_var],
            outputs=[mean_var, log_std_var],
        )

        self._dist = DiagonalGaussian(action_dim)

        super(GaussianConvPolicy, self).__init__(env_spec)
        LasagnePowered.__init__(self, [std_network.output_layer])

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

    @property
    def distribution(self):
        return self._dist