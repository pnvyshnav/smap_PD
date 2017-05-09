import sys

# Add the ptdraft folder path to the sys.path list
from rllab.algos.npo import NPO
from rllab.baselines.gaussian_conv_baseline import GaussianConvBaseline

sys.path.append('/home/eric/dev/rllab/')

from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.policies.categorical_conv_policy import CategoricalConvPolicy
from smap_rllab import SmapExplore
from cnn_policy import GaussianConvPolicy
from categorical_conv_baseline import CategoricalConvBaseline


def run_task(*_):
    env = SmapExplore(skip_frame=100, global_view=False,
                      discrete_actions=False, debug=False,
                      holonomic_actions=True)
    env_spec = env.env_spec()
    env = normalize(env)
    # env = normalize(GymEnv("Pendulum-v0"))

    policy = GaussianMLPPolicy(
        env_spec=env_spec,
        # The neural network policy should have two hidden layers, each with 4 hidden units.
        hidden_sizes=(32, 32, 16)
    )
    baseline = LinearFeatureBaseline(env_spec=env_spec)

    # policy = GaussianConvPolicy(
    #     name="CNN_Policy",
    #     env_spec=env_spec,
    #     # The neural network policy should have two hidden layers, each with 4 hidden units.
    #     hidden_sizes=[200, 100, 50],
    #     conv_filters=[150, 150, 150],
    #     conv_filter_sizes=[(2,2), (2,2), (2,2)],
    #     conv_strides=[1, 2, 2],
    #     conv_pads=[0] #['same']
    # )

    # policy = CategoricalConvPolicy(
    #     name="CNN_Policy",
    #     env_spec=env_spec,
    #     # The neural network policy should have two hidden layers, each with 4 hidden units.
    #     hidden_sizes=[32, 16],
    #     conv_filters=[32, 32, 32],
    #     conv_filter_sizes=[(3,3), (3,3), (3,3)],
    #     conv_strides=[1, 2, 2],
    #     conv_pads=['same', 'same', 'same'] #['same']
    # )

    # regressor_args GaussianConvPolicy = {
    #     'hidden_sizes': [32, 32],
    #     'conv_filters': [2],
    #     'conv_filter_sizes': [(3, 3)],
    #     'conv_strides': [1, 2],
    #     'conv_pads': ['same']
    # }
    # policy = GaussianConvBaseline(
    #     env_spec=env_spec,
    #     regressor_args=regressor_args
    # )

# TODO this was the right baseline
    # regressor_args = dict(
    #     hidden_sizes=[16, 16],
    #     conv_filters=[16, 16],
    #     conv_filter_sizes=[(3,3), (3,3)],
    #     conv_strides=[1, 2],
    #     conv_pads=['same', 'same']
    # )
    # baseline = CategoricalConvBaseline(
    #     env_spec=env_spec,
    #     regressor_args=regressor_args
    # )

    algo = TRPO(
        env=env,
        policy=policy,
        baseline=baseline,
        batch_size=4000,
        whole_paths=True,
        max_path_length=600,
        n_itr=5000,
        discount=0.99,
        step_size=0.01,
        # Uncomment both lines (this and the plot parameter below) to enable plotting
        plot=True,
    )
    algo.train()


def main():
    run_experiment_lite(
        run_task,
        # Number of parallel workers for sampling
        n_parallel=1,
        # Only keep the snapshot parameters for the last iteration
        snapshot_mode="last",
        # Specifies the seed for the experiment. If this is not provided, a random seed
        # will be used
        seed=1,
        plot=True,
    )


if __name__ == "__main__":
    main()
    # run_task()
