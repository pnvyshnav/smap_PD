import sys
# Add the ptdraft folder path to the sys.path list
sys.path.append('/home/eric/dev/rllab/')

from rllab.algos.trpo import TRPO
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from smap_rllab import SmapExplore, env_spec


def run_task(*_):
    env = SmapExplore()
    env = normalize(env)
    # env = normalize(GymEnv("Pendulum-v0"))

    policy = GaussianMLPPolicy(
        env_spec=env_spec,
        # The neural network policy should have two hidden layers, each with 4 hidden units.
        hidden_sizes=(32, 32, 16)
    )

    baseline = LinearFeatureBaseline(env_spec=env_spec)

    algo = TRPO(
        env=env,
        policy=policy,
        baseline=baseline,
        batch_size=4000,
        whole_paths=True,
        max_path_length=600,
        n_itr=1000,
        discount=0.99,
        step_size=0.01,
        # Uncomment both lines (this and the plot parameter below) to enable plotting
        plot=True,
    )
    algo.train()


run_experiment_lite(
    run_task,
    # Number of parallel workers for sampling
    n_parallel=3,
    # Only keep the snapshot parameters for the last iteration
    snapshot_mode="last",
    # Specifies the seed for the experiment. If this is not provided, a random seed
    # will be used
    seed=1,
    plot=True,
)
