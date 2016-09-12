from __future__ import absolute_import
import sys, pickle, os

from rllab.algos.trpo import TRPO
from rllab.algos.vpg import VPG
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.baselines.zero_baseline import ZeroBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy
from os.path import dirname
sys.path.append(dirname(dirname(dirname(os.getcwd()))))

from pin_env_discrete import *
import numpy as np
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

# stub(globals())

class PolicyGenerator:

    """
    Class that trains a tensioning policy given a config file and a file to dump the policy to.
    """

    def __init__(self, experiment_folder="", config_file="experiment.json", writefile="policydiscrete.p", datafile="data.p", iterations=30, gravity=False):
        self.experiment_folder = experiment_folder
        self.config_file = config_file
        self.writefile = writefile
        self.simulation = load_simulation_from_config(self.config_file, gravity=gravity)
        self.pin_position, self.option = load_pin_from_config(self.config_file)
        self.simulation.reset()
        self.iterations = iterations
        self.datafile = datafile


    def train(self):
        """
        Trains a policy and dumps it to file.
        """
        env = normalize(PinEnvDiscrete(self.simulation, self.pin_position[0], self.pin_position[1], self.simulation.trajectory, 0, self.option))
        policy = CategoricalMLPPolicy(
            env_spec=env.spec,
            hidden_sizes=(32, 32)
        )
        baseline = ZeroBaseline(env_spec=env.spec)

        scores = []
        for i in range(10):
            print "Iteration", i
            algo = TRPO(
                env=env,
                policy=policy,
                baseline=baseline,
                batch_size=1000,
                step_size = 0.001,
                discount = 1,
                n_itr = 3
            )

            # run_experiment_lite(
            #     algo.train(),
            #     n_parallel=1,
            #     snapshot_mode="last",
            #     log_dir="temp",
            #     seed=1,
            #     # plot=True,
            # )
            algo.train()
            score = rollout(env, policy, flag=True, wait=False, render=False)
            scores.append(score)


        ## richard
        self._env = env
        self._policy = policy 
        rollout(env, policy, flag=True, wait=False)
        
        with open(self.datafile, "w+") as f:
            pickle.dump(scores, f)

        with open(self.writefile, "w+") as f:
            pickle.dump(policy, f)

    @property
    def env(self):
        return PinEnvDiscrete(self.simulation, self.pin_position[0], self.pin_position[1], self.simulation.trajectory, 0, self.option)


def rollout(env, policy, flag=False, wait=False, render=False):
    observations, actions, rewards = [], [], []
    if flag:
        env = env._wrapped_env
    if wait:
        import ipdb; ipdb.set_trace()
    observation = env.reset()
    total = len(env.simulation.cloth.shapepts)
    while not env.traj_index >= len(env.trajectory) - 1:
        if render:
            env.render()
        action = policy.get_action(np.array(observation))[0]
        actions.append(action)
        observations.append(observation)
        observation, reward, terminal, _ = env.step(action)
        rewards.append(reward)
    print(rewards)
    print "Score", total - len(env.simulation.cloth.shapepts)
    return env.simulation.cloth.evaluate(), rewards

def rollout_no_policy(env, policy=None, flag=False, wait=False, render=False):
    observations, actions, rewards = [], [], []
    if flag:
        env = env._wrapped_env
    if wait:
        import ipdb; ipdb.set_trace()
    observation = env.reset()
    total = len(env.simulation.cloth.shapepts)
    while not env.traj_index >= len(env.trajectory) - 1:
        if render:
            env.render()
        action = 0
        actions.append(action)
        observations.append(observation)
        observation, reward, terminal, _ = env.step(action)
        rewards.append(reward)
    print(rewards)
    print "Score", total - len(env.simulation.cloth.shapepts)
    return env.simulation.cloth.evaluate(), rewards




if __name__ == '__main__':
    
    writefile = "one.p"
    datafile = "data.p"
    config_file = "experiment.json"
    # import ipdb; ipdb.set_trace()
    pg = PolicyGenerator("", config_file, writefile, datafile)
    pg.train()

    policy =  "one.p"
    datafile = "test1.p"

    with open(policy, "rb") as f:
        policy = pickle.load(f)

    d = {}
    for i in range(21):
        gravity = -i * 200
        d[gravity] = []
        print gravity
        env = PolicyGenerator("", "experiment.json", gravity=gravity).env
        for j in range(20):
            a, b = rollout(env, policy)
            d[gravity].append([a, b])

    with open(datafile, "w+") as f:
        pickle.dump(d, f)

    datafile = "nopolicy.p"

    d = {}
    for i in range(21):
        gravity = -i * 200
        d[gravity] = []
        print gravity
        env = PolicyGenerator("", "experiment.json", gravity=gravity).env
        for j in range(20):
            d[gravity].append(list(rollout_no_policy(env)))

    with open(datafile, "w+") as f:
        pickle.dump(d, f)

    datafile = "nograsp.p"

    d = {}
    for i in range(21):
        gravity = -i * 200
        d[gravity] = []
        print gravity
        env = PolicyGenerator("", "experiment.json", gravity=gravity).env
        env.pinx, env.piny = 0, 0
        for j in range(20):
            d[gravity].append(list(rollout_no_policy(env)))

    with open(datafile, "w+") as f:
        pickle.dump(d, f)

