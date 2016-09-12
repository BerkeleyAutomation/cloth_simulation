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
from tension_finder import *

# stub(globals())

class PolicyGenerator:

    """
    Class that trains a tensioning policy given a config file and a file to dump the policy to.
    """

    def __init__(self, pin_position=None, experiment_folder="", config_file="experiment.json", writefile="policydiscrete.p", datafile="data.p", iterations=20):
        self.experiment_folder = experiment_folder
        self.config_file = config_file
        self.writefile = writefile
        self.datafile = datafile
        self.simulation = load_simulation_from_config(self.config_file)
        self.pin_position, self.option = load_pin_from_config(self.config_file)
        self.option = 50
        self.simulation.reset()
        self.iterations = iterations



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
                batch_size=500,
                step_size = 0.01,
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
        # rollout(env, policy, flag=True, wait=False, render=False)
        
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
    import IPython; IPython.embed()
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
    
    writefile = "circle/one.p"
    datafile = "circle/data1.p"
    config_file = "circle/experiment.json"

    pg = PolicyGenerator(None, "", config_file, writefile, datafile)
    pg.train()

    writefile = "circle/two.p"
    datafile = "circle/data2.p"
    config_file = "circle/experiment2.json"

    pg = PolicyGenerator(None, "", config_file, writefile, datafile)
    pg.train()

    writefile = "circle/three.p"
    datafile = "circle/data3.p"
    config_file = "circle/experiment3.json"

    pg = PolicyGenerator(None, "", config_file, writefile, datafile)
    pg.train()

    writefile = "circle/four.p"
    datafile = "circle/data4.p"
    config_file = "circle/experiment4.json"

    pg = PolicyGenerator(None, "", config_file, writefile, datafile)
    pg.train()

    writefile = "circle/five.p"
    datafile = "circle/data5.p"
    config_file = "circle/experiment5.json"

    pg = PolicyGenerator(None, "", config_file, writefile, datafile)
    pg.train()



    policy = "circle/one.p"
    datafile = "circle/test1.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2


    policy = "circle/two.p"
    datafile = "circle/test2.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/three.p"
    datafile = "circle/test3.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/four.p"
    datafile = "circle/test4.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2


    policy = "circle/five.p"
    datafile = "circle/test5.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment4.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2


    policy = "circle/one.p"
    datafile = "circle/test5_1.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment5.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/two.p"
    datafile = "circle/test5_2.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment5.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/three.p"
    datafile = "circle/test5_3.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment5.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/four.p"
    datafile = "circle/test5_4.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment5.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2

    policy = "circle/five.p"
    datafile = "circle/test5_5.p"
    policy = pickle.load(open(policy, "rb"))
    env = PolicyGenerator(None, "", "circle/experiment5.json", "", "").env

    scores = []
    for i in range(10):
        score1, score2 = rollout(env, policy)
        scores.append([score1, score2])
        print score1, score2
