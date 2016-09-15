from __future__ import absolute_import

from rllab.algos.trpo import TRPO
from rllab.algos.vpg import VPG
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.baselines.zero_baseline import ZeroBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy

from trpo_discrete_pin import *
from environment_rep.pin_env_discrete import *
import numpy as np
import sys, pickle, os
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *
from tension_finder import *
import IPython


class PolicyGenerator2(object):

    """
    Class that trains a tensioning policy given a config file and a file to dump the policy to.
    """

    def __init__(self, simulation, x, y, writefile, datafile, iterations=25, policy=None):
        self.writefile = writefile
        self.datafile = datafile
        self.simulation = simulation
        self.pin_position, self.option = (x, y), 50
        self.simulation.reset()
        self.iterations = iterations
        self.policy = policy


    def train(self):
        """
        Trains a policy and dumps it to file.
        """
        env = normalize(PinEnvDiscrete(self.simulation, self.pin_position[0], self.pin_position[1], self.simulation.trajectory, 0, self.option))
        policy = CategoricalMLPPolicy(
            env_spec=env.spec,
            hidden_sizes=(32, 32)
        )
        if self.policy:
            policy = self.policy
        baseline = ZeroBaseline(env_spec=env.spec)

        scores = []
        for i in range(6):
            print "Iteration", i
            algo = TRPO(
                env=env,
                policy=policy,
                baseline=baseline,
                batch_size=500,
                step_size = 0.01,
                discount = 1,
                n_itr = 2
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
            print score


        with open(self.writefile, "w+") as f:
            pickle.dump(policy, f)

        for i in range(10):
            score = rollout(env, policy, flag=True, wait=False, render=False)
            print score
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
    return env.simulation.cloth.evaluate()

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
    return env.simulation.cloth.evaluate()


if __name__ == '__main__':

    experiment_folder = "experiment_data/physical_experiments/1/"
    config_file = "experiment.json"
    # import ipdb; ipdb.set_trace()
    pg = PolicyGenerator(experiment_folder, config_file, "")
    simulation = pg.simulation
    option = pg.option
    cloth = simulation.cloth
    tpf = TensionPointFinder(cloth)

    # plt.imshow(np.flipud(tpf.find_valid_pts(allpts=True)), cmap='Greys_r')
    # plt.show()

    pts = tpf.find_valid_pts(allpts=True).nonzero()
    lst = []
    for i in range(len(pts[0])):
    	if min([pts[0][i]*20+50, pts[1][i]*20+50]) > 100 and max([pts[0][i]*20+50, pts[1][i]*20+50]) < 500:
        	lst.append([pts[0][i]*20+50, pts[1][i]*20+50])
    pts = lst

    pts_to_test = pts
    if len(pts_to_test) > 30:
        indices = np.random.choice(len(pts_to_test), 30, replace=False)
        pts_to_test = np.random.permutation(np.array(pts_to_test)[indices,:].tolist())

    print pts_to_test
    # IPython.embed()
    # simulation.render = True
    # pts_to_test = [[170, 340]]
    directory = "physical_pts/"
    if not os.path.exists(directory):
        os.makedirs(directory)
    # simulation.render = True
    for pt in pts_to_test:
        x, y = pt[0], pt[1]
        writefile = directory + "policy_" + str(x) + "_" + str(y) + ".p"
        datafile = directory + "data_" + str(x) + "_" + str(y) + ".p"
        pg2 = PolicyGenerator2(simulation, x, y, writefile, datafile)
        pg2.train()


    # pg.train()