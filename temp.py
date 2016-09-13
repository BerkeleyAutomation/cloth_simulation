from __future__ import absolute_import
import numpy as np
import sys, os, pickle, time
from simulation import *
from tensioner import *
from shapecloth import *
from simulation_policy import *
from registration import *
from notch_finder import *
from tension_finder import *
from pattern_designer import *
import IPython, ipdb
from environment_rep.pin_env_discrete import *
from trpo_discrete_pin import *

from rllab.algos.trpo import TRPO
from rllab.algos.vpg import VPG
from rllab.baselines.linear_feature_baseline import LinearFeatureBaseline
from rllab.baselines.zero_baseline import ZeroBaseline
from rllab.envs.gym_env import GymEnv
from rllab.envs.normalized_env import normalize
from rllab.misc.instrument import stub, run_experiment_lite
from rllab.policies.gaussian_mlp_policy import GaussianMLPPolicy
from rllab.policies.categorical_mlp_policy import CategoricalMLPPolicy

from environment_rep import *
from environment_rep.pin_env_discrete import *
import numpy as np
import sys, pickle, os
from simulation import *
from scorer import *
from shapecloth import *
from tensioner import *

# stub(globals())

class PolicyGenerator2:

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
        for i in range(10):
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

    #=========================================================
    # Setup
    #=========================================================

    if len(sys.argv) > 3:
        width = sys.argv[1]
        height = sys.argv[2]
        filename = sys.argv[3]
        pd = PatternDesigner(width, height)
    else:
        filename = sys.argv[1]
        fn = filename
        pd = PatternDesigner()

    pd.load_pts(filename)
   
    corners = pd.corners
    points = pd.trajectory

    mouse = Mouse(down=True, button=0)
    armOrientation = "right"
    shape_fn = get_shape_fn(corners, points, True)
    ##################################################################################
    cloth = ShapeCloth(shape_fn, mouse, 25, 25, 20, 20)  # ADD HERE THE PINNING POINT
    ##################################################################################
    trajectory = get_trajectory(corners, points, True)

    #=========================================================
    # Find the best segment trajectory to pass to pinning
    #=========================================================

    simulate = True

    # Find the notch points and segments to complete the trajectory
    ##################################################################################
    npf = NotchPointFinder(cloth, trajectory, [300, 300]) # ADD HERE THE PINNING POINT
    ##################################################################################
    npf.find_pts(armOrientation)
    npf.find_segments(armOrientation)

    # find the best trajectory and simulate it
    scorer = Scorer(0)

    lst = npf.find_best_trajectory(scorer)
    newOrdering, newIndices, worst_score = lst[0], lst[1], lst[2]

    tpf = TensionPointFinder(cloth)
    # plt.imshow(np.flipud(tpf.find_valid_pts()), cmap='Greys_r')
    # plt.show()

    pts = tpf.find_valid_pts(allpts=True).nonzero()
    lst = []
    for i in range(len(pts[0])):
        lst.append([pts[0][i]*10+50, pts[1][i]*10+50])
    pts = lst

    pts_to_test = pts
    if len(pts_to_test) > 30:
        indices = np.random.choice(len(pts_to_test), 30, replace=False)
        pts_to_test = np.array(pts_to_test)[indices,:].tolist()

    # while len(pts_to_test) > 30:
        # pts_to_test = pts_to_test[::2]
    # turn into a single list for simulation
    newTrajectory = []
    for seg in newOrdering:
        newTrajectory = newTrajectory + seg
    while len(newTrajectory) > 150:
        newTrajectory = newTrajectory[::2]
    simulation = Simulation(cloth, trajectory=newTrajectory)
    # simulation.render = True

    x, y = 300, 300
    env = PolicyGenerator2(simulation, x, y, "writefile", "datafile").env
    print "SCORE", rollout_no_policy(env, render=False)
    print "SCORE"

    print len(pts_to_test)
    directory = "policy_training_pts"
    if not os.path.exists(directory):
        os.makedirs(directory)
    pts_to_test = [[300, 300]]
    for pt in pts_to_test:
        x, y = pt[0], pt[1]
        writefile = "policy_training_pts/"+ fn + "_" + str(x) + "_" + str(y) + ".p"
        datafile = "policy_training_pts/data" + fn + "_" + str(x) + "_" + str(y) + ".p"
        pg2 = PolicyGenerator2(simulation, x, y, writefile, datafile)
        pg2.train()





    # if (simulate):
    #     simulation = Simulation(cloth, render=True, trajectory=newTrajectory)
    #     simulation.reset()
    #     print simulation.pin_position(300, 300)
    #     totalpts = len(simulation.cloth.shapepts)
    #     init_score = scorer.score(simulation.cloth) + totalpts
    #     print "Initial Score", init_score
    #     for i in range(len(simulation.trajectory)):
    #         simulation.update()
    #         simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    #     best_score = scorer.score(simulation.cloth) + totalpts
    #     print "Best Score", best_score

    #     simulation.reset()
    #     totalpts = len(simulation.cloth.shapepts)
    #     init_score = scorer.score(simulation.cloth) + totalpts
    #     print "Initial Score", init_score
    #     for i in range(len(simulation.trajectory)):
    #         simulation.update()
    #         simulation.move_mouse(simulation.trajectory[i][0], simulation.trajectory[i][1])
    #     best_score = scorer.score(simulation.cloth) + totalpts
    #     print "Best Score", best_score
        # save results
        # f = open("sim_files/%s/nohold" %(filename), "w+")
        # data = {'totalpts': totalpts, 'init_score': init_score, 'best_score': best_score, 'worst_score': worst_score+totalpts,
        #         'old_trajectory': oldTrajectory, 'trajectory': newOrdering, 'indices_of_pts': newIndices}
        # pickle.dump(data, f)
        # f.close()



