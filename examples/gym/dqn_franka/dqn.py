from env import Franka
from replay import ReplayBuffer

import torch.distributions
import torch.nn as nn
import torch.nn.functional as F
import math

"""
    Policy built on top of Vanilla DQN.
"""


# define network architecture
class Net(nn.Module):
    def __init__(self, num_obs=26, num_act=9):
        super(Net, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(num_obs, 256),
            nn.LeakyReLU(),
            nn.Linear(256, 256),
            nn.LeakyReLU(),
            nn.Linear(256, num_act),
        )

    def forward(self, x):
        return self.net(x)


# some useful functions
def soft_update(net, net_target, tau):
    # update target network with momentum (for approximating ground-truth Q-values)
    for param_target, param in zip(net_target.parameters(), net.parameters()):
        param_target.data.copy_(param_target.data * tau + param.data * (1.0 - tau))


class DQN:
    def __init__(self, args):
        self.args = args

        # initialise parameters
        self.env = Franka(args)

        self.act_space = 9
        self.obs_space = 26
        self.discount = 0.99
        self.mini_batch_size = 32
        self.batch_size = self.args.num_envs * self.mini_batch_size
        self.buffer_limit = 8096
        self.tau = 0.95
        self.num_eval_freq = 100
        self.lr = 5e-4
        self.run_step = 1
        self.score = 0

        self.replay = ReplayBuffer(buffer_limit=self.buffer_limit, num_envs=args.num_envs)

        # define Q-network
        self.q        = Net(num_obs=self.obs_space, num_act=self.act_space).to(self.args.sim_device)
        self.q_target = Net(num_obs=self.obs_space, num_act=self.act_space).to(self.args.sim_device)
        soft_update(self.q, self.q_target, tau=0.0)
        self.q_target.eval()
        self.optimizer = torch.optim.Adam(self.q.parameters(), lr=self.lr)

    def update(self):
        # policy update using TD loss
        self.optimizer.zero_grad()

        obs, act, reward, next_obs, done_mask = self.replay.sample(self.mini_batch_size)
        q_val = self.q(obs)
        q_val = q_val.reshape(self.batch_size, -1).max(1)[0]

        with torch.no_grad():
            q_val_next = self.q_target(next_obs)
            q_val_next = q_val_next.reshape(self.batch_size, -1).max(1)[0]
        target = reward + self.discount * q_val_next * done_mask
        # target = reward.view(-1, 1) + self.discount * q_val_next * done_mask.view(-1, 1)
        # print("q_val: ", q_val.shape, q_val)
        # print("q_val_next: ", q_val_next.shape, q_val_next)

        # loss = F.smooth_l1_loss(q_val, target)
        loss = F.mse_loss(q_val, target)
        # print(loss)
        loss.backward()
        self.optimizer.step()

        # soft update target networks
        soft_update(self.q, self.q_target, self.tau)
        return loss

    def choose_act(self, obs, epsilon=0.0): # action selection policy
        '''
        inputs: observation tensor
        output: action tensor
        '''
        coin = torch.rand([self.args.num_envs, 1], device=self.args.sim_device) < epsilon
        rand_act = torch.rand([self.args.num_envs, self.act_space], device=self.args.sim_device)
        rand_act = 2 * (rand_act - 0.5)  # maps to -1 to 1

        with torch.no_grad():
            q_table = self.q(obs)
            true_act = q_table
            # if values higher than zero, set to 1. else set to -1
            true_act = torch.where(true_act > 0, torch.tensor(1.0, device=self.args.sim_device), torch.tensor(-1.0, device=self.args.sim_device))
        
        # if coin is true, choose random action, else choose policy action
        act = coin.float()*rand_act + (1-coin.float())*true_act
        # act = torch.clamp(clip_act, -1, 1)
        return act

    def run(self):
        epsilon = max(0.01, 0.8 - 0.01 * (self.run_step / 300)) # linear annealing from 0.8 to 0.01 over steps Epsilon으로 E-Greedy 탐색법에서 0.9부터 0.05까지 EPISODE를 반복할수록 낮추어가며 진행

        # collect data
        obs = self.env.obs_buf.clone()
        action = self.choose_act(obs, epsilon)
        self.env.step(action)
        next_obs, reward, done = self.env.obs_buf.clone(), self.env.reward_buf.clone(), self.env.reset_buf.clone()
        self.env.reset()
        self.replay.push(obs, action, reward, next_obs, 1 - done)

        # training mode
        if self.replay.size() > self.mini_batch_size:
            loss = self.update()
            self.score += torch.mean(reward.float()).item() / self.num_eval_freq

            # evaluation mode
            if self.run_step % self.num_eval_freq == 0:
                print('Steps: {:04d} | Reward {:.04f} | TD Loss {:.04f} Epsilon {:.04f} Buffer {:03d}'
                      .format(self.run_step, self.score, loss.item(), epsilon, self.replay.size()))
                self.score = 0

        self.run_step += 1
