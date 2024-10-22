from cartpole_env import CartpoleEnv
from replay import ReplayBuffer

import torch.distributions
import torch.nn as nn
import torch.nn.functional as F

# define network architecture
class Net(nn.Module):
    def __init__(self, num_obs=4, num_act=2):
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

class DQN:
    def __init__(self, args):
        self.args = args

        # initialise parameters
        self.replay = ReplayBuffer(num_envs=args.num_envs)

        self.act_space = 2
        self.discount = 0.99
        self.mini_batch_size = 128
        self.batch_size = self.args.num_envs * self.mini_batch_size
        self.tau = 0.995
        self.num_eval_freq = 100
        self.lr = 3e-4

        self.run_step = 1
        self.score = 0

        # define Q-network
        self.q        = Net(num_act=self.act_space).to(self.args.sim_device)
        self.q_target = Net(num_act=self.act_space).to(self.args.sim_device)
        self.soft_update(self.q, self.q_target, tau=0.0)
        self.q_target.eval()
        self.optimizer = torch.optim.Adam(self.q.parameters(), lr=self.lr)

    # some useful functions
    def soft_update(self, net, net_target, tau):
        # update target network with momentum (for approximating ground-truth Q-values)
        for param_target, param in zip(net_target.parameters(), net.parameters()):
            param_target.data.copy_(param_target.data * tau + param.data * (1.0 - tau))

    def compute_loss(self):
        # policy update using TD loss
        self.optimizer.zero_grad()

        obs, act, reward, next_obs, done_mask = self.replay.sample(self.mini_batch_size)
        q_table = self.q(obs)
        q_val = q_table.reshape(self.batch_size, -1).max(1)[0]

        with torch.no_grad():
            q_val_next = self.q_target(next_obs)
            q_val_next = q_val_next.reshape(self.batch_size, -1).max(1)[0]

        target = reward + self.discount * q_val_next * done_mask
        loss = F.mse_loss(q_val, target)
        loss.backward()
        self.optimizer.step()

        # soft update target networks
        self.soft_update(self.q, self.q_target, self.tau)
        return loss

    def select_action(self, obs):
        # linear annealing from 0.8 to 0.01 over 20k steps
        self.epsilon = max(0.01, 0.8 - 0.01 * (self.run_step / 20))

        # collect data
        coin = torch.rand(self.args.num_envs, device=self.args.sim_device) < self.epsilon

        rand_act = torch.rand(self.args.num_envs, device=self.args.sim_device)
        with torch.no_grad():
            q_table = self.q(obs)
            true_act = torch.cat([(q_table[b] == q_table[b].max()).nonzero(as_tuple=False)[0]
                                for b in range(self.args.num_envs)])
            true_act = true_act / (self.act_space - 1)

        # choose random action or policy action
        act = coin.float() * rand_act + (1 - coin.float()) * true_act
        clip_act = 2 * (act - 0.5)  # maps to -1 to 1
        action = clip_act.view(-1,1)

        return action

    def update(self, action, next_obs, obs, reward, done):

        self.replay.push(obs, action, reward, next_obs, 1 - done.long())

        # training mode
        if self.replay.size() > self.mini_batch_size:
            loss = self.compute_loss()
            self.score += torch.mean(reward.float()).item() / self.num_eval_freq

            # evaluation mode
            if self.run_step % self.num_eval_freq == 0:
                print('Steps: {:04d} | Reward {:.04f} | TD Loss {:.04f} Epsilon {:.04f} Buffer {:03d}'
                    .format(self.run_step, self.score, loss.item(), self.epsilon, self.replay.size()))
                self.score = 0

        self.run_step += 1
