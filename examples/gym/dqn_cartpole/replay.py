import torch
import collections  #replay buffer에서 쓰일 deque를 import 하기 위함
import random

"""
    A simple Random Replay Buffer.
"""


class ReplayBuffer:
    def __init__(self, buffer_limit=int(1e5), num_envs=1):
        self.buffer = collections.deque(maxlen=buffer_limit)    # buffer_limit : buffer 최대 크기
        self.num_envs = num_envs

    def push(self, obs, action, reward, next_obs, done):
        '''
        push( ) : 집어 넣는 method
        transition이 들어오면 buffer에 넣어주는 역할 
        '''
        self.buffer.append(tuple([obs, action, reward, next_obs, done]))

    def sample(self, mini_batch_size):
        '''
        sample( ) : 빼내는 method
        n개를 뽑아 tensor로 만드는 과정 
        buffer에서 n개를 랜덤으로 뽑아서 mini_batch_를 만듦
        '''
        obs, action, reward, next_obs, done = zip(*random.sample(self.buffer, mini_batch_size))

        rand_idx = torch.randperm(mini_batch_size * self.num_envs)  # random shuffle tensors

        obs = torch.cat(obs)[rand_idx]
        action = torch.cat(action)[rand_idx]
        reward = torch.cat(reward)[rand_idx]
        next_obs = torch.cat(next_obs)[rand_idx]
        done = torch.cat(done)[rand_idx]
        return obs, action, reward, next_obs, done

    def size(self):
        return len(self.buffer)
