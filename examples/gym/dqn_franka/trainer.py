from dqn import DQN

import torch
import random
import argparse


parser = argparse.ArgumentParser()
parser.add_argument('--mode', default=None, type=str)
parser.add_argument('--port', default=None, type=int)

parser.add_argument('--sim_device', type=str, default="cuda:0", help='Physics Device in PyTorch-like syntax')
parser.add_argument('--compute_device_id', default=0, type=int)
parser.add_argument('--graphics_device_id', type=int, default=0, help='Graphics Device ID')
parser.add_argument('--num_envs', default=20, type=int)
parser.add_argument("--headless", default=False, action="store_true",
                                help="Flag to viewer")
parser.add_argument('--method', default='dqn', type=str)
parser.add_argument('--control_type', default='osc', type=str)

args = parser.parse_args()

torch.manual_seed(0)
random.seed(0)


policy = DQN(args)

while True:
    policy.run()
