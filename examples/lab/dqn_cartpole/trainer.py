import argparse
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an Cartpole with DQN.")
parser.add_argument('--sim_device', type=str, default="cuda:0", help='Physics Device in PyTorch-like syntax')
parser.add_argument('--compute_device_id', default=0, type=int)
parser.add_argument('--num_envs', default=20, type=int)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
from dqn import DQN
from cartpole_env import CartpoleEnv, CartpoleEnvCfg

def main():
    """Main function."""
    # create environment configuration
    env_cfg = CartpoleEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # setup Direct RL environment
    env = CartpoleEnv(cfg=env_cfg)
    env.reset()
    # simulate physics
    count = 0

    policy = DQN(args_cli)

    while simulation_app.is_running():
        # get observations
        obs = env.get_observations()
        # select action
        actions = policy.select_action(obs["policy"])
        # step the environment
        next_obs, reward, terminated, time_outs, _ = env.step(actions)
        # network update
        policy.update(actions, next_obs["policy"], obs["policy"], reward, terminated|time_outs)
        count += 1

    # close the environment
    env.close()

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()