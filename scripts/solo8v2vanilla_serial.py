from ros2sim import parsers
from ros2sim import sims

import gym
import gym_solo

from gym_solo.core import obs


if __name__ == '__main__':
  env = gym.make('solo8vanilla-realtime-v0')

  env.obs_factory.register_observation(obs.TorsoIMU(env.robot))
  env.obs_factory.register_observation(obs.MotorEncoder(env.robot))
  print(env.get_obs())
  parser = parsers.SimExecutor(env)
  parser.action(None)
  simulator = sims.SerialSimulator(parser)
  simulator.serve()