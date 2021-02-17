from ros2sim import parsers
from ros2sim import sims

import gym
import gym_solo

from gym_solo.core import obs

import logging

if __name__ == '__main__':
  logging.basicConfig(level=logging.INFO)
  env = gym.make('solo8vanilla-realtime-v0')

  env.obs_factory.register_observation(obs.TorsoIMU(env.robot))
  env.obs_factory.register_observation(obs.MotorEncoder(env.robot))
  
  sim_executor = parsers.SimExecutor(env)
  
  simulator = sims.SerialSimulator(sim_executor)
  simulator.serve()