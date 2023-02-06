import numpy as np
from pygame.locals import *
from multiagent.agent import SimAgent
from multiagent.sim import Simulation, PredictionSim

if __name__=="__main__":
	agent_1 = SimAgent(initial_state=np.array([4, 0, 0, 0]),
	                   color=(0,255,0),
	                   keycodes=(K_UP, K_DOWN, K_LEFT, K_RIGHT, K_RCTRL),
	                   constraint="vertical")

	agent_2 = SimAgent(initial_state=np.array([0, -4, 0, 0]),
	                   color=(0,255,0),
	                   keycodes=(K_w, K_s, K_a, K_d, K_e),
	                   constraint="horizontal")

	agents = [agent_1, agent_2]

	# Ids must be zero indexed
	for id, agent in enumerate(agents):
		agent.id = id

	#sim = Simulation(agents)
	sim = PredictionSim(agents)
	sim.run()
