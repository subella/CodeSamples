import os
import argparse
import numpy as np
from pygame.locals import *
from multiagent.agent import SimAgent
from multiagent.sim import Simulation, DataSim, PredictionSim

if __name__=="__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', type=str, required=True)
	args = parser.parse_args()

	assets_path = os.path.join(os.path.dirname(__file__), '../assets')
	agent_1 = SimAgent(initial_state=np.array([3, 0.75, 0, 0]),
					   agent_type=0,
					   image_path=os.path.join(assets_path, "blue_car.png"),
					   image_scale=0.03333,
					   image_rotation=180,
	                   color=(0,255,0),
	                   keycodes=(K_UP, K_DOWN, K_LEFT, K_RIGHT, K_b, K_RSHIFT),
	                   constraint="vertical")

	# agent_2 = SimAgent(initial_state=np.array([0, -3, 0, 0]),
	# 				   agent_type=0,
	# 				   image_path=os.path.join(assets_path, "green_car.png"),
	# 				   image_scale=0.1,
	# 				   image_rotation=90,
	#                    color=(0,255,0),
	#                    keycodes=(K_w, K_s, K_a, K_d, K_e, K_q),
	#                    constraint="horizontal")

	agent_2 = SimAgent(initial_state=np.array([0, -3, 0, 0]),
					   agent_type=0,
					   image_path=os.path.join(assets_path, "police_car.png"),
					   image_scale=0.133333,
					   image_rotation=90,
	                   color=(0,255,0),
	                   keycodes=(K_w, K_s, K_a, K_d, K_e, K_q),
	                   constraint="horizontal")
	agents = [agent_1, agent_2]

	# Ids must be zero indexed
	for id, agent in enumerate(agents):
		agent.id = id

	if args.mode == "generation":
		print("Starting data generation sim...")
		sim = DataSim(agents)
	elif args.mode == "prediction":
		print("Starting prediction sim...")
		sim = PredictionSim(agents)
	sim.run()
