import numpy as np
import json
import os
from itertools import combinations
from scipy.spatial import KDTree
from shapely.geometry import LineString

class Annotater(object):
	
	def __init__(self, agents, look_ahead=3, dt=0.01):
		self.agents = agents
		self.look_ahead = look_ahead
		self.dt = dt
		self.data = {"data" : []}
		self.current_data = []

	def classify_interaction(self, t):
		current_id = int(t / self.dt)
		label_array = [[""] * len(self.agents) for i in range(len(self.agents))]

		look_ahead = self.look_ahead
		for agent in self.agents:
			if agent.type == 1:
				look_ahead = self.look_ahead + 1
				break

		for agent in self.agents:
			agent.update_future_state_array(current_id, look_ahead)
			if len(agent.future_state_array) <= 1:
				return None

		for agent_pair in combinations(self.agents, 2):
			agent_1, agent_2 = agent_pair
			line_segment_1 = LineString([(agent_1.future_state_array[0][0], agent_1.future_state_array[0][1]),
										 (agent_1.future_state_array[-1][0], agent_1.future_state_array[-1][1])])

			line_segment_2 = LineString([(agent_2.future_state_array[0][0], agent_2.future_state_array[0][1]),
										 (agent_2.future_state_array[-1][0], agent_2.future_state_array[-1][1])])

			intersection_point = line_segment_1.intersection(line_segment_2)
			if intersection_point.is_empty:
				label_array[agent_1.id][agent_2.id] = "IGNORING"
				label_array[agent_2.id][agent_1.id] = "IGNORING"
			else:
				point = np.array(intersection_point.coords[0])
				if agent_1.get_closest_id(point, current_id) < agent_2.get_closest_id(point, current_id):
					label_array[agent_1.id][agent_2.id] = "GOING"
					label_array[agent_2.id][agent_1.id] = "YIELDING"
				else:
					label_array[agent_1.id][agent_2.id] = "YIELDING"
					label_array[agent_2.id][agent_1.id] = "GOING"

		self.update_data(t, label_array)

		return(label_array)

	def update_data(self, t, label_array):
		data_entry = {}
		data_entry["time"] = t
		data_entry["labels"] = label_array
		data_entry["agents"] = []
		for agent in self.agents:
			agent_data = {}
			agent_data["agent_id"] = agent.id
			agent_data['state'] = agent.state.tolist()
			agent_data["type"] = agent.type
			data_entry["agents"].append(agent_data)
	        
		self.current_data.append(data_entry)

	def reset_data(self):
		self.data["data"].append(self.current_data)
		self.current_data = []

	def save_data(self):
		file = os.path.join(os.path.dirname(__file__), '../data/data_siren_test.json')
		with open(file, 'w') as outfile:
		    json.dump(self.data, outfile, indent=4)

    #def reset():
    #	self.data {}

