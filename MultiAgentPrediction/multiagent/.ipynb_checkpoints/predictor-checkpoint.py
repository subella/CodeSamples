import torch
import math
from torch_geometric.data import Data
from itertools import permutations
from multiagent.model import GraphNeuralNetwork

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class Predictor(object):
	def __init__(self, model_path, agents, dt):
		self.model = GraphNeuralNetwork().to(device)
		self.model.load_state_dict(torch.load(model_path))
		self.model.eval()
		self.agents = agents
		self.dt = dt
		self.num_past_states = 5
		self.look_back_time = 2

	def predict_interactions(self):
		node_features = torch.zeros(len(self.agents), self.num_past_states, 4)
		node_types = torch.zeros((len(self.agents),1))
		for agent_id, agent in enumerate(self.agents):
			node_types[agent_id] = agent.type
			past_states = agent.get_past_states(self.num_past_states, self.look_back_time, self.dt)
			if past_states is None:
				return None
			past_states_tensor = torch.tensor(past_states)
			node_features[agent_id] = past_states_tensor

		edge_features = torch.zeros((math.factorial(len(self.agents)),4))
		edge_list = torch.zeros((math.factorial(len(self.agents)),2))
		for agent_pair_id, agent_pair in enumerate(permutations(self.agents, 2)):
			from_node, to_node = agent_pair
			edge_list[agent_pair_id][0] = from_node.id
			edge_list[agent_pair_id][1] = to_node.id
			edge_features[agent_pair_id] = torch.tensor(from_node.state - to_node.state)

		data = Data(x=node_features, edge_index=edge_list, edge_attr=edge_features).to(device)
		out = self.model(data.x, data.edge_index.type(torch.LongTensor).to(device), data.edge_attr,data.batch,node_types) 
		label_array = [[""] * len(self.agents) for i in range(len(self.agents))]

		L = torch.argmax(out, dim=1)
		for id in range(edge_list.shape[0]):
			connection = edge_list[:, id]
			row = int(connection[0].item())
			col = int(connection[1].item())
			label = self.id_to_label(L[id].item())
			label_array[row][col] = label
		return label_array

	def id_to_label(self, id):
		if id == 0:
			return "IGNORING"
		elif id == 1:
			return "GOING"
		elif id == 2:
			return "YIELDING"



