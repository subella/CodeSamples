import numpy as np
import pygame
from pygame.locals import *
from scipy.spatial import KDTree

class Agent(object):
	def __init__(self,
				 *
				 id,
	             initial_state=np.array([0,0,0,0]), 
	             agent_type=0,
		         m=1, F=[0,0], k=0.5,
                 constraint="None"):
		self.id = id
		self.initial_state = initial_state.copy()
		self.state = initial_state.copy()
		self.type = agent_type
		self.state_array = [self.state.copy()]
		self.type_array = [self.type]
		self.m = 1
		self.F = F.copy()
		self.k = 0.5
		self.tree = None
		self.future_state_array = []
		self.constraint = constraint
		self.sampled_ids = []

	@staticmethod
	def dynamics_fn(state, t, m, F, k):
		x, y, x_dot, y_dot = state
		F_x, F_y = F
		state_dot = [x_dot, y_dot, (F_x - k * x_dot) / m, (F_y - k * y_dot) / m]
		return state_dot

	def update_state_array(self):
		self.state_array.append(self.state.copy())
		self.type_array.append(self.type)
	
	def update_future_state_array(self, current_id, distance):
		# TODO only works for 1D case
		current_pos = self.state_array[current_id][:2]
		if self.constraint == "horizontal":
			goal_pos = current_pos + np.array([0, distance])
		elif self.constraint == "vertical":	
			goal_pos = current_pos - np.array([distance, 0])
		else:
			print("Future states only works for 1D case right now!")

		end_id = self.get_closest_id(goal_pos, current_id)
		self.future_state_array = self.state_array[current_id:end_id]

	def get_closest_id(self, goal_pos, current_id):
		# TODO: handle case where cars go backwards
		closest_ids = [self.tree.query(goal_pos)[1]]

		return self.sampled_ids[closest_ids[0]]

	def get_past_states(self, num_states, seconds_back, dt):
		end_index = seconds_back / dt
		indices = np.linspace(0, end_index, num_states, dtype=int)
		if end_index > len(self.state_array):
			return None
		indices = len(self.state_array) - indices - 1
		np_state_array = np.asarray(self.state_array)
		past_states = np_state_array[[indices]]
		# Reverse order to match training
		past_states = np.flip(past_states, 0).copy()
		return past_states

	def finish(self):
		pos_array = np.asarray(self.state_array)[:,:2]
		# Downsampling doesn't work for some reason
		# Source of recursion error
		self.sampled_ids = np.linspace(1, pos_array.shape[0], 100, dtype=int) - 1
		self.tree = KDTree(pos_array[[self.sampled_ids]])

	def reset(self):
		self.state = self.initial_state.copy()
		self.state_array = [self.state.copy()]
		self.F = [0, 0]
		self.future_state_array = []
		self.type = 0
		self.type_array = [self.type]

class SimAgent(Agent):
	def __init__(self, 
				 *,
				 image_path="",
				 image_scale=.5,
				 image_rotation=180,
				 color=(0,255,0),
				 keycodes=(K_t, K_y, K_u, K_i, K_e, K_q), 
				 **kwargs):
		self.image_path = image_path
		self.image_scale = image_scale
		self.image_rotation = image_rotation
		self.image = None
		self.color = color
		self.keycodes = keycodes
		super().__init__(**kwargs)

	def load_image(self):
		try:
			image = pygame.image.load(self.image_path).convert_alpha()
			width = image.get_width() * self.image_scale   #Window size a bit smaller than monoitor size
			height= width * image.get_height() / image.get_width()
			image = pygame.transform.scale(image, (width, height))
			self.image = pygame.transform.rotate(image, self.image_rotation)
		except Exception as E:
			print(E)
			self.image = None

if __name__=="__main__":
	agent = Agent(id=0)
	sim_agent = SimAgent(id=0, initial_state=[0,1,2,3],
		                 color=(255,0,0), keycodes=(K_UP, K_DOWN, K_RIGHT, K_LEFT))
