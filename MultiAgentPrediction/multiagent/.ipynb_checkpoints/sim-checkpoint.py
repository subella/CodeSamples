import sys
import os
import numpy as np
import pygame
from itertools import combinations
from pygame.locals import *
from scipy.integrate import odeint
from multiagent.agent import Agent
from multiagent.annotater import Annotater
from multiagent.predictor import Predictor

def agent_to_screen(x, y):
    # Transform 0,0 m to center of screen
    # Transform 1m = 1/8 screen size
    # Invert y axis
    w, h = pygame.display.get_surface().get_size()
    scaled_x = x * w/8
    scaled_y = -y * h/8
    px = scaled_x + w/2
    py = scaled_y + h/2
    return px, py

class Simulation(object):
    def __init__(self, agents, dt=0.01):
        pygame.init()
        width = 800      
        height = 800
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('MultiAgent Simulation')
        background_path = os.path.join(os.path.dirname(__file__), '../assets/intersection.jpg')
        background = pygame.image.load(background_path)
        self.background = pygame.transform.scale(background, (width, height))
        self.screen.blit(self.background, [0,0])
        self.finished = False      
        self.scene_finished = False
        self.start_time = 0
        self.time = self.start_time
        self.dt = dt
        self.agents = agents
        for agent in self.agents:
            agent.load_image()

    def run(self):
        while not self.finished:
            self.simulate_scene()
            self.reset()
            self.reset_agents()

    def simulate_scene(self):
        while not self.finished and not self.scene_finished:
            self.screen.blit(self.background, [0,0])
            self.get_input()
            self.solve()
            for agent in self.agents:
                agent.update_state_array()
            self.draw_agents()
            self.handle_logic()
            pygame.display.flip()
            self.time += self.dt

        for agent in self.agents:
            agent.finish()

    def handle_logic(self):
        pass

    def reset(self):
        self.time = 0
        self.scene_finished = False

    def reset_agents(self):
        for agent in self.agents:
            agent.reset()

    def get_input(self):
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            self.finished = True
        if keys[K_p]:
            self.scene_finished = True

        for agent in self.agents:
            if keys[agent.keycodes[0]] and agent.constraint != "vertical":
                agent.F[1] = .1
            if keys[agent.keycodes[1]] and agent.constraint != "vertical":
                agent.F[1] = -.1
            if keys[agent.keycodes[2]] and agent.constraint != "horizontal":
                agent.F[0] = -.1
            if keys[agent.keycodes[3]] and agent.constraint != "horizontal":
                agent.F[0] = .1
            if keys[agent.keycodes[4]]:
                agent.F = [0,0]
            if keys[agent.keycodes[5]]:
                # Turns on siren (can't turn it off)
                agent.type = 1

    def solve(self):
        for agent in self.agents:
            agent.state = odeint(Agent.dynamics_fn, agent.state, 
                                 [self.time, self.time + self.dt], 
                                 args = (agent.m, agent.F, agent.k)) [-1]

    def draw_agents(self):
        for agent in self.agents:
            px, py = agent_to_screen(agent.state[0], agent.state[1])
            if agent.type == 1:
                surface = pygame.Surface((self.screen.get_width(), self.screen.get_height()), pygame.SRCALPHA)
                rect = agent.image.get_rect()
                center = rect.center
                pygame.draw.circle(surface, (255,0,0,100), (px + center[0], py + center[1]), 100)
                pygame.draw.circle(surface, (0,0,255,100), (px + center[0], py + center[1]), 50)
                self.screen.blit(surface, [0,0])

            if agent.image is not None:
                self.screen.blit(agent.image, [px, py])
            else:
                pygame.draw.circle(self.screen, (255,0,0), (px, py), 15)


    def draw_labels(self, label_array):
        # TODO: only works with 2 agents right now
        for agent_pair in combinations(self.agents, 2): 
            agent_1, agent_2 = agent_pair
            agent_1_label = label_array[agent_1.id][agent_2.id] 
            agent_2_label = label_array[agent_2.id][agent_1.id] 
            self.draw_label(agent_1, agent_1_label)
            self.draw_label(agent_2, agent_2_label)

    def draw_label(self, agent, label):
        px, py = agent_to_screen(agent.state[0], agent.state[1])
        font = pygame.font.Font(None, 30)
        text = font.render(label, True, (255, 255, 255), (159, 182, 205))
        textRect = text.get_rect()
        textRect.centerx = px + 80
        textRect.centery = py + 40
        self.screen.blit(text, textRect)


class DataSim(Simulation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.annotater = Annotater(self.agents, dt=self.dt)

    def run(self):
        while not self.finished:
            self.simulate_scene()
            self.reset()
            self.playback()
            self.reset()
            self.reset_agents()
        self.annotater.save_data()

    def playback(self):
        for i in range(len(self.agents[0].state_array)):
            self.screen.blit(self.background, [0,0])
            for agent in self.agents:
                agent.state = agent.state_array[i]
                agent.type = agent.type_array[i]
            label_array = self.annotater.classify_interaction(self.time) 
            if label_array is None:
                break
            self.draw_agents()
            self.draw_labels(label_array)
            pygame.display.update()
            self.time += self.dt
        self.annotater.reset_data()

class PredictionSim(Simulation):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        model_path = os.path.join(os.path.dirname(__file__), "../models/best_model.pth")
        self.predictor = Predictor(model_path, self.agents, self.dt)

    def handle_logic(self):
        label_array = self.predictor.predict_interactions()
        if label_array is not None:
            self.draw_labels(label_array)

