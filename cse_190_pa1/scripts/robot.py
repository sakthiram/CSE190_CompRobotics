#!/usr/bin/env python

import rospy
import random as r
import math as m
import numpy as np
from copy import deepcopy
from cse_190_assi_1.msg import temperatureMessage, RobotProbabilities
from cse_190_assi_1.srv import requestTexture, moveService
from std_msgs.msg import Bool, String, Float32
from collections import deque
from read_config import read_config


class RobotController():
    def __init__(self):
        self.config = read_config()
        rospy.init_node("robot_controller")
        self.temperature_subscriber = rospy.Subscriber(
                "/temp_sensor/data",
                temperatureMessage,
                self.handle_incoming_temperature_data
        )
        self.texture_requester = rospy.ServiceProxy(
                "requestTexture",
                requestTexture,
        )
        self.move_requester = rospy.ServiceProxy(
                "moveService",
                moveService,
        )
        self.temp_activator = rospy.Publisher(
                "/temp_sensor/activation",
                Bool,
                queue_size = 10
        )
        self.temp_res_pub = rospy.Publisher(
                "/results/temperature_data",
                Float32,
                queue_size = 10
        )
        self.tex_res_pub = rospy.Publisher(
                "/results/texture_data",
                String,
                queue_size = 10
        )
        self.prob_res_pub = rospy.Publisher(
                "/results/probabilities",
                RobotProbabilities,
                queue_size = 10
        )
        self.shutdown_pub = rospy.Publisher(
                "/map_node/sim_complete",
                Bool,
                queue_size = 10
        )
        self.initialize_maps()
        self.initialize_beliefs()
        self.motions = deque(self.config['move_list'])
        rospy.sleep(2)
        self.simulate()

    def initialize_maps(self):
        self.heat_map = self.generate_heatmap(self.config['pipe_map'])

    def initialize_beliefs(self):
        """
        set the initial probability matrix - everywhere is equally likely.
        """
        init_prob = 1.0 / float(len(self.config['texture_map'])) / float(len(self.config['texture_map'][0]))
        starting_beliefs = [[init_prob for row in range(len(self.config['texture_map'][0]))]
                            for col in range(len(self.config['texture_map']))]
        self.probability_matrix = starting_beliefs

    def generate_heatmap(self, pipe_map):
        """
        Generates a heat map based on whether there is a cold or hot pipe
        running under each tile. Tiles with no pipes underneath will have
        a "warm" temperature
        """
        temp_cold = 20.0
        temp_hot  = 40.0
        temp_warm = 25.0
        heat_map = [[ 0.0 for x in row] for row in pipe_map]
        for i in range(len(pipe_map)):
            for j in range(len(pipe_map[0])):
                if pipe_map[i][j] == 'C':
                    heat_map[i][j] = temp_cold

                elif pipe_map[i][j] == 'H':
                    heat_map[i][j] = temp_hot
                else:
                    heat_map[i][j] = temp_warm
        return heat_map

    def simulate(self):
        """
        Kicks the whole thing off. Calls rospy.spin() after activating
        the temperature sensor because from there on out everything happens
        on callback functions.
        """
        self.begin_simulation()
        rospy.spin()

    def begin_simulation(self):
        """
        Publishes to the topic that activates the temperature sensor.
        """
        activation_message = Bool()
        activation_message.data = True
        self.temp_activator.publish(activation_message)

    def end_simulation(self):
        """
        Publishes to the topic that stops the temperature sensor at the
        end of the simulation.
        """
        activation_message = Bool()
        activation_message.data = False
        self.temp_activator.publish(activation_message)
        self.shutdown_pub.publish(True)
        rospy.sleep(1)
        rospy.signal_shutdown("because I said so")

    def handle_incoming_temperature_data(self, message):
        """
        Callback function for the temp data. Initiates the timestep.
        """
        temp_reading = message.temperature
        self.timestep(temp_reading)

    def timestep(self, temp_reading):
        """
        Main sense->evalute->move function
        """
        self.update_from_temp(temp_reading)
        texture_reading = self.request_texture_reading()
        self.update_from_tex(texture_reading)
        self.publish_sensor_values(texture_reading, temp_reading)
        if len(self.motions) > 0:
            self.move_and_update(self.motions.popleft())
            self.publish_beliefs()
        else:
            print "Motion complete"
            self.show(self.probability_matrix)
            self.end_simulation()

    def publish_sensor_values(self, tex, temp):
        self.tex_res_pub.publish(tex)
        self.temp_res_pub.publish(temp)

    def publish_beliefs(self):
        prob_list = [p for row in self.probability_matrix for p in row]
        self.prob_res_pub.publish(prob_list)

    def show(self, p):
        """
        prints a probability matrix to the screen in a readable format
        """
        rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
        print '[' + ',\n '.join(rows) + ']'

    def update_from_temp(self, temp_reading):
        """
        Update the beliefs about the robot's position based on a temperature
        measurement.
        """
        temp_probs = deepcopy(self.probability_matrix)
        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                if self.config['temp_noise_std_dev'] == 0:
                    prob_hit = heat_map[i][j] == temp_reading
                else:
                    prob_hit = 1/(m.sqrt(2*m.pi)*self.config['temp_noise_std_dev'])*m.e**(-0.5*(float(self.heat_map[i][j] - temp_reading)/self.config['temp_noise_std_dev'])**2)
                temp_probs[i][j] = prob_hit * self.probability_matrix[i][j]
	print "Temperature Probabilities"
	self.show(temp_probs)

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs
	print "Temperature Probabilities NORMALIZED"
	self.show(self.probability_matrix)
	#print self.probability_matrix

    def update_from_tex(self, texture_reading):
        """
        Update the beliefs about the robot's position based on a texture
        measurement.
        """
        temp_probs = deepcopy(self.probability_matrix)
        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                hit = int(self.config['texture_map'][i][j] == texture_reading)
                temp_probs[i][j] = (self.config['prob_tex_correct'] * hit + (1 - self.config['prob_tex_correct']) * (1 - hit)) * self.probability_matrix[i][j]
	print "Texture Probabilities"
	self.show(temp_probs)

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs
	print "Texture Probabilities NORMALIZED"
	self.show(self.probability_matrix)

    def request_texture_reading(self):
        """
        Get a texture reading from the temperature sensor node via the
        texture request service.
        """
        response = self.texture_requester()
        texture_reading = response.data
        return texture_reading

    def move_and_update(self, move):
        self.send_move_command(move)
        self.update_beliefs_after_move(move)

    def send_move_command(self, move_command):
        self.move_requester(move_command)

    def update_beliefs_after_move(self, move_command):
        """
        Update position beliefs after attempting a move based on the known
        parameters of the motion model.
        """
        temp_probs = deepcopy(self.probability_matrix)

        num_cols = len(temp_probs[0])
        num_rows = len(temp_probs)
        for i in range(num_rows):
            for j in range(num_cols):
                possible_starts = [
                        [(i-1) % num_rows, j],
                        [i, j],
                        [(i+1) % num_rows, j],
                        [i, (j-1) % num_cols],
                        [i, (j+1) % num_cols]
                ]
                correct_start = [(i - move_command[0]) % num_rows, (j - move_command[1]) % num_cols]
                incorrect_starts = deepcopy(possible_starts)
                incorrect_starts.remove(correct_start)
                #print possible_starts, incorrect_starts, correct_start
                temp_probs[i][j] = self.config['prob_move_correct'] * self.probability_matrix[correct_start[0]][correct_start[1]]
                for start in incorrect_starts:
                    temp_probs[i][j] += ((1 - self.config['prob_move_correct'])/4) * temp_probs[start[0]][start[1]]
	print "Move Probabilities"
	self.show(temp_probs)

        total_prob = sum(sum(p) for p in temp_probs)
        if total_prob == 0:
            print "sum of probabilities is zero"
        else:
            temp_probs = [[prob/total_prob for prob in row] for row in temp_probs]
        self.probability_matrix = temp_probs
	print "Move Probabilities NORMALIZED"
	self.show(self.probability_matrix)


if __name__ == '__main__':
    rc = RobotController()
