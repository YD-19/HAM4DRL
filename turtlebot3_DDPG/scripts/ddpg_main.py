#! /usr/bin/env python

import ddpg_functions
import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Input, merge
from keras.layers.merge import Add, Concatenate
from keras.optimizers import Adam
import keras.backend as K
import tensorflow as tf
import random
from collections import deque
import os
import timeit
import csv
import math
import time
import matplotlib.pyplot as plt
import scipy.io as sio


def stack_samples(samples):
	array = np.array(samples)
	current_states = np.stack(array[:,0]).reshape((array.shape[0],-1))
	actions = np.stack(array[:,1]).reshape((array.shape[0],-1))
	rewards = np.stack(array[:,2]).reshape((array.shape[0],-1))
	new_states = np.stack(array[:,3]).reshape((array.shape[0],-1))
	dones = np.stack(array[:,4]).reshape((array.shape[0],-1))

	return current_states, actions, rewards, new_states, dones 
	
class ActorCritic:
	def __init__(self, env, sess):
		self.env  = env
		self.sess = sess

		self.learning_rate = 0.0001
		self.epsilon = .9
		self.epsilon_decay = .99995
		self.gamma = .90
		self.tau   = .01


		self.buffer_size = 1000000
		self.batch_size = 512

		self.hyper_parameters_lambda3 = 0.2
		self.hyper_parameters_eps = 0.2
		self.hyper_parameters_eps_d = 0.4

		self.demo_size = 1000

		self.memory = deque(maxlen=1000000)
		self.actor_state_input, self.actor_model = self.create_actor_model()
		_, self.target_actor_model = self.create_actor_model()

		self.actor_critic_grad = tf.placeholder(tf.float32,
			[None, self.env.action_space.shape[0]])

		actor_model_weights = self.actor_model.trainable_weights
		self.actor_grads = tf.gradients(self.actor_model.output,
			actor_model_weights, -self.actor_critic_grad) 
		grads = zip(self.actor_grads, actor_model_weights)
		self.optimize = tf.train.AdamOptimizer(self.learning_rate).apply_gradients(grads)


		self.critic_state_input, self.critic_action_input, \
			self.critic_model = self.create_critic_model()
		_, _, self.target_critic_model = self.create_critic_model()

		self.critic_grads = tf.gradients(self.critic_model.output,
			self.critic_action_input) 

		self.sess.run(tf.initialize_all_variables())

	def create_actor_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		h1 = Dense(500, activation='relu')(state_input)
		#h2 = Dense(1000, activation='relu')(h1)
		h2 = Dense(500, activation='relu')(h1)
		h3 = Dense(500, activation='relu')(h2)
		delta_theta = Dense(1, activation='tanh')(h3) 
		speed = Dense(1, activation='sigmoid')(h3) 
		output = Concatenate()([delta_theta, speed])
		model = Model(input=state_input, output=output)
		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, model

	def create_critic_model(self):
		state_input = Input(shape=self.env.observation_space.shape)
		state_h1 = Dense(500, activation='relu')(state_input)
		#state_h2 = Dense(1000)(state_h1)

		action_input = Input(shape=self.env.action_space.shape)
		action_h1    = Dense(500)(action_input)

		merged    = Concatenate()([state_h1, action_h1])
		merged_h1 = Dense(500, activation='relu')(merged)
		merged_h2 = Dense(500, activation='relu')(merged_h1)
		output = Dense(1, activation='linear')(merged_h2)
		model  = Model(input=[state_input,action_input], output=output)

		adam  = Adam(lr=0.0001)
		model.compile(loss="mse", optimizer=adam)
		return state_input, action_input, model

	def remember(self, cur_state, action, reward, new_state, done):
		self.memory.append([cur_state, action, reward, new_state, done])

	def _train_critic_actor(self, samples):
		cur_states, actions, rewards, new_states, dones= stack_samples(samples)
		target_actions = self.target_actor_model.predict(new_states)
		future_rewards = self.target_critic_model.predict([new_states, target_actions])
		rewards = rewards + self.gamma* future_rewards * (1 - dones)
		evaluation = self.critic_model.fit([cur_states, actions], rewards, verbose=0)
		predicted_actions = self.actor_model.predict(cur_states)
		grads = self.sess.run(self.critic_grads, feed_dict={
			self.critic_state_input:  cur_states,
			self.critic_action_input: predicted_actions
		})[0]

		self.sess.run(self.optimize, feed_dict={
			self.actor_state_input: cur_states,
			self.actor_critic_grad: grads
		})

	def read_Q_values(self, cur_states, actions):
		critic_values = self.critic_model.predict([cur_states, actions])
		return critic_values

	def train(self):
		batch_size = self.batch_size
		if len(self.memory) < batch_size: 
			return
		samples = random.sample(self.memory, batch_size)    
		self.samples = samples
		print("length of memory is %s", len(self.memory))
		self._train_critic_actor(samples)

	def _update_actor_target(self):
		actor_model_weights  = self.actor_model.get_weights()
		actor_target_weights = self.target_actor_model.get_weights()
		
		for i in range(len(actor_target_weights)):
			actor_target_weights[i] = actor_model_weights[i]*self.tau + actor_target_weights[i]*(1-self.tau)
		self.target_actor_model.set_weights(actor_target_weights)

	def _update_critic_target(self):
		critic_model_weights  = self.critic_model.get_weights()
		critic_target_weights = self.target_critic_model.get_weights()
		
		for i in range(len(critic_target_weights)):
			critic_target_weights[i] = critic_model_weights[i]*self.tau + critic_target_weights[i]*(1-self.tau)
		self.target_critic_model.set_weights(critic_target_weights)

	def update_target(self):
		self._update_actor_target()
		self._update_critic_target()

	def act(self, cur_state):  
		self.epsilon *= self.epsilon_decay
		eps = self.epsilon
		action = self.actor_model.predict(cur_state)
		if np.random.random() < self.epsilon:
			action[0][0] = action[0][0] + (np.random.random()-0.5)*0.4
			action[0][1] = action[0][1] + np.random.random()*0.4
			return action, eps	
		else:
			action[0][0] = action[0][0] 
			action[0][1] = action[0][1]
			return action, eps
		
	def save_weight(self, num_trials, trial_len):
		self.actor_model.save_weights('actormodel' + '-' +  str(num_trials) + '-' + str(trial_len) + 'small.h5', overwrite=True)
		self.critic_model.save_weights('criticmodel' + '-' + str(num_trials) + '-' + str(trial_len) + 'small.h5', overwrite=True)

	def play(self, cur_state):
		return self.actor_model.predict(cur_state)


def main(test_no=0,noise=0,policy_id=1000):
	
	sess = tf.Session()
	K.set_session(sess)
	game_state= ddpg_functions.GameState() 
	actor_critic = ActorCritic(game_state, sess)
	num_trials = 3
	trial_len  = 1000
	train_indicator = 0
	# test_no = str('TEST2')

	# current_state = game_state.reset()

	# actor_critic.read_human_data()
	
	step_reward = [0,0]
	step_Q = [0,0]
	step = 0

	if (train_indicator==1):
		for i in range(num_trials):
			print("trial:" + str(i))
			current_state = game_state.reset()
			total_reward = 0
			
			for j in range(trial_len):
				current_state = current_state.reshape((1, game_state.observation_space.shape[0]))
				action, eps = actor_critic.act(current_state)
				action = action.reshape((1, game_state.action_space.shape[0]))
				print("action is speed: %s, angular: %s", action[0][1], action[0][0])
				reward, new_state, crashed_value, level = game_state.game_step(0.1, action[0][1], action[0][0],
																			   noise_level=noise * 0.1)
				total_reward = total_reward + reward
				if j == (trial_len - 1):
					crashed_value = 1
					print("this is reward:", total_reward)
					print('eps is', eps)
				if crashed_value > 0:
					break

				step = step + 1
				step_reward = np.append(step_reward,[step,reward])
				sio.savemat('step_reward.mat',{'data':step_reward},True,'5', False, False,'row')
				print("step is %s", step)

				Q_values = actor_critic.read_Q_values(current_state, action)
				step_Q = np.append(step_Q,[step,Q_values[0][0]])
				print("step_Q is %s", Q_values[0][0])
				sio.savemat('step_Q.mat',{'data':step_Q},True,'5', False, False,'row')

				start_time = time.time()

				if (j % 5 == 0):
					actor_critic.train()
					actor_critic.update_target()   

				end_time = time.time()
				print("train time is %s", (end_time - start_time))
				
				new_state = new_state.reshape((1, game_state.observation_space.shape[0]))
				actor_critic.remember(current_state, action, reward, new_state, crashed_value)
				current_state = new_state

			if (i % 10==0):
				actor_critic.save_weight(i, trial_len)

		

	if train_indicator==0:
		os.mkdir(test_no)
		os.mkdir(test_no + '/path')
		os.mkdir(test_no + '/failure_level')
		os.mkdir(test_no + '/action')
		os.mkdir(test_no + '/state')
		os.mkdir(test_no + '/crash')
		crash_record = list()
		for i in range(num_trials):
			print("trial:" + str(i))
			current_state = game_state.reset()
			
			actor_critic.actor_model.load_weights("weights/actormodel-"+str(policy_id)+"-1000small.h5")
			actor_critic.critic_model.load_weights("weights/criticmodel-"+str(policy_id)+"-1000small.h5")
			total_reward = 0
			xypoints = list()
			failure_level_record = list()
			action_record = list()
			state_record = list()
			noise_level = round(abs(np.random.normal(0,noise,1)),1)
			for j in range(trial_len):
				current_state = current_state.reshape((1, game_state.observation_space.shape[0]))
				start_time = time.time()
				action = actor_critic.play(current_state) 
				action = action.reshape((1, game_state.action_space.shape[0]))
				end_time = time.time()
				print(1/(end_time - start_time), "fps for calculating next step")

				reward, new_state, crashed_value, level = game_state.game_step(0.1, action[0][1], action[0][0],noise_level=noise_level*0.1) 
				total_reward = total_reward + reward
				xypoints = np.append(xypoints,[game_state.position.x,game_state.position.y])
				failure_level_record = np.append(failure_level_record, level)
				action_record = np.append(action_record, action)
				state_record = np.append(state_record, current_state)
				if j == (trial_len - 1):
					crashed_value = 1
					print("this is reward:", total_reward)

				if crashed_value > 0:
					break

				
				new_state = new_state.reshape((1, game_state.observation_space.shape[0]))
				current_state = new_state



			path_data = np.array(xypoints)
			np.savetxt(str(test_no)+'/path/' + 'path-' + str(i) + '.csv',path_data)
			level_data = np.array(failure_level_record)
			np.savetxt(str(test_no)+'/failure_level/' + 'failure_level-' + str(i) + '.csv', level_data)
			action_data = np.array(action_record)
			np.savetxt(str(test_no)+'/action/' + 'action-' + str(i) + '.csv', action_data)
			state_data = np.array(state_record)
			np.savetxt(str(test_no)+'/state/' + 'state-' + str(i) + '.csv', state_data)
			crash_record = np.append(crash_record,crashed_value)

		np.savetxt(str(test_no)+'/crash/' + 'crash-' + str(i) + '.csv', crash_record)


if __name__ == "__main__":
	# main()
	for i in range(30):
		test_no = str('TESTpolicy') + str(i) + str('_original')
		noise = 0
		policy_id = i*30
		main(test_no, noise, policy_id)


	# for i in range(20):
	# 	test_no = str('testetsetset-') + str(i+1) + str('_original')
	# 	noise = i+1
	# 	main(test_no,noise)

	# i = 5
	# test_no = str('addTEST-') + str(i) + str('_original')
	# noise = i
	# main(test_no, noise)

# if a >2 and a <8:
# 	print("it works")
# else:
# 	print("no work")
