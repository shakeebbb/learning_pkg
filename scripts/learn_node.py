#!/usr/bin/env python

import rospy
import numpy
import random
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from learning_pkg.srv import *

from keras.models import Sequential
from keras.layers import Dense, Activation

comm_msg = Learn()
comm_msg.instruction = "py2c:waiting4action?"

seed = 7
numpy.random.seed(seed)

gamma = 1  # discount factor
	
nFeatures = 640

action_dim = 4

# Model 0
model0 = Sequential()
model0.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model0.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model0.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model0.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 1
model1 = Sequential()
model1.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model1.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model1.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model1.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 2
model2 = Sequential()
model2.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model2.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model2.add(Dense(1, kernel_initializer='uniform', activation='sigmoid'))

model2.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

print("NN model initialized ...")

def commCallback():

	global scan
	global state
	global reward

	try:
		scan
	except NameError:
		print "Waiting for sensor information..."
		return
	
	rospy.wait_for_service('action2reward')
	
	try:
		clientHandle = rospy.ServiceProxy('action2reward', Learn)	
					
		epsilon = numpy.random.binomial(n=1, p=0.8, size=None)
	
		if epsilon == 0: # explore random action
			instruction = "py2c:check4action"
			action = random.randint(0,action_dim)
			state = scan
			
			response = clientHandle(instruction, action)
			reward = response.reward
			
			print("Reward : %f", reward)
			
			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			Q_star = max(model0_out, model1_out, model2_out)
		
			Q = reward + gamma*(Q_star)
		
			if response.action == 0:
				model0.fit(state, Q, epochs=1, batch_size=10, verbose=2)
			elif response.action == 1:
				model1.fit(state, Q, epochs=1, batch_size=10, verbose=2)
			elif response.action == 2:
				model2.fit(state, Q, epochs=1, batch_size=10, verbose=2)

			return

		else: # choose the best action

			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			temp_array = [model0_out, model0_out, model0_out]
			
			instruction = "py2c:check4action"
			action = temp_array.index(max(temp_array))
			state = scan
			
			response = clientHandle(instruction, action)
			reward = response.reward
			
			print("Reward : %f", reward)
			
			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			Q_star = max(model0_out, model1_out, model2_out)
		
			Q = reward + gamma*(Q_star)
		
			if response.action == 0:
				model0.fit(state, Q, epochs=1, batch_size=10, verbose=2)
			elif response.action == 1:
				model1.fit(state, Q, epochs=1, batch_size=10, verbose=2)
			elif response.action == 2:
				model2.fit(state, Q, epochs=1, batch_size=10, verbose=2)

			return
				
	except rospy.ServiceException as e:
			print "Service call failed: %s"%e	
		
def scanCallback(msg):
	
	global scan	
	scan = numpy.transpose(msg.ranges.reshape((640, 1)))


def main ():

	rospy.init_node('learn_node', anonymous=True)

	rospy.Subscriber("scan", numpy_msg(LaserScan), scanCallback)
	
	rate = rospy.Rate(5) # 10hz
	
	while not rospy.is_shutdown():
		commCallback()
		#if comm_msg.instruction == "py2c:waiting4action?":
		#	commPub.publish(comm_msg)
		#rate.sleep()
		#rospy.spinOnce()
	
if __name__ == '__main__':
	main()

