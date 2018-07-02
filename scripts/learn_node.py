#!/usr/bin/env python

import rospy
import numpy
import random
import math
import warnings
import time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from learning_pkg.srv import *

from keras.models import Sequential
from keras.layers import Dense, Activation

comm_msg = Learn()
comm_msg.instruction = "py2c:waiting4action?"

#seed = 7
#numpy.random.seed(seed)

isUpdated = 0

gamma = 0.2  # discount factor
	
nFeatures = 640

action_dim = 3

# Model 0
model0 = Sequential()
model0.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model0.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model0.add(Dense(6, kernel_initializer='uniform', activation='relu'))
model0.add(Dense(1, kernel_initializer='uniform', activation='linear'))

model0.compile(loss='mean_squared_error', optimizer='sgd', metrics=['accuracy'])
# Model 1
model1 = Sequential()
model1.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model1.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model1.add(Dense(6, kernel_initializer='uniform', activation='relu'))
model1.add(Dense(1, kernel_initializer='uniform', activation='linear'))

model1.compile(loss='mean_squared_error', optimizer='sgd', metrics=['accuracy'])
# Model 2
model2 = Sequential()
model2.add(Dense(12, activation='relu', kernel_initializer='uniform', input_dim=nFeatures))
model2.add(Dense(8, kernel_initializer='uniform', activation='relu'))
model2.add(Dense(6, kernel_initializer='uniform', activation='relu'))
model2.add(Dense(1, kernel_initializer='uniform', activation='linear'))

model2.compile(loss='mean_squared_error', optimizer='sgd', metrics=['accuracy'])

print("NN model initialized ...")

def commCallback():

	global scan
	global state
	global reward
	global features

	try:
		scan
	except NameError:
		print "Waiting for sensor information..."
		return
	
	rospy.wait_for_service('action2reward')
	
	try:
		clientHandle = rospy.ServiceProxy('action2reward', Learn)	
					
		epsilon = numpy.random.binomial(n=1, p=0.5, size=None)
	
		if epsilon == 0: # explore random action
					
			instruction = "py2c:check4action"
			action = random.randint(0,action_dim-1)
			state = scan
			
			pub.publish(features)
			
			response = clientHandle(instruction, action)
			reward = response.reward
			
			if response.instruction == "c2py:ready4reset":
				del state
				del reward
				return
				
			#print (scan)
			print ("Explored random action : ", action)				
			print("Reward : ", reward)
			
			pub.publish(features)
			
			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			Q_star = max(model0_out, model1_out, model2_out)
		
			Q = reward + gamma*(Q_star)
		
			if response.action == 0:
				model0.fit(state, Q, epochs=1, batch_size=1, verbose=2)
			elif response.action == 1:
				model1.fit(state, Q, epochs=1, batch_size=1, verbose=2)
			elif response.action == 2:
				model2.fit(state, Q, epochs=1, batch_size=1, verbose=2)

			time.sleep(2)
			#raw_input()
			return

		else: # choose the best action
			
			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
		
			temp_array = [model0_out, model1_out, model2_out]
			
			#print("Q(s,a) (predicted) = max[", numpy.asarray(temp_array), "]")
			
			instruction = "py2c:check4action"
			action = temp_array.index(max(temp_array))
			state = scan
								
			pub.publish(features)
			
			response = clientHandle(instruction, action)
			reward = response.reward
			
			if response.instruction == "c2py:ready4reset":
				del state
				del reward
				return
			
			#print (scan)
			print ("Chosen best action : ", action)
			print("Reward = ", reward)
			
			pub.publish(features)
			
			model0_out = model0.predict(scan)
			model1_out = model1.predict(scan)
			model2_out = model2.predict(scan)
			
			Q_star = max(model0_out, model1_out, model2_out)
			
			Q = reward + gamma*(Q_star)
			
			if response.action == 0:
				model0.fit(state, Q, epochs=1, batch_size=1, verbose=2)
			elif response.action == 1:
				model1.fit(state, Q, epochs=1, batch_size=1, verbose=2)
			elif response.action == 2:
				model2.fit(state, Q, epochs=1, batch_size=1, verbose=2)
				
			print("Q(s,a) (calculated) = ", Q)
			#print("Q(s',a*) = max [", model0_out, model1_out, model2_out, "]")
			
			time.sleep(2)
			#raw_input()
			return
				
	except rospy.ServiceException as e:
			print "Service call failed: %s"%e	
		
def scanCallback(msg):
	
	global scan	
	global features
	
	#msg.ranges.setflags(write=1)
	#msg.ranges[numpy.argwhere(numpy.isnan(msg.ranges))] = 0.0
	
	msg.ranges = numpy.nan_to_num(msg.ranges)
	
	scan = numpy.transpose(msg.ranges.reshape((640, 1)))

	features = msg
	
	#if numpy.isnan(numpy.sum(scan)):
		#print('Non-numeric data found in the file.')
	#	scan.fill(10)
		
	#else:
		
	#	scan[numpy.isnan(scan)]=numpy.nanmax(scan)
		
	#	scan = (scan - numpy.min(scan))/numpy.ptp(scan)
	
	#if numpy.all((numpy.isnan(scan))):
	#	scan.fill(1)
	
	#else:
	
		##print('Raw : ',scan)
		
		#scan[numpy.isnan(scan)]=numpy.nanmax(scan)
	
		#scan = (scan - numpy.min(scan))/numpy.ptp(scan)
	
		##scan = scan / numpy.nanmean(scan) # Use a mask to mark the NaNs
	
	##print(scan)


def main ():

	rospy.init_node('learn_node', anonymous=True)

	rospy.Subscriber("scan", numpy_msg(LaserScan), scanCallback)
	global pub 
	pub = rospy.Publisher("features", numpy_msg(LaserScan))
	
	rate = rospy.Rate(5) # 10hz
	
	while not rospy.is_shutdown():
		commCallback()
		#if comm_msg.instruction == "py2c:waiting4action?":
		#	commPub.publish(comm_msg)
		#rate.sleep()
		#rospy.spinOnce()
	
if __name__ == '__main__':
	main()

