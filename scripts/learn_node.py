#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from keras.models import Sequential
from keras.layers import Dense, Activation

scan = LaserScan();

seed = 7
numpy.random.seed(seed)

input_dim = 8
action_dim = 4
gamma = 1  # discount factor

# Model 0
model0 = Sequential()
model0.add(Dense(12, input_dim=8, init='uniform', activation='relu'))
model0.add(Dense(8, init='uniform', activation='relu'))
model0.add(Dense(1, init='uniform', activation='sigmoid'))

model0.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 1
model1 = Sequential()
model1.add(Dense(12, input_dim=8, init='uniform', activation='relu'))
model1.add(Dense(8, init='uniform', activation='relu'))
model1.add(Dense(1, init='uniform', activation='sigmoid'))

model1.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Model 2
model2 = Sequential()
model2.add(Dense(12, input_dim=8, init='uniform', activation='relu'))
model2.add(Dense(8, init='uniform', activation='relu'))
model2.add(Dense(1, init='uniform', activation='sigmoid'))

model2.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

print("NN model initialized ...")

def instructCallback(msg):

	print scan
	
	if msg.data == "c2py:waiting4action":
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
		
	epsilon = numpy.random.binomial(n=1, p=0.8, size=None);
	
	if epsilon == 0 # explore random action
		global action = random.randint(0,action_dim)
		#Publish Action
		#Publish Instruction
		
	else # choose the best action
		model0_out = model0.predict(state)
		model1_out = model1.predict(state)
		model2_out = model2.predict(state)
		
		temp_array = [model0_out, model0_out, model0_out]
		global action = temp_array.index(max(temp_array))
		
		global state = scan
		
		#Publish Action
		#Publish Instruction
		
	if msg.data == "c2py:check4reward"
	
		# fill global reward i.e. r = ...
		model0_out = model0.predict(state)
		model1_out = model1.predict(state)
		model2_out = model2.predict(state)
		
		Q* = max(model0_out, model0_out, model0_out)
		
		Q = r + gamma*(Q*)
		
		if global action == 0
		model0.fit(state, Q, epochs=150, batch_size=10, verbose=2)
		elif global action == 1
		model1.fit(state, Q, epochs=150, batch_size=10, verbose=2)
		elif global action == 2
		model2.fit(state, Q, epochs=150, batch_size=10, verbose=2)
		
		#Publish Instruction / Acknowledgement
		
		
def scanCallback(msg):
	
	global scan
	scan = msg.ranges

def listener ():

	rospy.init_node('learn_node', anonymous=True)
	
	rospy.Subscriber("local_instruct", String, instructCallback)
	rospy.Subscriber("scan", LaserScan, scanCallback)
	
	rospy.spin()
	
	
if __name__ == '__main__':

	listener()

