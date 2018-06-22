#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from learning_pkg.msg import Learn

from keras.models import Sequential
from keras.layers import Dense, Activation

comm_msg = Learn()
comm_msg.instruction = "py2c:waiting4action?"

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

def commCallback(msg):

	global scan
	
	if msg.instruction == "c2py:waiting4action":
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
		
		epsilon = numpy.random.binomial(n=1, p=0.8, size=None)
	
		if epsilon == 0: # explore random action
			msg.instruction = "py2c:check4action"
			msg.action = random.randint(0,action_dim)
			msg.state = scan
			msg.reward = float('nan')
			commPub.publish(msg)
		
		else: # choose the best action
			model0_out = model0.predict(state)
			model1_out = model1.predict(state)
			model2_out = model2.predict(state)
		
			temp_array = [model0_out, model0_out, model0_out]
			
			msg.instruction = "py2c:check4action"
			msg.action = temp_array.index(max(temp_array))
			msg.state = scan
			msg.reward = float('nan')
			commPub.publish(msg)
		
	elif msg.instruction == "c2py:check4reward":

		model0_out = model0.predict(msg.state)
		model1_out = model1.predict(msg.state)
		model2_out = model2.predict(msg.state)
		
		Q_star = max(model0_out, model0_out, model0_out)
		
		Q = msg.reward + gamma*(Q_star)
		
		#global action
		if msg.action == 0:
			model0.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		elif msg.action == 1:
			model1.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		elif msg.action == 2:
			model2.fit(msg.state, Q, epochs=150, batch_size=10, verbose=2)
		
		#Publish Instruction / Acknowledgement
		msg.instruction = "py2c:waiting4action?"
		commPub.publish(msg)
		
def scanCallback(msg):
	
	global scan
	scan = msg.ranges

def main ():

	rospy.init_node('learn_node', anonymous=True)
	
	commPub = rospy.Publisher("local_communication", Learn, queue_size=10)
	
	rospy.Subscriber("local_communication", Learn, commCallback)
	rospy.Subscriber("scan", LaserScan, scanCallback)
	
	rate = rospy.Rate(5) # 10hz
	
	global bugHim
	
	while not rospy.is_shutdown():
		if comm_msg.instruction == "py2c:waiting4action?":
			commPub.publish(comm_msg)
		rate.sleep()
		#rospy.spinOnce()
	
if __name__ == '__main__':
	main()

